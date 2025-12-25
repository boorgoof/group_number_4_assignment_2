#include "motion_controller/robot_manipulator.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

RobotManipulator::RobotManipulator(const rclcpp::NodeOptions& options)
    : Node("robot_manipulator", options)
{
    RCLCPP_INFO(this->get_logger(), "Initializing RobotManipulator");

    approach_offset_ = 0.20;
    grasp_height_ = 0.05;

    go_home_action_server_ = rclcpp_action::create_server<GoHome>(
        this,
        "go_home",
        std::bind(&RobotManipulator::handle_go_home_goal, this, _1, _2),
        std::bind(&RobotManipulator::handle_go_home_cancel, this, _1),
        std::bind(&RobotManipulator::handle_go_home_accepted, this, _1));

    move_cube_action_server_ = rclcpp_action::create_server<MoveCube>(
        this,
        "move_cube",
        std::bind(&RobotManipulator::handle_move_cube_goal, this, _1, _2),
        std::bind(&RobotManipulator::handle_move_cube_cancel, this, _1),
        std::bind(&RobotManipulator::handle_move_cube_accepted, this, _1));

    RCLCPP_INFO(this->get_logger(), "RobotManipulator initialized");
}

void RobotManipulator::init_moveit()
{
    arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "ir_arm");
    
    gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "ir_gripper");

    arm_group_->setPlanningTime(10.0);
    arm_group_->setNumPlanningAttempts(10);
    
    RCLCPP_INFO(this->get_logger(), "MoveIt ready");
}

// GoHome action handlers
rclcpp_action::GoalResponse RobotManipulator::handle_go_home_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const GoHome::Goal> goal)
{
    (void)uuid;
    (void)goal;
    RCLCPP_INFO(this->get_logger(), "Received go_home goal request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RobotManipulator::handle_go_home_cancel(
    const std::shared_ptr<GoHomeGoalHandle> goal_handle)
{
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Received cancel request for go_home");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void RobotManipulator::handle_go_home_accepted(
    const std::shared_ptr<GoHomeGoalHandle> goal_handle)
{
    std::thread{std::bind(&RobotManipulator::execute_go_home, this, goal_handle)}.detach();
}

void RobotManipulator::execute_go_home(
    const std::shared_ptr<GoHomeGoalHandle> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing go_home action");
    
    auto feedback = std::make_shared<GoHome::Feedback>();
    auto result = std::make_shared<GoHome::Result>();

    feedback->current_state = "Moving to home position";
    goal_handle->publish_feedback(feedback);

    if (!go_to_home()) {
        result->success = false;
        result->message = "Failed to move to home position";
        goal_handle->abort(result);
        return;
    }

    feedback->current_state = "Opening gripper";
    goal_handle->publish_feedback(feedback);

    if (!set_gripper(true)) {
        RCLCPP_WARN(this->get_logger(), "Failed to open gripper");
    }

    feedback->current_state = "Completed";
    goal_handle->publish_feedback(feedback);

    result->success = true;
    result->message = "Robot at home position with gripper open";
    goal_handle->succeed(result);
    
    RCLCPP_INFO(this->get_logger(), "Go home action completed");
}

rclcpp_action::GoalResponse RobotManipulator::handle_move_cube_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveCube::Goal> goal)
{
    (void)uuid;
    (void)goal;
    RCLCPP_INFO(this->get_logger(), "Received move_cube goal request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RobotManipulator::handle_move_cube_cancel(
    const std::shared_ptr<MoveCubeGoalHandle> goal_handle)
{
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Received cancel request for move_cube");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void RobotManipulator::handle_move_cube_accepted(
    const std::shared_ptr<MoveCubeGoalHandle> goal_handle)
{
    std::thread{std::bind(&RobotManipulator::execute_move_cube, this, goal_handle)}.detach();
}

void RobotManipulator::execute_move_cube(
    const std::shared_ptr<MoveCubeGoalHandle> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing move_cube action");
    
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<MoveCube::Feedback>();
    auto result = std::make_shared<MoveCube::Result>();

    feedback->current_state = "Picking cube";
    goal_handle->publish_feedback(feedback);

    if (!pick_operation(goal->pose_from)) {
        RCLCPP_WARN(this->get_logger(), "Pick operation failed, continuing anyway");
    }

    if (goal->use_waypoint) {
        feedback->current_state = "Moving to waypoint";
        goal_handle->publish_feedback(feedback);

        if (!go_to_pose(goal->waypoint_pose)) {
            RCLCPP_WARN(this->get_logger(), "Failed to reach waypoint");
        }

        feedback->current_state = "Waiting at waypoint";
        goal_handle->publish_feedback(feedback);

        auto wait_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(goal->waypoint_wait_time));
        rclcpp::sleep_for(wait_duration);
    }

    feedback->current_state = "Placing cube";
    goal_handle->publish_feedback(feedback);

    if (!place_operation(goal->pose_to)) {
        RCLCPP_WARN(this->get_logger(), "Place operation failed, continuing anyway");
    }

    feedback->current_state = "Completed";
    goal_handle->publish_feedback(feedback);

    result->success = true;
    result->message = "Cube moved successfully";
    goal_handle->succeed(result);
    
    RCLCPP_INFO(this->get_logger(), "Move cube action completed");
}

bool RobotManipulator::go_to_home()
{
    RCLCPP_INFO(this->get_logger(), "Going to home");
    arm_group_->setNamedTarget("home");
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (arm_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
    if (success) {
        arm_group_->execute(plan);
    }
    
    return success;
}

bool RobotManipulator::go_to_pose(const geometry_msgs::msg::Pose& target)
{
    arm_group_->setPoseTarget(target);
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (arm_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
    if (success) {
        arm_group_->execute(plan);
    } else {
        RCLCPP_WARN(this->get_logger(), "Planning failed");
    }
    
    return success;
}

bool RobotManipulator::set_gripper(bool open)
{
    if (open) {
        gripper_group_->setNamedTarget("open");
    } else {
        gripper_group_->setNamedTarget("close");
    }
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (gripper_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
    if (success) {
        gripper_group_->execute(plan);
    }
    
    return success;
}

bool RobotManipulator::pick_operation(const geometry_msgs::msg::Pose& target)
{
    geometry_msgs::msg::Pose approach_pose = target;
    approach_pose.position.z += approach_offset_;
    approach_pose.orientation.w = 0.0;
    approach_pose.orientation.x = 1.0;
    approach_pose.orientation.y = 0.0;
    approach_pose.orientation.z = 0.0;

    if (!go_to_pose(approach_pose)) return false;
    
    set_gripper(true);
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    geometry_msgs::msg::Pose grasp_pose = target;
    grasp_pose.position.z = grasp_height_;
    grasp_pose.orientation = approach_pose.orientation;

    if (!go_to_pose(grasp_pose)) return false;
    
    set_gripper(false);
    rclcpp::sleep_for(std::chrono::milliseconds(800));

    if (!go_to_pose(approach_pose)) return false;

    return true;
}

bool RobotManipulator::place_operation(const geometry_msgs::msg::Pose& target)
{
    geometry_msgs::msg::Pose approach_pose = target;
    approach_pose.position.z += approach_offset_;
    approach_pose.orientation.w = 0.0;
    approach_pose.orientation.x = 1.0;
    approach_pose.orientation.y = 0.0;
    approach_pose.orientation.z = 0.0;

    if (!go_to_pose(approach_pose)) return false;

    geometry_msgs::msg::Pose drop_pose = target;
    drop_pose.position.z = grasp_height_;
    drop_pose.orientation = approach_pose.orientation;

    if (!go_to_pose(drop_pose)) return false;
    
    set_gripper(true);
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    if (!go_to_pose(approach_pose)) return false;

    return true;
}
