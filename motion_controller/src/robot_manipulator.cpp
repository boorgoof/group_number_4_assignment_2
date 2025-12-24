#include "motion_controller/robot_manipulator.hpp"

RobotManipulator::RobotManipulator(const rclcpp::NodeOptions& options)
    : Node("robot_manipulator", options)
{
    RCLCPP_INFO(this->get_logger(), "Initializing RobotManipulator");

    approach_offset_ = 0.20;
    grasp_height_ = 0.05;

    using namespace std::placeholders;
    swap_service_ = this->create_service<motion_controller::srv::SwapCubes>(
        "swap_cubes",
        std::bind(&RobotManipulator::swap_callback, this, _1, _2));

    RCLCPP_INFO(this->get_logger(), "RobotManipulator initialized, waiting for MoveIt setup...");
}

void RobotManipulator::init_moveit()
{
    arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "ir_arm");
    
    gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "ir_gripper");

    arm_group_->setPlanningTime(10.0);
    arm_group_->setNumPlanningAttempts(10);
    
    RCLCPP_INFO(this->get_logger(), "RobotManipulator ready");
}

void RobotManipulator::swap_callback(
    const std::shared_ptr<motion_controller::srv::SwapCubes::Request> request,
    std::shared_ptr<motion_controller::srv::SwapCubes::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Swap service called");
    //we have to understand a good position
    geometry_msgs::msg::Pose safe_pose;
    safe_pose.position.x = 0.0;
    safe_pose.position.y = 0.0;
    safe_pose.position.z = 0.0;
    safe_pose.orientation.w = 0.0;
    safe_pose.orientation.x = 0.0;
    safe_pose.orientation.y = 0.0;
    safe_pose.orientation.z = 0.0;

    // 1. Home
    if (!go_to_home()) {
        response->success = false;
        response->message = "Failed to go home";
        return;
    }

    // 2. Pick A
    if (!pick_operation(request->pose_a)) {
        RCLCPP_WARN(this->get_logger(), "Pick A failed, continuing anyway");
    }

    // 3. Place A at buffer
    if (!place_operation(safe_pose)) {
        RCLCPP_WARN(this->get_logger(), "Place buffer failed, continuing");
    }

    // 4. Pick B
    if (!pick_operation(request->pose_b)) {
        RCLCPP_WARN(this->get_logger(), "Pick B failed, continuing anyway");
    }

    // 5. Place B at A
    if (!place_operation(request->pose_a)) {
        RCLCPP_WARN(this->get_logger(), "Place A failed, continuing");
    }

    // 6. Pick A from buffer
    if (!pick_operation(safe_pose)) {
        RCLCPP_WARN(this->get_logger(), "Pick buffer failed, continuing");
    }

    // 7. Place A at B
    if (!place_operation(request->pose_b)) {
        RCLCPP_WARN(this->get_logger(), "Place B failed, continuing");
    }

    // 8. Home
    if (!go_to_home()) {
        response->success = false;
        response->message = "Failed to return home";
        return;
    }

    response->success = true;
    response->message = "Swap completed";
    RCLCPP_INFO(this->get_logger(), "Swap operation completed");
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
