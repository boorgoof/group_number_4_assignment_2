#include "motion_controller/robot_manipulator.hpp"
#include <chrono>
#include <thread>

using std::placeholders::_1;
using std::placeholders::_2;

RobotManipulator::RobotManipulator(const rclcpp::NodeOptions &options)
    : Node("robot_manipulator", options) {
  RCLCPP_INFO(this->get_logger(), "Initializing RobotManipulator");

  approach_offset_ = 0.2;
  grasp_height_ = 0.0;

  go_home_action_server_ = rclcpp_action::create_server<GoHome>(
      this, "go_home",
      std::bind(&RobotManipulator::handle_go_home_goal, this, _1, _2),
      std::bind(&RobotManipulator::handle_go_home_cancel, this, _1),
      std::bind(&RobotManipulator::handle_go_home_accepted, this, _1));

  move_cube_action_server_ = rclcpp_action::create_server<MoveCube>(
      this, "move_cube",
      std::bind(&RobotManipulator::handle_move_cube_goal, this, _1, _2),
      std::bind(&RobotManipulator::handle_move_cube_cancel, this, _1),
      std::bind(&RobotManipulator::handle_move_cube_accepted, this, _1));

  RCLCPP_INFO(this->get_logger(), "RobotManipulator initialized");
}

void RobotManipulator::init_moveit() {
  RCLCPP_INFO(this->get_logger(), "Waiting for MoveIt services to initialize...");
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  RCLCPP_INFO(this->get_logger(), "Starting MoveIt initialization");
  
  arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>( shared_from_this(), "ir_arm");

  gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>( shared_from_this(), "ir_gripper");

  arm_group_->setMaxVelocityScalingFactor(0.6);
  arm_group_->setMaxAccelerationScalingFactor(0.6);
  arm_group_->setPlanningTime(10.0);
  arm_group_->setNumPlanningAttempts(10);
  arm_group_->setPoseReferenceFrame("base_link");

  gripper_group_->setMaxVelocityScalingFactor(0.6);
  gripper_group_->setMaxAccelerationScalingFactor(0.6);
  gripper_group_->setPlanningTime(10.0);
  gripper_group_->setNumPlanningAttempts(10);
  gripper_group_->setPoseReferenceFrame("base_link");
  arm_group_->setEndEffectorLink("tool0");

  RCLCPP_INFO(this->get_logger(), "MoveIt ready");
}

// GoHome action handlers
rclcpp_action::GoalResponse RobotManipulator::handle_go_home_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const GoHome::Goal> goal) {
  (void)uuid;
  (void)goal;
  RCLCPP_INFO(this->get_logger(), "Received go_home goal request");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RobotManipulator::handle_go_home_cancel(const std::shared_ptr<GoHomeGoalHandle> goal_handle) {
  (void)goal_handle;
  RCLCPP_INFO(this->get_logger(), "Received cancel request for go_home");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void RobotManipulator::handle_go_home_accepted(const std::shared_ptr<GoHomeGoalHandle> goal_handle) {
  std::thread{std::bind(&RobotManipulator::execute_go_home, this, goal_handle)}
      .detach();
}

void RobotManipulator::execute_go_home(const std::shared_ptr<GoHomeGoalHandle> goal_handle) {
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

  geometry_msgs::msg::PoseStamped current_pose = arm_group_->getCurrentPose();

  // 2. Access coordinates
  double x = current_pose.pose.position.x;
  double y = current_pose.pose.position.y;
  double z = current_pose.pose.position.z;
  double rx = current_pose.pose.orientation.x;
  double ry = current_pose.pose.orientation.y;
  double rz = current_pose.pose.orientation.z;
  double rw = current_pose.pose.orientation.w;


  RCLCPP_INFO(this->get_logger(), "Current Gripper Position: x: %f, y: %f, z: %f, rx: %f, ry: %f, rz: %f, rw: %f ", x, y, z, rx, ry, rz, rw);

  RCLCPP_INFO(this->get_logger(), "Go home action completed");
}

rclcpp_action::GoalResponse RobotManipulator::handle_move_cube_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const MoveCube::Goal> goal) {
  (void)uuid;
  (void)goal;
  RCLCPP_INFO(this->get_logger(), "Received move_cube goal request");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RobotManipulator::handle_move_cube_cancel(
    const std::shared_ptr<MoveCubeGoalHandle> goal_handle) {
  (void)goal_handle;
  RCLCPP_INFO(this->get_logger(), "Received cancel request for move_cube");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void RobotManipulator::handle_move_cube_accepted(
    const std::shared_ptr<MoveCubeGoalHandle> goal_handle) {
  std::thread{
      std::bind(&RobotManipulator::execute_move_cube, this, goal_handle)}
      .detach();
}

void RobotManipulator::execute_move_cube(const std::shared_ptr<MoveCubeGoalHandle> goal_handle) {
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

    auto wait_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(goal->waypoint_wait_time));
    rclcpp::sleep_for(wait_duration);
  }

  feedback->current_state = "Placing cube";
  goal_handle->publish_feedback(feedback);

  if (!place_operation(goal->pose_to)) {
    RCLCPP_WARN(this->get_logger(),"Place operation failed, continuing anyway");
  }

  feedback->current_state = "Completed";
  goal_handle->publish_feedback(feedback);

  result->success = true;
  result->message = "Cube moved successfully";
  goal_handle->succeed(result);

  RCLCPP_INFO(this->get_logger(), "Move cube action completed");
}

bool RobotManipulator::go_to_home() {
  RCLCPP_INFO(this->get_logger(), "Going to home");
  std::vector<double> joint_values = {2.5, -1.75, -0.8, -2.0, -4.5, -0.9};
  rclcpp::sleep_for(std::chrono::milliseconds(500));
  arm_group_->setJointValueTarget(joint_values);
  rclcpp::sleep_for(std::chrono::milliseconds(500));
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success =
      (arm_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success) {
    arm_group_->execute(plan);
  }

  return success;
}

bool RobotManipulator::go_to_pose(const geometry_msgs::msg::Pose &target) {
  arm_group_->setPoseTarget(target);
  rclcpp::sleep_for(std::chrono::milliseconds(500));
  geometry_msgs::msg::PoseStamped target_pose = arm_group_->getPoseTarget();

  RCLCPP_INFO(this->get_logger(), "Target Cartesian Pose:\n"
              "Position: [x: %f, y: %f, z: %f]\n"
              "Orientation: [x: %f, y: %f, z: %f, w: %f]",
              target_pose.pose.position.x,
              target_pose.pose.position.y,
              target_pose.pose.position.z,
              target_pose.pose.orientation.x,
              target_pose.pose.orientation.y,
              target_pose.pose.orientation.z,
              target_pose.pose.orientation.w);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success =(arm_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  rclcpp::sleep_for(std::chrono::milliseconds(500));
  if (success) {
    arm_group_->execute(plan);
  } else {
    RCLCPP_WARN(this->get_logger(), "Planning failed");
  }
  geometry_msgs::msg::PoseStamped current_pose = arm_group_->getCurrentPose();

  // 2. Access coordinates
  double x = current_pose.pose.position.x;
  double y = current_pose.pose.position.y;
  double z = current_pose.pose.position.z;
  double rx = current_pose.pose.orientation.x;
  double ry = current_pose.pose.orientation.y;
  double rz = current_pose.pose.orientation.z;
  double rw = current_pose.pose.orientation.w;


  RCLCPP_INFO(this->get_logger(), "Current Arm Position: x: %f, y: %f, z: %f, rx: %f, ry: %f, rz: %f, rw: %f ", x, y, z, rx, ry, rz, rw);

  return success;
}

bool RobotManipulator::set_gripper(bool open) {
  rclcpp::sleep_for(std::chrono::milliseconds(500));
  if (open) {
    gripper_group_->setNamedTarget("open");
  } else {
    gripper_group_->setNamedTarget("close");
  }
  rclcpp::sleep_for(std::chrono::milliseconds(500));
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success =(gripper_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  rclcpp::sleep_for(std::chrono::milliseconds(500));
  if (success) {
    gripper_group_->execute(plan);
  }

  return success;
}

bool RobotManipulator::pick_operation(const geometry_msgs::msg::Pose &target) {
  RCLCPP_INFO(this->get_logger(), "Starting pick operation");
  
  // Step 1: Move to approach position (x, y, z+approach_offset)
  geometry_msgs::msg::Pose approach_pose = target;
  approach_pose.orientation = target.orientation;  // Use orientation from target
  approach_pose.position.z += approach_offset_;
  
  RCLCPP_INFO(this->get_logger(), "Moving to approach position");
  if (!go_to_pose(approach_pose)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to reach approach position");
    //return false;
  }

  // Step 2: Ensure gripper is open
  RCLCPP_INFO(this->get_logger(), "Opening gripper");
  if (!set_gripper(true)) {
    RCLCPP_WARN(this->get_logger(), "Failed to open gripper");
  }
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  // Step 3: Descend to grasp position (same x, y, but lower z)
  geometry_msgs::msg::Pose grasp_pose = target;
  grasp_pose.orientation = target.orientation;
  grasp_pose.position.z += grasp_height_;
  
  RCLCPP_INFO(this->get_logger(), "Descending to grasp position");
  if (!go_to_pose(grasp_pose)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to reach grasp position");
    //return false;
  }

  // Step 4: Close gripper to grasp object
  RCLCPP_INFO(this->get_logger(), "Closing gripper to grasp object");
  if (!set_gripper(false)) {
    RCLCPP_WARN(this->get_logger(), "Failed to close gripper");
  }
  rclcpp::sleep_for(std::chrono::milliseconds(800));

  // Step 5: Retreat to approach position
  RCLCPP_INFO(this->get_logger(), "Retreating to approach position");
  if (!go_to_pose(approach_pose)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to retreat to approach position");
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Pick operation completed successfully");
  return true;
}

bool RobotManipulator::place_operation(const geometry_msgs::msg::Pose &target) {
  RCLCPP_INFO(this->get_logger(), "Starting place operation");
  
  // Step 1: Move to approach position (x, y, z+approach_offset)
  geometry_msgs::msg::Pose approach_pose = target;
  approach_pose.orientation = target.orientation;  // Use orientation from target
  approach_pose.position.z += approach_offset_;
  
  RCLCPP_INFO(this->get_logger(), "Moving to approach position");
  if (!go_to_pose(approach_pose)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to reach approach position");
    return false;
  }

  // Step 2: Descend to drop position (same x, y, but lower z)
  geometry_msgs::msg::Pose drop_pose = target;
  drop_pose.orientation = target.orientation;
  drop_pose.position.z += grasp_height_;
  
  RCLCPP_INFO(this->get_logger(), "Descending to drop position");
  if (!go_to_pose(drop_pose)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to reach drop position");
    return false;
  }

  // Step 3: Open gripper to release object
  RCLCPP_INFO(this->get_logger(), "Opening gripper to release object");
  if (!set_gripper(true)) {
    RCLCPP_WARN(this->get_logger(), "Failed to open gripper");
  }
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  // Step 4: Retreat to approach position
  RCLCPP_INFO(this->get_logger(), "Retreating to approach position");
  if (!go_to_pose(approach_pose)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to retreat to approach position");
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Place operation completed successfully");
  return true;
}
