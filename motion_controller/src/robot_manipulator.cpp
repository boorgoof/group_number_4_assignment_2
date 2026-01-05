#include "motion_controller/robot_manipulator.hpp"
#include <chrono>
#include <thread>

using std::placeholders::_1;
using std::placeholders::_2;

RobotManipulator::RobotManipulator(const rclcpp::NodeOptions &options)
    : Node("robot_manipulator", options) {
  RCLCPP_INFO(this->get_logger(), "Initializing RobotManipulator");

  approach_offset_ = 0.22;
  grasp_height_ = 0.15;

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

  reset_action_server_ = rclcpp_action::create_server<Reset>(
      this, "reset",
      std::bind(&RobotManipulator::handle_reset_goal, this, _1, _2),
      std::bind(&RobotManipulator::handle_reset_cancel, this, _1),
      std::bind(&RobotManipulator::handle_reset_accepted, this, _1));

  gripper_client_ = rclcpp_action::create_client<GripperCommand>(
      this, "/gripper_controller/gripper_cmd");

  RCLCPP_INFO(this->get_logger(), "RobotManipulator initialized");
}

void RobotManipulator::init_moveit() {
  RCLCPP_INFO(this->get_logger(), "Waiting for MoveIt services to initialize...");
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  RCLCPP_INFO(this->get_logger(), "Starting MoveIt initialization");
  
  arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>( shared_from_this(), "ir_arm");

  gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>( shared_from_this(), "ir_gripper");

  arm_group_->setMaxVelocityScalingFactor(0.2);
  arm_group_->setMaxAccelerationScalingFactor(0.2);
  arm_group_->setPlanningTime(10.0);
  arm_group_->setNumPlanningAttempts(10);
  arm_group_->setPoseReferenceFrame("base_link");

  gripper_group_->setMaxVelocityScalingFactor(0.2);
  gripper_group_->setMaxAccelerationScalingFactor(0.2);
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

  if (!set_gripper_action(0.0, 10.0)) {
    RCLCPP_WARN(this->get_logger(), "Failed to open gripper");
  }

  feedback->current_state = "Completed";
  goal_handle->publish_feedback(feedback);

  result->success = true;
  result->message = "Robot at home position with gripper open";
  goal_handle->succeed(result);

  geometry_msgs::msg::PoseStamped current_pose = arm_group_->getCurrentPose();

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

// Reset action handlers
rclcpp_action::GoalResponse RobotManipulator::handle_reset_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const Reset::Goal> goal) {
  (void)uuid;
  (void)goal;
  RCLCPP_INFO(this->get_logger(), "Received reset goal request");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RobotManipulator::handle_reset_cancel(
    const std::shared_ptr<ResetGoalHandle> goal_handle) {
  (void)goal_handle;
  RCLCPP_INFO(this->get_logger(), "Received cancel request for reset");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void RobotManipulator::handle_reset_accepted(
    const std::shared_ptr<ResetGoalHandle> goal_handle) {
  std::thread{
      std::bind(&RobotManipulator::execute_reset, this, goal_handle)}
      .detach();
}

void RobotManipulator::execute_reset(const std::shared_ptr<ResetGoalHandle> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Executing reset action");

  auto feedback = std::make_shared<Reset::Feedback>();
  auto result = std::make_shared<Reset::Result>();

  feedback->current_state = "Moving to initial position";
  goal_handle->publish_feedback(feedback);

  if (!reset()) {
    result->success = false;
    result->message = "Failed to move to initial position";
    goal_handle->abort(result);
    return;
  }

  feedback->current_state = "Completed";
  goal_handle->publish_feedback(feedback);

  result->success = true;
  result->message = "Robot Reset";
  goal_handle->succeed(result);

  RCLCPP_INFO(this->get_logger(), "Reset action completed");
}

bool RobotManipulator::go_to_home() {
  RCLCPP_INFO(this->get_logger(), "Going to home");
  std::vector<double> joint_values = {2.5, -1.75, -0.8, -2.0, -4.6, 1.0};
  arm_group_->setJointValueTarget(joint_values);
  moveit::planning_interface::MoveGroupInterface::Plan plan;

  bool success = (arm_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success) {
    arm_group_->execute(plan);
  }

  return success;
}

bool RobotManipulator::reset() {
  RCLCPP_INFO(this->get_logger(), "Going to home position");

  arm_group_->setNamedTarget("home");
  moveit::planning_interface::MoveGroupInterface::Plan plan;

  bool success = (arm_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success) {
    arm_group_->execute(plan);
  }

  return success;
}

bool RobotManipulator::go_to_pose(const geometry_msgs::msg::Pose &target) {
  arm_group_->setPoseTarget(target);
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success =(arm_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  rclcpp::sleep_for(std::chrono::milliseconds(500));
  if (success) {
    arm_group_->execute(plan);
  } else {
    RCLCPP_WARN(this->get_logger(), "Planning failed");
  }
  geometry_msgs::msg::PoseStamped current_pose = arm_group_->getCurrentPose();

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

bool RobotManipulator::cartesian_move_vertical(double z_offset) {
  RCLCPP_INFO(this->get_logger(), "Executing Cartesian vertical movement of %.3f meters", z_offset);

  geometry_msgs::msg::PoseStamped current_pose_stamped = arm_group_->getCurrentPose();
  geometry_msgs::msg::Pose current_pose = current_pose_stamped.pose;
  
  geometry_msgs::msg::Pose target_pose = current_pose;
  target_pose.position.z += z_offset;
  
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(target_pose);
  
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double eef_step = 0.001;  // 1mm resolution
  
  double fraction = arm_group_->computeCartesianPath(
    waypoints,
    eef_step,
    trajectory
  );
  
  RCLCPP_INFO(this->get_logger(), "Cartesian path computed: %.2f%% achieved", fraction * 100.0);
  
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory = trajectory;
  
  auto result = arm_group_->execute(plan);
  bool success = (result == moveit::core::MoveItErrorCode::SUCCESS);
  
  if (success) {
    RCLCPP_INFO(this->get_logger(), "Cartesian vertical movement executed successfully");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to execute Cartesian path");
  }
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

  geometry_msgs::msg::Pose approx_target = target;
  approx_target.position.x = std::round(target.position.x * 10.0) / 10.0;
  approx_target.position.y = std::round(target.position.y * 10.0) / 10.0;
  approx_target.position.z = std::round(target.position.z * 10.0) / 10.0;

  // Move to approach position
  geometry_msgs::msg::Pose approach_pose = approx_target;
  approach_pose.orientation = target.orientation;
  approach_pose.position.z += approach_offset_;
  
  RCLCPP_INFO(this->get_logger(), "Moving to approach position");
  if (!go_to_pose(approach_pose)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to reach approach position");
    return false;
  }

  // Ensure gripper is open
  RCLCPP_INFO(this->get_logger(), "Opening gripper");
  if (!set_gripper_action(0.0, 10.0)) {
    RCLCPP_WARN(this->get_logger(), "Failed to open gripper");
  }
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  // Descend to grasp position using Cartesian path
  double descend_distance = -(approach_offset_ - grasp_height_);
  
  RCLCPP_INFO(this->get_logger(), "Descending to grasp position (Cartesian)");
  if (!cartesian_move_vertical(descend_distance)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to descend to grasp position");
    return false;
  }

  // Close gripper to grasp object
  RCLCPP_INFO(this->get_logger(), "Closing gripper to grasp object");
  if (!set_gripper_action(0.8, 10.0)) {  // 0.8 = close, 0.0 = open 
    RCLCPP_WARN(this->get_logger(), "Failed to close gripper");
  }
  rclcpp::sleep_for(std::chrono::milliseconds(800));

  // Retreat to approach position using Cartesian path
  double ascend_distance = approach_offset_ - grasp_height_;
  
  RCLCPP_INFO(this->get_logger(), "Retreating to approach position (Cartesian)");
  if (!cartesian_move_vertical(ascend_distance)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to retreat to approach position");
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Pick operation completed successfully");
  return true;
}

bool RobotManipulator::place_operation(const geometry_msgs::msg::Pose &target) {
  RCLCPP_INFO(this->get_logger(), "Starting place operation");

  geometry_msgs::msg::Pose approx_target = target;
  approx_target.position.x = std::round(target.position.x * 10.0) / 10.0;
  approx_target.position.y = std::round(target.position.y * 10.0) / 10.0;
  approx_target.position.z = std::round(target.position.z * 10.0) / 10.0;
  
  // Move to approach position
  geometry_msgs::msg::Pose approach_pose = approx_target;
  approach_pose.orientation = target.orientation;
  approach_pose.position.z += approach_offset_;
  
  RCLCPP_INFO(this->get_logger(), "Moving to approach position");
  if (!go_to_pose(approach_pose)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to reach approach position");
    return false;
  }

  // Descend to drop position using Cartesian path
  double descend_distance = -(approach_offset_ - grasp_height_);
  
  RCLCPP_INFO(this->get_logger(), "Descending to drop position (Cartesian)");
  if (!cartesian_move_vertical(descend_distance)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to descend to drop position");
    return false;
  }

  // Open gripper to release object
  RCLCPP_INFO(this->get_logger(), "Opening gripper to release object");
  if (!set_gripper_action(0.0, 10.0)) {
    RCLCPP_WARN(this->get_logger(), "Failed to open gripper");
  }
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  // Retreat to approach position using Cartesian path
  double ascend_distance = approach_offset_ - grasp_height_;
  
  RCLCPP_INFO(this->get_logger(), "Retreating to approach position (Cartesian)");
  if (!cartesian_move_vertical(ascend_distance)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to retreat to approach position");
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Place operation completed successfully");
  return true;
}

bool RobotManipulator::set_gripper_action(double position, double max_effort) {
  RCLCPP_INFO(this->get_logger(), "=== set_gripper_action called: position=%f, max_effort=%f ===", position, max_effort);
  
  // Wait for the action server to be available
  RCLCPP_INFO(this->get_logger(), "Waiting for gripper action server...");
  if (!gripper_client_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_WARN(this->get_logger(), "Gripper action server NOT available, falling back to MoveIt control");
    // MoveIt gripper control
    // Gripper positions: 0.0 = open, 0.8 = close 
    if (position > 0.4) {
      RCLCPP_INFO(this->get_logger(), "Fallback: Closing gripper with MoveIt (position=%f > 0.4)", position);
      return set_gripper(false);
    } else {
      RCLCPP_INFO(this->get_logger(), "Fallback: Opening gripper with MoveIt (position=%f <= 0.4)", position);
      return set_gripper(true);
    }
  }

  RCLCPP_INFO(this->get_logger(), "Gripper action server IS available!");

  // Create the goal
  GripperCommand::Goal gripper_goal;
  gripper_goal.command.position = position;
  gripper_goal.command.max_effort = max_effort;

  RCLCPP_INFO(this->get_logger(), "Sending gripper goal: position=%f, effort=%f", position, max_effort);

  // Send the goal asynchronously
  auto send_goal_options = rclcpp_action::Client<GripperCommand>::SendGoalOptions();
  
  send_goal_options.result_callback = 
    [this](const GoalHandleGripper::WrappedResult & result) {
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(this->get_logger(), "Gripper action succeeded");
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(this->get_logger(), "Gripper action was aborted");
          break;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_ERROR(this->get_logger(), "Gripper action was canceled");
          break;
        default:
          RCLCPP_ERROR(this->get_logger(), "Unknown gripper action result code");
          break;
      }
    };

  auto goal_handle_future = gripper_client_->async_send_goal(gripper_goal, send_goal_options);
  
  RCLCPP_INFO(this->get_logger(), "Waiting 2 seconds for gripper action to complete...");
  rclcpp::sleep_for(std::chrono::milliseconds(2000));
  
  RCLCPP_INFO(this->get_logger(), "Gripper command sent and wait completed");
  return true;
}



