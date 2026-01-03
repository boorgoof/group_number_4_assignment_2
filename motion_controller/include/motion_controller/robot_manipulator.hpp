#ifndef ROBOT_MANIPULATOR_HPP
#define ROBOT_MANIPULATOR_HPP

#include "motion_controller/action/go_home.hpp"
#include "motion_controller/action/move_cube.hpp"
#include "motion_controller/action/reset.hpp"
#include <control_msgs/action/gripper_command.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

class RobotManipulator : public rclcpp::Node {
public:
  explicit RobotManipulator(const rclcpp::NodeOptions &options);
  void init_moveit();

private:
  using GoHome = motion_controller::action::GoHome;
  using MoveCube = motion_controller::action::MoveCube;
  using Reset = motion_controller::action::Reset;
  using GoHomeGoalHandle = rclcpp_action::ServerGoalHandle<GoHome>;
  using MoveCubeGoalHandle = rclcpp_action::ServerGoalHandle<MoveCube>;
  using ResetGoalHandle = rclcpp_action::ServerGoalHandle<Reset>;
  using GripperCommand = control_msgs::action::GripperCommand;
  using GoalHandleGripper = rclcpp_action::ClientGoalHandle<GripperCommand>;

  rclcpp_action::GoalResponse handle_go_home_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const GoHome::Goal> goal);

  rclcpp_action::CancelResponse handle_go_home_cancel(const std::shared_ptr<GoHomeGoalHandle> goal_handle);

  void handle_go_home_accepted(const std::shared_ptr<GoHomeGoalHandle> goal_handle);

  void execute_go_home(const std::shared_ptr<GoHomeGoalHandle> goal_handle);

  rclcpp_action::GoalResponse handle_move_cube_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const MoveCube::Goal> goal);

  rclcpp_action::CancelResponse handle_move_cube_cancel(const std::shared_ptr<MoveCubeGoalHandle> goal_handle);

  void handle_move_cube_accepted(const std::shared_ptr<MoveCubeGoalHandle> goal_handle);

  void execute_move_cube(const std::shared_ptr<MoveCubeGoalHandle> goal_handle);

  rclcpp_action::GoalResponse handle_reset_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const Reset::Goal> goal);

  rclcpp_action::CancelResponse handle_reset_cancel(const std::shared_ptr<ResetGoalHandle> goal_handle);

  void handle_reset_accepted(const std::shared_ptr<ResetGoalHandle> goal_handle);

  void execute_reset(const std::shared_ptr<ResetGoalHandle> goal_handle);

  bool go_to_home();
  bool reset();
  bool go_to_pose(const geometry_msgs::msg::Pose &target);
  bool cartesian_move_vertical(double z_offset);
  bool set_gripper(bool open);
  bool set_gripper_action(double position, double max_effort = 10.0);
  bool pick_operation(const geometry_msgs::msg::Pose &target);
  bool place_operation(const geometry_msgs::msg::Pose &target);

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;

  rclcpp_action::Server<GoHome>::SharedPtr go_home_action_server_;
  rclcpp_action::Server<MoveCube>::SharedPtr move_cube_action_server_;
  rclcpp_action::Server<Reset>::SharedPtr reset_action_server_;
  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_client_;

  double approach_offset_;
  double grasp_height_;
};

#endif
