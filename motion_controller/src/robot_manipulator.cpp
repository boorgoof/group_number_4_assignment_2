#include "motion_controller/robot_manipulator.hpp"

RobotManipulator::RobotManipulator(const rclcpp::NodeOptions& options)
    : Node("robot_manipulator", options)
{
    RCLCPP_INFO(this->get_logger(), "Initializing RobotManipulator");

    using namespace std::placeholders;
    swap_service_ = this->create_service<motion_controller::srv::SwapCubes>(
        "swap_cubes",
        std::bind(&RobotManipulator::swap_callback, this, _1, _2));

    RCLCPP_INFO(this->get_logger(), "RobotManipulator initialized");
}

void RobotManipulator::init_moveit()
{
    // TODO: Initialize MoveGroupInterface for arm and gripper
    arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "ir_arm");
    
    gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "ir_gripper");
    
    RCLCPP_INFO(this->get_logger(), "MoveIt ready");
}

void RobotManipulator::swap_callback(
    const std::shared_ptr<motion_controller::srv::SwapCubes::Request> request,
    std::shared_ptr<motion_controller::srv::SwapCubes::Response> response)
{
    // TODO: Implement cube swap logic
    RCLCPP_INFO(this->get_logger(), "Swap service called");
    
    response->success = false;
    response->message = "Not implemented yet";
}
