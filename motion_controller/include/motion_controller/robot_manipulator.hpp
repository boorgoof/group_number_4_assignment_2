#ifndef ROBOT_MANIPULATOR_HPP
#define ROBOT_MANIPULATOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "motion_controller/srv/swap_cubes.hpp"

class RobotManipulator : public rclcpp::Node
{
public:
    explicit RobotManipulator(const rclcpp::NodeOptions& options);
    void init_moveit();

private:
    void swap_callback(
        const std::shared_ptr<motion_controller::srv::SwapCubes::Request> request,
        std::shared_ptr<motion_controller::srv::SwapCubes::Response> response);

    bool go_to_home();
    bool go_to_pose(const geometry_msgs::msg::Pose& target);
    bool set_gripper(bool open);
    bool pick_operation(const geometry_msgs::msg::Pose& target);
    bool place_operation(const geometry_msgs::msg::Pose& target);

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;
    
    rclcpp::Service<motion_controller::srv::SwapCubes>::SharedPtr swap_service_;
    
    double approach_offset_;
    double grasp_height_;
};

#endif
