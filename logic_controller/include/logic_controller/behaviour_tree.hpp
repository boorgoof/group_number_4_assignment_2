#ifndef BEHAVIOUR_TREE_HPP
#define BEHAVIOUR_TREE_HPP

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "motion_controller/action/move_cube.hpp"
#include "motion_controller/action/go_home.hpp"
#include "find_cubes/msg/cubes_poses.hpp"

namespace BT
{
    template<>
    inline geometry_msgs::msg::Pose convertFromString(StringView str)
    {
        // Expected format: "x;y;z;qx;qy;qz;qw"
        auto parts = splitString(str, ';');
        if (parts.size() != 7) {
            throw RuntimeError("Invalid Pose format. Expected 7 values separated by ';'");
        }
        geometry_msgs::msg::Pose pose;
        pose.position.x = convertFromString<double>(parts[0]);
        pose.position.y = convertFromString<double>(parts[1]);
        pose.position.z = convertFromString<double>(parts[2]);
        pose.orientation.x = convertFromString<double>(parts[3]);
        pose.orientation.y = convertFromString<double>(parts[4]);
        pose.orientation.z = convertFromString<double>(parts[5]);
        pose.orientation.w = convertFromString<double>(parts[6]);
        return pose;
    }
}

using namespace BT;

class MoveCubeAction : public StatefulActionNode {
public:
    MoveCubeAction(const std::string& name, const NodeConfig& config, rclcpp::Node::SharedPtr node)
        : StatefulActionNode(name, config), node_(node) {
        client_ = rclcpp_action::create_client<motion_controller::action::MoveCube>(node_, "move_cube");
    }

    static PortsList providedPorts() {
        return { 
            InputPort<geometry_msgs::msg::Pose>("from"),
            InputPort<geometry_msgs::msg::Pose>("to"),
            InputPort<bool>("use_waypoint"),
            InputPort<geometry_msgs::msg::Pose>("waypoint_pose"),
            InputPort<float>("waypoint_wait_time")
        };
    }

    NodeStatus onStart() override {
        if (!client_->wait_for_action_server(std::chrono::seconds(2))) return NodeStatus::FAILURE;

        auto goal = motion_controller::action::MoveCube::Goal();

        if (!getInput("from", goal.pose_from) || !getInput("to", goal.pose_to)) {
            return NodeStatus::FAILURE;
        }
        
        goal.use_waypoint = false;
        getInput("from", goal.pose_from);
        getInput("to", goal.pose_to);
        getInput("use_waypoint", goal.use_waypoint);

        if (goal.use_waypoint){
            getInput("waypoint_pose", goal.waypoint_pose);
            getInput("waypoint_wait_time", goal.waypoint_wait_time);
        }
        

        auto send_goal_options = rclcpp_action::Client<motion_controller::action::MoveCube>::SendGoalOptions();
        send_goal_options.result_callback = [this](const auto& result) { 
            done_ = true; 
            success_ = result.result->success;
        };

        client_->async_send_goal(goal, send_goal_options);
        done_ = false;
        return NodeStatus::RUNNING;
    }

    NodeStatus onRunning() override {
        if (!done_) return NodeStatus::RUNNING;
        return success_ ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
    }

    void onHalted() override { client_->async_cancel_all_goals(); }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<motion_controller::action::MoveCube>::SharedPtr client_;
    bool done_ = false;
    bool success_ = false;
};

class GoHomeAction : public StatefulActionNode {
public:
    GoHomeAction(const std::string& name, const NodeConfig& config, rclcpp::Node::SharedPtr node)
        : StatefulActionNode(name, config), node_(node) {
        client_ = rclcpp_action::create_client<motion_controller::action::GoHome>(node_, "go_home");
    }

    static PortsList providedPorts() { return {}; }

    NodeStatus onStart() override {
        if (!client_->wait_for_action_server(std::chrono::seconds(2))) {
            return NodeStatus::FAILURE;
        }

        auto goal = motion_controller::action::GoHome::Goal();
        
        auto send_goal_options = rclcpp_action::Client<motion_controller::action::GoHome>::SendGoalOptions();
        
        send_goal_options.result_callback = [this](const auto& result) { 
            done_ = true; 
            success_ = result.result->success;
        };

        client_->async_send_goal(goal, send_goal_options);
        done_ = false;
        return NodeStatus::RUNNING;
    }

    NodeStatus onRunning() override { return done_ ? (success_ ? NodeStatus::SUCCESS : NodeStatus::FAILURE) : NodeStatus::RUNNING; }
    void onHalted() override {}

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<motion_controller::action::GoHome>::SharedPtr client_;
    bool done_, success_;
};

class GetCubesPoses : public SyncActionNode {
public:
    GetCubesPoses(const std::string& name, const NodeConfig& config, rclcpp::Node::SharedPtr node)
        : SyncActionNode(name, config) {
        sub_ = node->create_subscription<find_cubes::msg::CubesPoses>(
            "/cubes_poses", 10, [this](const find_cubes::msg::CubesPoses::SharedPtr msg) { last_msg_ = msg; });
    }

    static PortsList providedPorts() {
        return { OutputPort<geometry_msgs::msg::Pose>("cube1_pose"), OutputPort<geometry_msgs::msg::Pose>("cube2_pose") };
    }

    NodeStatus tick() override {
        if (!last_msg_) return NodeStatus::FAILURE;
        for (size_t i = 0; i < last_msg_->ids.size(); ++i) {
            if (last_msg_->ids[i] == id_cube1) setOutput("cube1_pose", last_msg_->poses[i]);
            if (last_msg_->ids[i] == id_cube2) setOutput("cube2_pose", last_msg_->poses[i]);
        }
        return NodeStatus::SUCCESS;
    }

    static constexpr int id_cube1 = 1, id_cube2 = 10;
private:
    rclcpp::Subscription<find_cubes::msg::CubesPoses>::SharedPtr sub_;
    find_cubes::msg::CubesPoses::SharedPtr last_msg_;
};

#endif