#ifndef BEHAVIOUR_TREE_HPP
#define BEHAVIOUR_TREE_HPP

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "motion_controller/action/move_cube.hpp"
#include "motion_controller/action/go_home.hpp"
#include "motion_controller/action/reset.hpp"
#include "cubes_info/msg/cubes_poses.hpp"
#include "cubes_info/action/detect_color.hpp"


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
    MoveCubeAction(const std::string& name, const NodeConfig& config, rclcpp::Node::SharedPtr node);

    static PortsList providedPorts() {
        return { 
            InputPort<geometry_msgs::msg::Pose>("from"),
            InputPort<geometry_msgs::msg::Pose>("to"),
            InputPort<bool>("use_waypoint"),
            InputPort<geometry_msgs::msg::Pose>("waypoint_pose"),
            InputPort<float>("waypoint_wait_time")
        };
    }

    NodeStatus onStart() override;

    NodeStatus onRunning() override;

    void onHalted() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<motion_controller::action::MoveCube>::SharedPtr client_;
    bool done_ = false;
    bool success_ = false;
};

class GoHomeAction : public StatefulActionNode {
public:
    GoHomeAction(const std::string& name, const NodeConfig& config, rclcpp::Node::SharedPtr node);

    static PortsList providedPorts() { return {}; }

    NodeStatus onStart() override;

    NodeStatus onRunning() override;
    void onHalted() override {}

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<motion_controller::action::GoHome>::SharedPtr client_;
    bool done_, success_;
};


class ResetAction : public StatefulActionNode {
public:
    ResetAction(const std::string& name, const NodeConfig& config, rclcpp::Node::SharedPtr node);

    static PortsList providedPorts() { return {}; }

    NodeStatus onStart() override;

    NodeStatus onRunning() override;
    void onHalted() override {}

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<motion_controller::action::Reset>::SharedPtr client_;
    bool done_, success_;
};


class GetCubesPoses : public StatefulActionNode {
public:
    GetCubesPoses(const std::string& name, const NodeConfig& config, rclcpp::Node::SharedPtr node);

    static PortsList providedPorts() {
        return { 
            OutputPort<geometry_msgs::msg::Pose>("cube1_pose"), 
            OutputPort<geometry_msgs::msg::Pose>("cube2_pose"),
            InputPort<double>("settling_time", 3.0, "Time to wait for cubes to land") 
        };
    }

    NodeStatus onStart() override;

    NodeStatus onRunning() override;

    void onHalted() override {}

    static constexpr int id_cube1 = 1, id_cube2 = 10;
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<cubes_info::msg::CubesPoses>::SharedPtr sub_;
    cubes_info::msg::CubesPoses::SharedPtr last_msg_;
    rclcpp::Time start_time_;
    double settling_time_;
};


class GetCubeColorAction : public StatefulActionNode {
public:
    GetCubeColorAction(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node);

    static PortsList providedPorts() {
        return { 
            InputPort<geometry_msgs::msg::Pose>("cube_pose"),
            InputPort<int>("id"),
            OutputPort<std::string>("color")
        };
    }

    NodeStatus onStart() override;
    NodeStatus onRunning() override;

    void onHalted() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<cubes_info::action::DetectColor>::SharedPtr client_;
    
    bool done_ = false;
    std::string detected_color_;
    rclcpp_action::ResultCode status_code_;
    int id_;
};


class DisplayCubeInfo : public SyncActionNode {
public:
    DisplayCubeInfo(const std::string& name, const NodeConfig& config, rclcpp::Node::SharedPtr node)
        : SyncActionNode(name, config), node_(node) {}

    static PortsList providedPorts() {
        return {
            InputPort<int>("id"), InputPort<geometry_msgs::msg::Pose>("pose"), InputPort<std::string>("color"),
        };
    }

    NodeStatus tick() override;

private:
    void printCube(int id, const geometry_msgs::msg::Pose& p, const std::string& c);
    rclcpp::Node::SharedPtr node_;
};


#endif