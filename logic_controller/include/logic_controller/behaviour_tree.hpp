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
    MoveCubeAction(const std::string& name, const NodeConfig& config, rclcpp::Node::SharedPtr node)
        : StatefulActionNode(name, config), node_(node) {
        RCLCPP_INFO(this->node_->get_logger(), "creating MoveCubeAction");
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
        RCLCPP_INFO(this->node_->get_logger(), "starting MoveCubeAction");
        if (!client_->wait_for_action_server(std::chrono::seconds(10))) return NodeStatus::FAILURE;

        auto goal = motion_controller::action::MoveCube::Goal();

        if (!getInput("from", goal.pose_from) || !getInput("to", goal.pose_to)) {
            return NodeStatus::FAILURE;
        }
        
        goal.use_waypoint = false;
        getInput("from", goal.pose_from);
        getInput("to", goal.pose_to);
        getInput("use_waypoint", goal.use_waypoint);

        goal.pose_from.orientation.x = 1.0;
        goal.pose_from.orientation.y = 0.0;
        goal.pose_from.orientation.z = 0.0;
        goal.pose_from.orientation.w = 0.0;

        goal.pose_to.orientation.x = 1.0;
        goal.pose_to.orientation.y = 0.0;
        goal.pose_to.orientation.z = 0.0;
        goal.pose_to.orientation.w = 0.0;

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
        // RCLCPP_INFO(this->node_->get_logger(), "running MoveCubeAction");
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
        RCLCPP_INFO(this->node_->get_logger(), "creating GoHomeAction");
        client_ = rclcpp_action::create_client<motion_controller::action::GoHome>(node_, "go_home");
    }

    static PortsList providedPorts() { return {}; }

    NodeStatus onStart() override {
        RCLCPP_INFO(this->node_->get_logger(), "starting GoHomeAction");
        if (!client_->wait_for_action_server(std::chrono::seconds(10))) {
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

    NodeStatus onRunning() override { RCLCPP_INFO(this->node_->get_logger(), "running GoHomeAction"); return done_ ? (success_ ? NodeStatus::SUCCESS : NodeStatus::FAILURE) : NodeStatus::RUNNING; }
    void onHalted() override {}

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<motion_controller::action::GoHome>::SharedPtr client_;
    bool done_, success_;
};


class ResetAction : public StatefulActionNode {
public:
    ResetAction(const std::string& name, const NodeConfig& config, rclcpp::Node::SharedPtr node)
        : StatefulActionNode(name, config), node_(node) {
        RCLCPP_INFO(this->node_->get_logger(), "creating ResetAction");
        client_ = rclcpp_action::create_client<motion_controller::action::Reset>(node_, "reset");
    }

    static PortsList providedPorts() { return {}; }

    NodeStatus onStart() override {
        RCLCPP_INFO(this->node_->get_logger(), "starting ResetAction");
        if (!client_->wait_for_action_server(std::chrono::seconds(10))) {
            return NodeStatus::FAILURE;
        }

        auto goal = motion_controller::action::Reset::Goal();

        auto send_goal_options = rclcpp_action::Client<motion_controller::action::Reset>::SendGoalOptions();

        send_goal_options.result_callback = [this](const auto& result) { 
            done_ = true; 
            success_ = result.result->success;
        };

        client_->async_send_goal(goal, send_goal_options);
        done_ = false;
        return NodeStatus::RUNNING;
    }

    NodeStatus onRunning() override { RCLCPP_INFO(this->node_->get_logger(), "running ResetAction"); return done_ ? (success_ ? NodeStatus::SUCCESS : NodeStatus::FAILURE) : NodeStatus::RUNNING; }
    void onHalted() override {}

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<motion_controller::action::Reset>::SharedPtr client_;
    bool done_, success_;
};


class GetCubesPoses : public StatefulActionNode {
public:
    GetCubesPoses(const std::string& name, const NodeConfig& config, rclcpp::Node::SharedPtr node)
        : StatefulActionNode(name, config), node_(node) {
        auto sub_qos = rclcpp::QoS(rclcpp::KeepLast(1));
        sub_qos.transient_local();
        sub_qos.reliable();

        sub_ = this->node_->create_subscription<cubes_info::msg::CubesPoses>(
            "/cubes_poses", 
            sub_qos, 
            [this](const cubes_info::msg::CubesPoses::SharedPtr msg) { 
                last_msg_ = msg; 
            });
    }

    static PortsList providedPorts() {
        return { 
            OutputPort<geometry_msgs::msg::Pose>("cube1_pose"), 
            OutputPort<geometry_msgs::msg::Pose>("cube2_pose"),
            InputPort<double>("settling_time", 3.0, "Time to wait for cubes to land") 
        };
    }

    NodeStatus onStart() override {
        start_time_ = node_->now();
        //last_msg_ = nullptr;
        
        if (!getInput("settling_time", settling_time_)) {
            settling_time_ = 3.0;
        }
        
        RCLCPP_INFO(node_->get_logger(), "Stabilizing cubes for %.1f seconds...", settling_time_);
        return NodeStatus::RUNNING;
    }

    NodeStatus onRunning() override {
        auto elapsed = (node_->now() - start_time_).seconds();

        if (last_msg_) {
            for (size_t i = 0; i < last_msg_->ids.size(); ++i) {
                if (last_msg_->ids[i] == id_cube1) setOutput("cube1_pose", last_msg_->poses[i]);
                if (last_msg_->ids[i] == id_cube2) setOutput("cube2_pose", last_msg_->poses[i]);
            }
        }

        if (elapsed >= settling_time_) {
            if (last_msg_) {
                RCLCPP_INFO(node_->get_logger(), "Cubes stabilized. Proceeding with swap.");
                return NodeStatus::SUCCESS;
            } else {
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, 
                                     "Time expired but no new cubes' poses received!");
                return NodeStatus::FAILURE;
            }
        }

        return NodeStatus::RUNNING;
    }

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
    GetCubeColorAction(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
        : StatefulActionNode(name, config), node_(node) {
        client_ = rclcpp_action::create_client<cubes_info::action::DetectColor>(node_, "detect_color");
    }

    static PortsList providedPorts() {
        return { 
            InputPort<geometry_msgs::msg::Pose>("cube_pose"),
            InputPort<int>("id"),
            OutputPort<std::string>("color")
        };
    }

    NodeStatus onStart() override {
        if (!client_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_ERROR(node_->get_logger(), "Action server /detect_color not available");
            return NodeStatus::FAILURE;
        }

        if (!getInput("id", id_)) {
            RCLCPP_ERROR(node_->get_logger(), "Missing id from action tree");
            return NodeStatus::FAILURE;
        }

        auto goal = cubes_info::action::DetectColor::Goal();

        geometry_msgs::msg::Pose pose_val;
        if (!getInput("cube_pose", pose_val)) {
            RCLCPP_ERROR(node_->get_logger(), "Missing cube_pose in GetCubeColorAction");
            return NodeStatus::FAILURE;
        }

        goal.cube_pose_base.header.frame_id = "base_link";
        goal.cube_pose_base.header.stamp = node_->now();
        goal.cube_pose_base.pose = pose_val;

        auto send_goal_options = rclcpp_action::Client<cubes_info::action::DetectColor>::SendGoalOptions();
        
        send_goal_options.result_callback = [this](const auto& result) {
            done_ = true;
            status_code_ = result.code;
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                detected_color_ = result.result->color;
            }
        };

        send_goal_options.feedback_callback = [this](auto, const auto feedback) {
            RCLCPP_INFO(node_->get_logger(), "Color detection status: %s", feedback->status.c_str());
        };

        client_->async_send_goal(goal, send_goal_options);
        
        done_ = false;
        RCLCPP_INFO(node_->get_logger(), "Sent color detection request...");
        return NodeStatus::RUNNING;
    }

    NodeStatus onRunning() override {
        if (!done_) return NodeStatus::RUNNING;

        if (status_code_ == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(node_->get_logger(), "Successfully detected color: %s for cube %d", detected_color_.c_str(), id_);
            setOutput("color", detected_color_);
            return NodeStatus::SUCCESS;
        }
        
        RCLCPP_ERROR(node_->get_logger(), "Color detection failed for cube %d", id_);
        return NodeStatus::FAILURE;
    }

    void onHalted() override {
        client_->async_cancel_all_goals();
    }

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

    NodeStatus tick() override {
        int id;
        geometry_msgs::msg::Pose p;
        std::string c;

        if (!getInput("id", id) || 
            !getInput("pose", p) ||
            !getInput("color", c))
        {
            return NodeStatus::FAILURE;
        }

        std::cout << std::string(40, '=') << "\n";
        printCube(id, p, c);
        std::cout << std::string(40, '=') << "\n\n";

        return NodeStatus::SUCCESS;
    }

private:
    void printCube(int id, const geometry_msgs::msg::Pose& p, const std::string& c) {
        printf("Cube ID: %d | Color: %-10s\n", id, c.c_str());
        printf("Position: [x: %.3f, y: %.3f, z: %.3f]\n", p.position.x, p.position.y, p.position.z);
    }
    rclcpp::Node::SharedPtr node_;
};


#endif