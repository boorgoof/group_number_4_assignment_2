#include "logic_controller/behaviour_tree.hpp"


using namespace BT;

MoveCubeAction::MoveCubeAction(const std::string& name, const NodeConfig& config, rclcpp::Node::SharedPtr node)
    : StatefulActionNode(name, config), node_(node) {
    RCLCPP_INFO(this->node_->get_logger(), "creating MoveCubeAction");
    client_ = rclcpp_action::create_client<motion_controller::action::MoveCube>(node_, "move_cube");
}

NodeStatus MoveCubeAction::onStart() {
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

NodeStatus MoveCubeAction::onRunning() {
    // RCLCPP_INFO(this->node_->get_logger(), "running MoveCubeAction");
    if (!done_) return NodeStatus::RUNNING;
    return success_ ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

void MoveCubeAction::onHalted() {
    client_->async_cancel_all_goals();
}


GoHomeAction::GoHomeAction(const std::string& name, const NodeConfig& config, rclcpp::Node::SharedPtr node)
    : StatefulActionNode(name, config), node_(node) {
    RCLCPP_INFO(this->node_->get_logger(), "creating GoHomeAction");
    client_ = rclcpp_action::create_client<motion_controller::action::GoHome>(node_, "go_home");
}

NodeStatus GoHomeAction::onStart() {
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

NodeStatus GoHomeAction::onRunning() {
    RCLCPP_INFO(this->node_->get_logger(), "running GoHomeAction"); 
    return done_ ? (success_ ? NodeStatus::SUCCESS : NodeStatus::FAILURE) : NodeStatus::RUNNING;
}



ResetAction::ResetAction(const std::string& name, const NodeConfig& config, rclcpp::Node::SharedPtr node)
    : StatefulActionNode(name, config), node_(node) {
    RCLCPP_INFO(this->node_->get_logger(), "creating ResetAction");
    client_ = rclcpp_action::create_client<motion_controller::action::Reset>(node_, "reset");
}


NodeStatus ResetAction::onStart() {
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

NodeStatus ResetAction::onRunning() {
    RCLCPP_INFO(this->node_->get_logger(), "running ResetAction");
    return done_ ? (success_ ? NodeStatus::SUCCESS : NodeStatus::FAILURE) : NodeStatus::RUNNING;
}



GetCubesPoses::GetCubesPoses(const std::string& name, const NodeConfig& config, rclcpp::Node::SharedPtr node)
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

NodeStatus GetCubesPoses::onStart() {
    start_time_ = node_->now();
    //last_msg_ = nullptr;
    
    if (!getInput("settling_time", settling_time_)) {
        settling_time_ = 3.0;
    }
    
    RCLCPP_INFO(node_->get_logger(), "Stabilizing cubes for %.1f seconds...", settling_time_);
    return NodeStatus::RUNNING;
}

NodeStatus GetCubesPoses::onRunning() {
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


GetCubeColorAction::GetCubeColorAction(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
    : StatefulActionNode(name, config), node_(node) {
    client_ = rclcpp_action::create_client<cubes_info::action::DetectColor>(node_, "detect_color");
}


NodeStatus GetCubeColorAction::onStart() {
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

NodeStatus GetCubeColorAction::onRunning() {
    if (!done_) return NodeStatus::RUNNING;

    if (status_code_ == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(node_->get_logger(), "Successfully detected color: %s for cube %d", detected_color_.c_str(), id_);
        setOutput("color", detected_color_);
        return NodeStatus::SUCCESS;
    }
    
    RCLCPP_ERROR(node_->get_logger(), "Color detection failed for cube %d", id_);
    return NodeStatus::FAILURE;
}

void GetCubeColorAction::onHalted() {
    client_->async_cancel_all_goals();
}



NodeStatus DisplayCubeInfo::tick() {
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

void DisplayCubeInfo::printCube(int id, const geometry_msgs::msg::Pose& p, const std::string& c) {
    printf("Cube ID: %d | Color: %-10s\n", id, c.c_str());
    printf("Position: [x: %.3f, y: %.3f, z: %.3f]\n", p.position.x, p.position.y, p.position.z);
}
