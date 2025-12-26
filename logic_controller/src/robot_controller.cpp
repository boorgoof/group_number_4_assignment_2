#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "logic_controller/behaviour_tree.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("robot_controller_bt");

    BT::BehaviorTreeFactory factory;

    factory.registerBuilder<MoveCubeAction>("MoveCube", 
        [node](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<MoveCubeAction>(name, config, node);
        });
    
    factory.registerBuilder<GoHomeAction>("GoHome", 
        [node](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<GoHomeAction>(name, config, node);
        });

    factory.registerBuilder<GetCubesPoses>("GetCubesPoses", 
        [node](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<GetCubesPoses>(name, config, node);
        });

    std::string pkg_share = ament_index_cpp::get_package_share_directory("robot_controller");
    auto tree = factory.createTreeFromFile(pkg_share + "/config/swap_cubes.xml");

    rclcpp::Rate rate(10);
    BT::NodeStatus status = BT::NodeStatus::RUNNING;

    while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
        status = tree.tickOnce();
        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}