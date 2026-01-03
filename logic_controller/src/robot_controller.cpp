#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "logic_controller/behaviour_tree.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

int main(int argc, char ** argv) {

    rclcpp::init(argc, argv);
    try{
        
        auto node = rclcpp::Node::make_shared("robot_controller_bt");
        RCLCPP_INFO(node->get_logger(), "created!");
        BT::BehaviorTreeFactory factory;

        factory.registerBuilder<MoveCubeAction>("MoveCube", 
            [node](const std::string& name, const BT::NodeConfig& config) {
                return std::make_unique<MoveCubeAction>(name, config, node);
            });
        
        factory.registerBuilder<GoHomeAction>("GoHome", 
            [node](const std::string& name, const BT::NodeConfig& config) {
                return std::make_unique<GoHomeAction>(name, config, node);
            });

        factory.registerBuilder<ResetAction>("Reset", 
            [node](const std::string& name, const BT::NodeConfig& config) {
                return std::make_unique<ResetAction>(name, config, node);
            });

        factory.registerBuilder<GetCubesPoses>("GetCubesPoses", 
            [node](const std::string& name, const BT::NodeConfig& config) {
                return std::make_unique<GetCubesPoses>(name, config, node);
            });
        factory.registerBuilder<GetCubeColorAction>("GetCubeColor", 
            [node](const std::string& name, const BT::NodeConfig& config) {
                return std::make_unique<GetCubeColorAction>(name, config, node);
            });
        factory.registerBuilder<DisplayCubeInfo>("DisplayCubeInfo", 
            [node](const std::string& name, const BT::NodeConfig& config) {
                return std::make_unique<DisplayCubeInfo>(name, config, node);
            });

        std::string pkg_share = ament_index_cpp::get_package_share_directory("logic_controller");
        auto tree = factory.createTreeFromFile(pkg_share + "/config/swap_cubes.xml");

        rclcpp::Rate rate(10);
        BT::NodeStatus status = BT::NodeStatus::RUNNING;
        RCLCPP_INFO(node->get_logger(), "running!");
        while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
            status = tree.tickOnce();
            rclcpp::spin_some(node);
            rate.sleep();
        }
        RCLCPP_INFO(node->get_logger(), "shutting down!");
    }
    catch (const std::exception& e) {
        std::cerr << "\n[CRITICAL ERROR]: " << e.what() << std::endl;
        return 1;
    }

    
    rclcpp::shutdown();
    return 0;
}