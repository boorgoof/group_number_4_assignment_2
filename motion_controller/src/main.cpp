#include "motion_controller/robot_manipulator.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<RobotManipulator>(options);
  node->init_moveit();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
