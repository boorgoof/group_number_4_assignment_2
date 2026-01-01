#include <memory>
#include "rclcpp/rclcpp.hpp"

#include "find_cubes_position_node.hpp"
#include "detect_color_node.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto cubes_node  = std::make_shared<FindCubesPositionNode>();
    auto color_node  = std::make_shared<DetectColorNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(cubes_node);
    executor.add_node(color_node);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}
