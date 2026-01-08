cmake_minimum_required(VERSION 3.8)
project(logic_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(cubes_info REQUIRED)
find_package(motion_controller REQUIRED)

include_directories(include)

add_executable(robot_logic_controller_node 
  src/robot_controller.cpp
  src/behaviour_tree.cpp
)

target_link_libraries(robot_logic_controller_node
  behaviortree_cpp::behaviortree_cpp
  ament_index_cpp::ament_index_cpp
)

ament_target_dependencies(robot_logic_controller_node
  rclcpp
  rclcpp_action
  geometry_msgs
  cubes_info 
  motion_controller
)

install(TARGETS
  robot_logic_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
