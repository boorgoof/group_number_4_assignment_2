cmake_minimum_required(VERSION 3.8)
project(find_cubes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_executable(find_cubes_nodes
  src/find_cubes_position_node.cpp
  src/main.cpp
)

ament_target_dependencies(find_cubes_nodes
  rclcpp
  sensor_msgs
  apriltag_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS
  find_cubes_nodes
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  cfg
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
