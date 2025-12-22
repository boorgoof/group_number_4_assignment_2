#ifndef FIND_CUBES_POSITION_NODE_HPP_
#define FIND_CUBES_POSITION_NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "apriltag_msgs/msg/april_tag_detection_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class FindCubesPositionNode : public rclcpp::Node
{
public:
  FindCubesPositionNode();

private:

  // Subscriber callback
  void cubes_pos_callback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg);

  // helpers
  bool apriltag_pose_wrt_odom(const std::string & tag_frame,const rclcpp::Time & stamp, geometry_msgs::msg::PoseStamped & out_pose);
  bool has_pose_changed(const geometry_msgs::msg::PoseStamped & prev_pose, const geometry_msgs::msg::PoseStamped & new_pose);

  // Subscriber
  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detections_sub_;

  // TF2
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Publishers 
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr red_pose_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr blue_pose_publisher_;

  // Frames 
  std::string tag_frame_prefix_ = "tag36h11:";
  std::string odom_frame_ = "odom";

  // Previous poses
  geometry_msgs::msg::PoseStamped previous_red_cube_pose_;
  geometry_msgs::msg::PoseStamped previous_blue_cube_pose_;
  bool has_previous_pose_red_cube_ = false;
  bool has_previous_pose_blue_cube_ = false;

  // Thresholds
  double position_threshold_ = 0.01;
  double orientation_threshold_ = 0.05;
};

#endif  // FIND_CUBES_POSITION_NODE_HPP_