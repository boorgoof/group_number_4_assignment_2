#include "../include/find_cubes_position_node.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>

FindCubesPositionNode::FindCubesPositionNode() : Node("find_cubes_position_node")
{
    RCLCPP_INFO(this->get_logger(), "find_cubes_position_node started");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    tag_frame_prefix_ = "tag36h11:";
    odom_frame_ = "odom";

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
    qos.transient_local();
    qos.reliable();

    red_pose_publisher_  = this->create_publisher<geometry_msgs::msg::PoseStamped>("/red_cube_pose", qos);
    blue_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/blue_cube_pose", qos);

    detections_sub_ = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
        "/apriltag/detections",
        rclcpp::SensorDataQoS(),
        [this](const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
        {
            this->cubes_pos_callback(msg);
        });

    has_previous_pose_red_cube_ = false;
    has_previous_pose_blue_cube_ = false;
    position_threshold_ = 0.01;
    orientation_threshold_ = 0.05;
}


void FindCubesPositionNode::cubes_pos_callback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
{
    if (msg->detections.size() < 2) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Apriltags detected: %zu (need at least 2)", msg->detections.size());
    return;
    }

    const int id0 = msg->detections[0].id;
    const int id1 = msg->detections[1].id;

    // we expect to see both tags: id=1 (red cube) and id=10 (blue cube)
    if (!((id0 == 1 && id1 == 10) || (id0 == 10 && id1 == 1))) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Unexpected tag IDs: [%d, %d]. Expected {1,10}", id0, id1);
        return;
    }

    // red_id=1, blue_id=10
    const int red_id  = 1;
    const int blue_id = 10;

    const std::string frame_red  = tag_frame_prefix_ + std::to_string(red_id);
    const std::string frame_blue = tag_frame_prefix_ + std::to_string(blue_id);

    geometry_msgs::msg::PoseStamped red_pose;
    geometry_msgs::msg::PoseStamped blue_pose;

    if (!apriltag_pose_wrt_odom(frame_red, msg->header.stamp, red_pose)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Failed TF lookup for RED frame %s", frame_red.c_str());
        return;
    }

    if (!apriltag_pose_wrt_odom(frame_blue, msg->header.stamp, blue_pose)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Failed TF lookup for BLUE frame %s", frame_blue.c_str());
        return;
    }

    
    if (!has_previous_pose_red_cube_ || has_pose_changed(previous_red_cube_pose_, red_pose)) {
        red_pose_publisher_->publish(red_pose);
        previous_red_cube_pose_ = red_pose;
        has_previous_pose_red_cube_ = true;
    }

    if (!has_previous_pose_blue_cube_ || has_pose_changed(previous_blue_cube_pose_, blue_pose)) {
        blue_pose_publisher_->publish(blue_pose);
        previous_blue_cube_pose_ = blue_pose;
        has_previous_pose_blue_cube_ = true;
    }
}

bool FindCubesPositionNode::apriltag_pose_wrt_odom(const std::string & tag_frame, const rclcpp::Time & stamp, geometry_msgs::msg::PoseStamped & out_pose)
{
  geometry_msgs::msg::TransformStamped tf_tag;
  try {
    tf_tag = tf_buffer_->lookupTransform(odom_frame_, tag_frame, stamp, tf2::durationFromSec(0.1));
  } catch (const tf2::TransformException &) {
    return false;
  }

  out_pose.header.stamp = this->now();
  out_pose.header.frame_id = odom_frame_;
  out_pose.pose.position.x = tf_tag.transform.translation.x;
  out_pose.pose.position.y = tf_tag.transform.translation.y;
  out_pose.pose.position.z = tf_tag.transform.translation.z;
  out_pose.pose.orientation = tf_tag.transform.rotation;
  return true;
}

bool FindCubesPositionNode::has_pose_changed(const geometry_msgs::msg::PoseStamped & prev, const geometry_msgs::msg::PoseStamped & now)
{
  const double dx = now.pose.position.x - prev.pose.position.x;
  const double dy = now.pose.position.y - prev.pose.position.y;
  const double dz = now.pose.position.z - prev.pose.position.z;
  const double pos_diff = std::sqrt(dx*dx + dy*dy + dz*dz);

  const double oz = std::abs(now.pose.orientation.z - prev.pose.orientation.z);
  const double ow = std::abs(now.pose.orientation.w - prev.pose.orientation.w);
  const double orient_diff = oz + ow;

  return (pos_diff > position_threshold_) || (orient_diff > orientation_threshold_);
}