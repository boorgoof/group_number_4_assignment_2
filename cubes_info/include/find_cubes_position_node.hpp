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
#include "cubes_info/msg/cubes_poses.hpp"


class FindCubesPositionNode : public rclcpp::Node
{
public:
  FindCubesPositionNode();

private:

    // Subscriber callback
    void cubes_pos_callback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg);

    // helpers
    bool get_pose_frame(const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & stamp, geometry_msgs::msg::PoseStamped & out_pose);
    bool poses_changed( const cubes_info::msg::CubesPoses & prev, const cubes_info::msg::CubesPoses & now_msg) const;
    
    // Subscriber
    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detections_sub_;

    // TF2
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Publisher
    rclcpp::Publisher<cubes_info::msg::CubesPoses>::SharedPtr cubes_pub_;

    // Frames 
    std::string tag_frame_prefix_ = "tag36h11:";
    std::string base_link_frame_ = "base_link";

    // Previous poses
    cubes_info::msg::CubesPoses previous_;
    bool has_previous_ = false;

    // Thresholds
    double position_threshold_ = 0.01;
    double orientation_threshold_ = 0.05;
};

#endif  // FIND_CUBES_POSITION_NODE_HPP_