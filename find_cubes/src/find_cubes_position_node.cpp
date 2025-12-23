#include "../include/find_cubes_position_node.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>

FindCubesPositionNode::FindCubesPositionNode() : Node("find_cubes_position_node")
{
    RCLCPP_INFO(this->get_logger(), "find_cubes_position_node started");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
    qos.transient_local();
    qos.reliable();

    cubes_pub_ = this->create_publisher<find_cubes::msg::CubesPoses>("/cubes_poses", qos);

    detections_sub_ = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
        "/apriltag/detections",
        rclcpp::SensorDataQoS(),
        [this](const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
        {
            this->cubes_pos_callback(msg);
        });

    has_previous_ = false;
    position_threshold_ = 0.01;
    orientation_threshold_ = 0.05;
}


void FindCubesPositionNode::cubes_pos_callback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
{
    if (msg->detections.size() != 2) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Apriltags detected: %zu (we want = 2)", msg->detections.size());
        return;
    }

    const int id0 = msg->detections[0].id;
    const int id1 = msg->detections[1].id;

    if (!((id0 == 1 && id1 == 10) || (id0 == 10 && id1 == 1))) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Unexpected tag IDs: [%d, %d]. Expected {1,10}", id0, id1);
        return;
    }

    geometry_msgs::msg::PoseStamped red_pose;
    geometry_msgs::msg::PoseStamped blue_pose;

    //todo modify with colors
    const std::string frame_red  = tag_frame_prefix_ + "1";
    const std::string frame_blue = tag_frame_prefix_ + "10";

    if (!get_pose_in_frame(base_link_frame_, frame_red, msg->header.stamp, red_pose)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "TF lookup failed: %s -> %s", base_link_frame_.c_str(), frame_red.c_str());
        return;
    }
    if (!get_pose_in_frame(base_link_frame_, frame_blue, msg->header.stamp, blue_pose)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "TF lookup failed: %s -> %s", base_link_frame_.c_str(), frame_blue.c_str());
        return;
    }

    find_cubes::msg::CubesPoses out;
    out.header.stamp = now();
    out.header.frame_id = base_link_frame_;

    out.ids = {1, 10};
    out.poses.clear();
    out.poses.push_back(red_pose.pose);
    out.poses.push_back(blue_pose.pose);

    if (!has_previous_ || poses_changed(previous_, out)) {
        cubes_pub_->publish(out);
        previous_ = out;
        has_previous_ = true;

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "Published /cubes_poses: red(%.3f,%.3f) blue(%.3f,%.3f)",
            out.poses[0].position.x, out.poses[0].position.y,
            out.poses[1].position.x, out.poses[1].position.y);
    }
}

bool FindCubesPositionNode::get_pose_in_frame(const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & stamp, geometry_msgs::msg::PoseStamped & out_pose)
{
    geometry_msgs::msg::TransformStamped tf;
    try {
        tf = tf_buffer_->lookupTransform(
            target_frame,
            source_frame,
            stamp,
            tf2::durationFromSec(0.1));
    } catch (const tf2::TransformException &) {
        return false;
    }

    out_pose.header.stamp = stamp;
    out_pose.header.frame_id = target_frame;
    out_pose.pose.position.x = tf.transform.translation.x;
    out_pose.pose.position.y = tf.transform.translation.y;
    out_pose.pose.position.z = tf.transform.translation.z;
    out_pose.pose.orientation = tf.transform.rotation;

    return true;
}

bool FindCubesPositionNode::poses_changed(const find_cubes::msg::CubesPoses & prev, const find_cubes::msg::CubesPoses & now_msg) const
{
    if (prev.poses.size() != now_msg.poses.size())
        return true;

    for (size_t i = 0; i < prev.poses.size(); ++i)
    {
        const auto & a = prev.poses[i];
        const auto & b = now_msg.poses[i];

        const double dx = a.position.x - b.position.x;
        const double dy = a.position.y - b.position.y;
        const double dz = a.position.z - b.position.z;
        const double pos = std::sqrt(dx * dx + dy * dy + dz * dz);

        const double oz = std::abs(a.orientation.z - b.orientation.z);
        const double ow = std::abs(a.orientation.w - b.orientation.w);
        const double orient = oz + ow;

        if (pos > position_threshold_ || orient > orientation_threshold_)
            return true;
    }

    return false;
}