#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include "cubes_info/action/detect_color.hpp"

class DetectColorNode : public rclcpp::Node
{
public:
    using DetectColor = cubes_info::action::DetectColor;
    using GoalHandleDetectColor = rclcpp_action::ServerGoalHandle<DetectColor>;

    DetectColorNode();

private:

    // Action callbacks
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const DetectColor::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleDetectColor> goal_handle);

    void handle_accepted(const std::shared_ptr<GoalHandleDetectColor> goal_handle);
    void execute(const std::shared_ptr<GoalHandleDetectColor> goal_handle);

    // ROS subscribers
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

    // Helpers
    bool transform_base_to_camera(const geometry_msgs::msg::PoseStamped & pose_base, geometry_msgs::msg::PoseStamped & pose_cam);
    bool project_point_to_image(const geometry_msgs::msg::PoseStamped & pose_cam,double & u, double & v) const;

    std::string estimate_color_at_pixel(const cv::Mat & image, double u, double v) const;

    // TF
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Action server
    rclcpp_action::Server<DetectColor>::SharedPtr action_server_;

    // Camera stuff
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

    cv::Mat last_image_;
    rclcpp::Time last_image_stamp_;
    mutable std::mutex image_mutex_;

    bool has_camera_info_;
    double fu_, fv_, u0_, v0_;

    // frames
    std::string base_frame_;
    std::string camera_frame_;
};