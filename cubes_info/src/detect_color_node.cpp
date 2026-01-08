#include "detect_color_node.hpp"

DetectColorNode::DetectColorNode()
: Node("detect_color_node"), has_camera_info_(false), fu_(0.0), fv_(0.0), u0_(0.0), v0_(0.0), base_frame_("base_link"), camera_frame_("external_camera/link/rgb_camera")
{
    RCLCPP_INFO(get_logger(), "detect_color_node started");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Subscriptions
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/rgb_camera/image",                 
        rclcpp::SensorDataQoS(),
        std::bind(&DetectColorNode::image_callback, this, std::placeholders::_1));

    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "/rgb_camera/camera_info",               
        10,
        std::bind(&DetectColorNode::camera_info_callback, this, std::placeholders::_1));

    // action server
    action_server_ = rclcpp_action::create_server<DetectColor>(
        this,
        "detect_color",  
        std::bind(&DetectColorNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&DetectColorNode::handle_cancel, this, std::placeholders::_1),
        std::bind(&DetectColorNode::handle_accepted, this, std::placeholders::_1));
}


rclcpp_action::GoalResponse DetectColorNode::handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const DetectColor::Goal> goal)
{
    (void)goal;
    RCLCPP_INFO(get_logger(), "Received goal request: detect the color of the cube in pose (%.2f, %.2f, %.2f) in base_link frame",
                goal->cube_pose_base.pose.position.x,
                goal->cube_pose_base.pose.position.y,
                goal->cube_pose_base.pose.position.z);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DetectColorNode::handle_cancel(const std::shared_ptr<GoalHandleDetectColor> goal_handle)
{
    (void)goal_handle;
    RCLCPP_INFO(get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void DetectColorNode::handle_accepted(const std::shared_ptr<GoalHandleDetectColor> goal_handle)
{
    std::thread{std::bind(&DetectColorNode::execute, this, goal_handle)}.detach();
}

void DetectColorNode::execute(const std::shared_ptr<GoalHandleDetectColor> goal_handle)
{
    auto result = std::make_shared<DetectColor::Result>();
    auto feedback = std::make_shared<DetectColor::Feedback>();
    
    const auto goal = goal_handle->get_goal();
    geometry_msgs::msg::PoseStamped pose_base = goal->cube_pose_base;
    
    RCLCPP_INFO(this->get_logger(), "Starting color detection task");
    feedback->status = "Starting color detection";
    goal_handle->publish_feedback(feedback);

    
    if (!has_camera_info_) {
        RCLCPP_INFO(this->get_logger(), "camera_info is not available");
        feedback->status = "camera_info is not available";
        goal_handle->publish_feedback(feedback);
        result->color = "unknown";
        goal_handle->succeed(result);
        return;
    }

    cv::Mat image;
    {
        std::lock_guard<std::mutex> lock(image_mutex_);
        if (last_image_.empty()) {
            RCLCPP_INFO(this->get_logger(), "image is not available");
            feedback->status = "image is not available";
            goal_handle->publish_feedback(feedback);
            result->color = "unknown";
            goal_handle->succeed(result);
            return;
        }
        image = last_image_.clone();
    }

    // from world (in this case base_link) to camera frame
    geometry_msgs::msg::PoseStamped pose_cam;
    if (!transform_base_to_camera(pose_base, pose_cam)) {
        RCLCPP_INFO(this->get_logger(), "transformation base_link to camera failed");
        feedback->status = "transformation base_link to camera failed";
        goal_handle->publish_feedback(feedback);
        result->color = "unknown";
        goal_handle->succeed(result);
        return;
    }

    RCLCPP_INFO(this->get_logger(), "transformation base link to camera done correctly");
    feedback->status = "transformation base link to camera done";
    goal_handle->publish_feedback(feedback);

    // from camera frame to image pixel
    double u = 0.0, v = 0.0;
    if (!project_point_to_image(pose_cam, u, v)) {
        RCLCPP_INFO(this->get_logger(), "Projection to image failed");
        feedback->status = "Projection to image failed";
        goal_handle->publish_feedback(feedback);
        result->color = "unknown";
        goal_handle->succeed(result);
        return;
    }

    // now we have the image and the position in the image to do the color estimation
    std::string color = estimate_color_at_pixel(image, u, v);

    feedback->status = "Color estimated: " + color;
    goal_handle->publish_feedback(feedback);

    result->color = color;
    goal_handle->succeed(result);

    RCLCPP_INFO(get_logger(), "detect_color result: %s", color.c_str());
}



void DetectColorNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(image_mutex_);
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        last_image_ = cv_ptr->image.clone();
        last_image_stamp_ = msg->header.stamp;
    } catch (const cv_bridge::Exception & e) {
        RCLCPP_ERROR(get_logger(), "exception: %s", e.what());
    }
}

void DetectColorNode::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    fu_ = msg->k[0];
    fv_ = msg->k[4];
    u0_ = msg->k[2];
    v0_ = msg->k[5];
    has_camera_info_ = true;

    RCLCPP_INFO_ONCE(get_logger(),"Camera intrinsics parameters: fu=%.1f fv=%.1f u0=%.1f v0=%.1f", fu_, fv_, u0_, v0_);
}


bool DetectColorNode::transform_base_to_camera(const geometry_msgs::msg::PoseStamped & pose_base, geometry_msgs::msg::PoseStamped & pose_cam)
{
    geometry_msgs::msg::PoseStamped pose_in_base = pose_base;

    if (pose_in_base.header.frame_id.empty()) {
        pose_in_base.header.frame_id = base_frame_;
    }

    try {
        tf_buffer_->transform(pose_in_base, pose_cam, camera_frame_, tf2::durationFromSec(0.1));
    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(get_logger(), "TransformException base_link to camera: %s", ex.what());
        return false;
    }

    return true;
}

bool DetectColorNode::project_point_to_image(const geometry_msgs::msg::PoseStamped & pose_cam, double & u, double & v) const
{
    if (!has_camera_info_) {
        return false;
    }

    const double x_c = pose_cam.pose.position.x;
    const double y_c = pose_cam.pose.position.y;
    const double z_c = pose_cam.pose.position.z;

    if (z_c <= 0.0) {
        return false;
    }

    u = fu_ * (x_c / z_c) + u0_;
    v = fv_ * (y_c / z_c) + v0_;

    return true;
}

std::string DetectColorNode::estimate_color_at_pixel(const cv::Mat & image, double u, double v) const
{
    int cx = static_cast<int>(std::round(u));
    int cy = static_cast<int>(std::round(v));

    // dimension Region Of Interest (ROI)
    const int roi_size = 15;

    // we take the colored part of the cube (not the apriltag)
    const int vertical_offset = 35;   


    int cx_shifted = cx;
    int cy_shifted = cy + vertical_offset;

    int x = std::max(0, cx_shifted - roi_size / 2);
    int y = std::max(0, cy_shifted - roi_size / 2);

    // to be sure ROI is inside the image (but we know that is always true)
    x = std::min(x, image.cols - roi_size);
    y = std::min(y, image.rows - roi_size);

    if (x < 0 || y < 0 || x + roi_size > image.cols || y + roi_size > image.rows)
        return "unknown"; // ROI is out of image 

    cv::Rect roi_rect(x, y, roi_size, roi_size);
    cv::Mat roi = image(roi_rect);

    
    // we use HSV color space to estimate the color
    cv::Mat roi_hsv;
    cv::cvtColor(roi, roi_hsv, cv::COLOR_BGR2HSV);

    cv::Scalar mean_hsv = cv::mean(roi_hsv);
    double H = mean_hsv[0];
    double S = mean_hsv[1];
    double V = mean_hsv[2];

    
    if (H < 10 || H >= 170) return "red";
    if (H >= 10 && H < 35) return "yellow";
    if (H >= 35 && H < 85) return "green";
    if (H >= 85 && H < 140) return "blue";
    if (H >= 140 && H < 170) return "fuchsia";

    if (S < 40){
        if (V < 60) return "black";
        if (V > 200) return "white";
        return "gray";
    }


    return "unknown";
}