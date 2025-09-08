#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <atomic>
#include <mutex>
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>

class BoxDetector : public rclcpp::Node
{
public:
    BoxDetector()
        : Node("box_detector_node"),
          fx_(declare_parameter("fx", 554.3827)),
          fy_(declare_parameter("fy", 554.3827)),
          cx_(declare_parameter("cx", 320.5)),
          cy_(declare_parameter("cy", 240.5)),
          min_area_px_(declare_parameter("min_area_px", 500.0)),
          camera_frame_(declare_parameter("camera_frame", "realsense_rgb_frame")),
          base_frame_(declare_parameter("base_frame", "base_link")),
          detection_active_(false)
    {
        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/detected_box_pose", 10);

        rgb_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "/camera/image_raw");
        depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "/camera/depth/image_raw");

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), *rgb_sub_, *depth_sub_);
        sync_->registerCallback(std::bind(&BoxDetector::imageCb, this, std::placeholders::_1, std::placeholders::_2));

        start_srv_ = create_service<std_srvs::srv::Trigger>(
            "/start_detection",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>&,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
                detection_active_ = true;
                resp->success = true;
                resp->message = "Detection started.";
                RCLCPP_INFO(get_logger(), " Detection STARTED.");
            });

        stop_srv_ = create_service<std_srvs::srv::Trigger>(
            "/stop_detection",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>&,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
                detection_active_ = false;
                resp->success = true;
                resp->message = "Detection stopped.";
                RCLCPP_INFO(get_logger(), " Detection STOPPED.");
            });

        RCLCPP_INFO(get_logger(), " BoxDetector ready. Call /start_detection to begin.");
    }

private:
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image>;

    void imageCb(const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg,
                 const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg)
    {
        if (!detection_active_.load()) return;

        RCLCPP_DEBUG(get_logger(), "Received image pair.");

        cv::Mat rgb, depth32;
        try {
            rgb = cv_bridge::toCvCopy(rgb_msg, "bgr8")->image;
            depth32 = cv_bridge::toCvCopy(depth_msg, "32FC1")->image;
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge error: %s", e.what());
            return;
        }

        if (rgb.empty() || depth32.empty()) {
            RCLCPP_WARN(get_logger(), "Empty image or depth.");
            return;
        }

        // Convert to HSV and create color mask
        cv::Mat hsv, mask;
        cv::cvtColor(rgb, hsv, cv::COLOR_BGR2HSV);

        cv::Mat red1, red2, blue;
        // Wider HSV ranges for robustness
        cv::inRange(hsv, cv::Scalar(0, 100, 50), cv::Scalar(10, 255, 255), red1);
        cv::inRange(hsv, cv::Scalar(160, 100, 50), cv::Scalar(180, 255, 255), red2);
        cv::inRange(hsv, cv::Scalar(100, 100, 50), cv::Scalar(140, 255, 255), blue);

        cv::bitwise_or(red1, red2, mask);
        cv::bitwise_or(mask, blue, mask);

        // Morphology to clean mask
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

        int mask_pixels = cv::countNonZero(mask);
        RCLCPP_DEBUG(get_logger(), " Mask non-zero pixels: %d", mask_pixels);
        if (mask_pixels < 100) {
            RCLCPP_DEBUG(get_logger(), "Not enough colored pixels.");
            return;
        }

        // Find contours
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        RCLCPP_DEBUG(get_logger(), " Found %zu contours.", contours.size());
        if (contours.empty()) return;

        // Get largest contour
        auto largest = *std::max_element(contours.begin(), contours.end(),
            [](const auto& a, const auto& b) { return cv::contourArea(a) < cv::contourArea(b); });

        double area = cv::contourArea(largest);
        RCLCPP_DEBUG(get_logger(), "Largest contour area: %.0f (min=%.0f)", area, min_area_px_);
        if (area < min_area_px_) return;

        // Compute centroid
        cv::Moments m = cv::moments(largest);
        if (m.m00 == 0) {
            RCLCPP_WARN(get_logger(), "Contour has zero area.");
            return;
        }

        int cx = static_cast<int>(m.m10 / m.m00);
        int cy = static_cast<int>(m.m01 / m.m00);

        RCLCPP_DEBUG(get_logger(), " Centroid at (%d, %d)", cx, cy);

        // Bounds check for depth
        if (cy >= depth32.rows || cx >= depth32.cols || cy < 0 || cx < 0) {
            RCLCPP_WARN(get_logger(), " Centroid out of depth image bounds.");
            return;
        }

        // Get depth
        float depth = depth32.at<float>(cy, cx);
        if (!std::isfinite(depth) || depth < 0.1f || depth > 8.0f) {
            RCLCPP_WARN(get_logger(), " Invalid depth: %.3f", depth);
            return;
        }

        RCLCPP_DEBUG(get_logger(), "Depth at centroid: %.3f m", depth);

        // Back-project to 3D in camera frame
        float X_cam = (cx - cx_) * depth / fx_;
        float Y_cam = (cy - cy_) * depth / fy_;
        float Z_cam = depth;

        RCLCPP_DEBUG(get_logger(), " 3D point in camera frame: [%.3f, %.3f, %.3f]", X_cam, Y_cam, Z_cam);
        // Lookup transform: camera_frame → base_frame
        geometry_msgs::msg::TransformStamped T_cb;
        try {
            T_cb = tf_buffer_->lookupTransform(base_frame_, camera_frame_, tf2::TimePointZero);
            RCLCPP_DEBUG(get_logger(), " TF lookup successful.");
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(get_logger(), " TF lookup failed: %s", ex.what());
            return;
        }

        // Transform point to base_link frame
        tf2::Transform tf_cb;
        tf2::fromMsg(T_cb.transform, tf_cb);

        tf2::Vector3 point_in_cam(X_cam, Y_cam, Z_cam);
        tf2::Vector3 point_in_base = tf_cb * point_in_cam;

        // Compute 2D orientation from contour
        cv::RotatedRect rotRect = cv::minAreaRect(largest);
        double angle_deg = rotRect.angle;

        // Align angle with long side of box
        if (rotRect.size.width < rotRect.size.height) {
            angle_deg += 90.0;
        }

        double angle_rad = angle_deg * M_PI / 180.0;

        // Create base rotation (around Z-axis = yaw)
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, angle_rad);
        q.normalize();

        //  FLIP Z-AXIS TO POINT UP (blue arrow toward sky)
        tf2::Quaternion q_flip;
        q_flip.setRPY(-1.57, 1.57, M_PI);  // 180° roll around X-axis
        q = q * q_flip;
        q.normalize();

        RCLCPP_INFO(get_logger(), " Box detected: [%.3f, %.3f, %.3f] | Yaw: %.1f° | Z-axis FLIPPED UP",
                    point_in_base.x(), point_in_base.y(), point_in_base.z(), angle_deg);

        // Publish PoseStamped
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = this->now();
        pose.header.frame_id = base_frame_;
        pose.pose.position.x = point_in_base.x();
        pose.pose.position.y = point_in_base.y();
        pose.pose.position.z = point_in_base.z();
        pose.pose.orientation = tf2::toMsg(q);  //  Includes Z-flip

        pose_pub_->publish(pose);

        // Broadcast TF
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = this->now();
        tf_msg.header.frame_id = base_frame_;
        tf_msg.child_frame_id = "detected_box";
        tf_msg.transform.translation.x = point_in_base.x();
        tf_msg.transform.translation.y = point_in_base.y();
        tf_msg.transform.translation.z = point_in_base.z();
        tf_msg.transform.rotation = tf2::toMsg(q);  //  Includes Z-flip

        tf_broadcaster_->sendTransform(tf_msg);
    }

    double fx_, fy_, cx_, cy_;
    double min_area_px_;
    std::string camera_frame_;
    std::string base_frame_;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> rgb_sub_, depth_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_, stop_srv_;

    std::atomic<bool> detection_active_;
    std::mutex mutex_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BoxDetector>());
    rclcpp::shutdown();
    return 0;
}