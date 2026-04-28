#include <chrono>
#include <functional>
#include <memory>
#include <algorithm>

#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/int32.hpp"

#include <opencv4/opencv2/core/mat.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/utilities.hpp>

// Message Filter Headers for Synchronization
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include "camera/camera.hpp"
#include "detector.hpp"
#include "pf_params.hpp"
#include "pnp_solver.hpp"
#include "types.hpp"
#include "uart_bridge/msg/auto_aim.hpp"

std::atomic<long long> jetson_to_mcb_offset_us{0};

class DetectorNode : public rclcpp::Node {
private:
    // Sync Policy Definition (Image and Odometry)
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, nav_msgs::msg::Odometry> SyncPolicy;
    
    // Message Filter Subscribers
    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
    message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    rclcpp::Publisher<uart_bridge::msg::AutoAim>::SharedPtr autoaim_pub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_;

    double dt;
    std::chrono::time_point<std::chrono::steady_clock> last_time;
    src::Kinematics::Robot::GimbalState gimbal_state;

    src::camera::Camera::CameraIntrinsics intrinsics;
    std::unique_ptr<rm_auto_aim::PnPSolver> pnp_solver;
    rm_auto_aim::Detector::LightParams l_params;
    rm_auto_aim::Detector::ArmorParams a_params;
    std::unique_ptr<rm_auto_aim::Detector> detector;

    bool recievedCameraInfo = false;

    void info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr camInfo);
    void sync_callback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, 
                       const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg);

public:
    DetectorNode() : Node("detector") {
        autoaim_pub_ = this->create_publisher<uart_bridge::msg::AutoAim>(
            "autoaim", rclcpp::SensorDataQoS());

        camera_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/camera_info", 10,
            std::bind(&DetectorNode::info_callback, this, std::placeholders::_1));

        // 1. Initialize message filter subscribers
        // Use rmw_qos_profile_sensor_data if your camera is publishing with SensorDataQoS
        image_sub_.subscribe(this, "/front/camera/color/image_raw");
        odom_sub_.subscribe(this, "odometry");

        // 2. Initialize the Synchronizer (Queue size of 10)
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), image_sub_, odom_sub_);

        // 3. Register the synchronized callback
        sync_->registerCallback(std::bind(&DetectorNode::sync_callback, this, std::placeholders::_1, std::placeholders::_2));

        l_params = {0.04, 0.4, 35.0, 0.7};
        a_params = {0.7, 0.8, 3.2, 3.2, 5.5, 35.0};

        last_time = std::chrono::steady_clock::now();
    }
};

void DetectorNode::info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr camInfo) {
    if (!recievedCameraInfo) {
        recievedCameraInfo = true;
        intrinsics.K = camInfo->k;
        intrinsics.dist_coeffs = camInfo->d;
        pnp_solver = std::make_unique<rm_auto_aim::PnPSolver>(
            intrinsics.K, intrinsics.dist_coeffs);
        detector = std::make_unique<rm_auto_aim::Detector>(
            PF::pfParams, pnp_solver.get(), 150, 0, l_params, a_params);

        RCLCPP_INFO(this->get_logger(), "Camera info received. Detector initialized.");
    }
}

void DetectorNode::sync_callback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, 
                                 const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg) 
{
    if (!recievedCameraInfo || !detector) {
        return;
    }

    // 1. Calculate orientation from the synchronized odometry message
    double w = odom_msg->pose.pose.orientation.w;
    double x = odom_msg->pose.pose.orientation.x;
    double y = odom_msg->pose.pose.orientation.y;
    double z = odom_msg->pose.pose.orientation.z;
    
    gimbal_state.yaw = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    gimbal_state.pitch = std::asin(2.0 * (w * y - z * x));

    // 2. Process Timing
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - last_time).count();
    last_time = now;

    if (dt <= 0.0001) {
        dt = 0.0166; // Default to ~60fps expected time
    }

    // 3. Convert ROS Image to OpenCV Mat
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(*image_msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat img = cv_ptr->image;

    // 4. Horizon Cropping Logic
    int y_crop = 0;
    double fy = intrinsics.K[4];
    double cy = intrinsics.K[5];

    int height = img.rows;
    int width = img.cols;

    double y_horizon_calc = cy - (fy * std::tan(gimbal_state.pitch));
    int y_horizon = std::max(0, std::min(height, static_cast<int>(y_horizon_calc)));
    int safety_buffer = 15;
    y_crop = std::max(0, y_horizon - safety_buffer);

    if (y_crop >= height) {
        // Prevent passing an empty Mat to detector, which causes OpenCV assertions
        return; 
    } 
    else {
        cv::Rect roi(0, y_crop, width, height - y_crop);
        img = img(roi);
    }

    if (pnp_solver) {
        pnp_solver->setCropOffset(y_crop);
    }

    // 5. Run Detection
    detector->run_detection(img, gimbal_state, dt);

    // 6. Extract Data and Publish
    auto auto_aim_data = detector->getAutoAimData();

    RCLCPP_DEBUG(this->get_logger(), "autoaim_topic_sent");

    uart_bridge::msg::AutoAim mymsg;
    mymsg.distance = auto_aim_data.distance;
    mymsg.yaw_error = auto_aim_data.yaw_error;
    mymsg.pitch_error = auto_aim_data.pitch_error;
    mymsg.target_id = auto_aim_data.id;

    autoaim_pub_->publish(mymsg); 

    // 7. Calculate and print FPS
    static double accumulated_dt = 0.0;
    static int frame_count = 0;

    accumulated_dt += dt;
    frame_count++;

    if (frame_count >= 60) {
        double average_fps = frame_count / accumulated_dt;
        RCLCPP_INFO(this->get_logger(), "Average FPS: %.2f", average_fps);
        accumulated_dt = 0.0;
        frame_count = 0;
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectorNode>());
    rclcpp::shutdown();
    return 0;
}
