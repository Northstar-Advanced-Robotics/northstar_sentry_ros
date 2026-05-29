#include <chrono>
#include <functional>
#include <memory>
#include <algorithm>
#include <deque>
#include <mutex>

#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/int32.hpp"

#include <opencv4/opencv2/core/mat.hpp>
#include <cv_bridge/cv_bridge.h>

#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/utilities.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/synchronizer.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "camera/camera.hpp"
#include "detector.hpp"
#include "pf_params.hpp"
#include "pnp_solver.hpp"
#include "types.hpp"
#include "uart_bridge/msg/auto_aim.hpp"
#include "uart/messages/autoaim_message.hpp"

std::atomic<long long> jetson_to_mcb_offset_us{0};

class DetectorNode : public rclcpp::Node {
private:
    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
    message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
    
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_;
    rclcpp::Publisher<uart_bridge::msg::AutoAim>::SharedPtr autoaim_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, nav_msgs::msg::Odometry> SyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    double dt;
    std::chrono::time_point<std::chrono::steady_clock> last_time;
    src::Kinematics::Robot::GimbalState gimbal_state;

    src::camera::Camera::CameraIntrinsics intrinsics;
    std::unique_ptr<rm_auto_aim::PnPSolver> pnp_solver;
    rm_auto_aim::Detector::LightParams l_params;
    rm_auto_aim::Detector::ArmorParams a_params;
    std::unique_ptr<rm_auto_aim::Detector> detector;

    bool recievedCameraInfo = false;

    void info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr camInfo);

    void synced_callback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, 
                     const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg);


public:
    DetectorNode() : Node("detector") {
        autoaim_pub_ = this->create_publisher<uart_bridge::msg::AutoAim>(
            "/autoaim", 10);

        camera_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/front/camera/color/camera_info", 10,
            std::bind(&DetectorNode::info_callback, this, std::placeholders::_1));

        image_sub_.subscribe(this, "/front/camera/color/image_raw", rmw_qos_profile_sensor_data);
        odom_sub_.subscribe(this, "/uart/odometry", rmw_qos_profile_sensor_data);

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(1000), image_sub_, odom_sub_);
        sync_->registerCallback(std::bind(&DetectorNode::synced_callback, this, std::placeholders::_1, std::placeholders::_2));

        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/detection_image", 10);

        l_params = {0.04, 0.4, 35.0, 0.7};
        a_params = {0.7, 0.8, 3.2, 3.2, 5.5, 35.0};

        last_time = std::chrono::steady_clock::now();
    }
};

void DetectorNode::info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr camInfo) {
    // Only initialize everything once
    if (!recievedCameraInfo) {
        
        // 1. Extract dynamic intrinsics from the ROS 2 message
        intrinsics.K = camInfo->k;
        intrinsics.dist_coeffs = camInfo->d;

        // 2. Initialize the solvers with the dynamic data
        pnp_solver = std::make_unique<rm_auto_aim::PnPSolver>(
            intrinsics.K, intrinsics.dist_coeffs);
            
        detector = std::make_unique<rm_auto_aim::Detector>(
            PF::pfParams, pnp_solver.get(), 150, 0, l_params, a_params);
            
        detector->classifier = std::make_unique<rm_auto_aim::NumberClassifier>(
            "src/detector/detector_submodule/model/mlp.onnx",
            "src/detector/detector_submodule/model/label.txt",
            0.7,
            std::vector<std::string>{"negative"});

        recievedCameraInfo = true;
        RCLCPP_INFO(this->get_logger(), "Camera info received. Detector & PnP Solver dynamically initialized.");
    }
}

void DetectorNode::synced_callback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, 
                     const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg) 
{
    // SAFETY CHECK: Drop frames until camera_info initializes the detector
    if (!recievedCameraInfo || !detector || !pnp_solver) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
            "Waiting for camera_info... dropping synchronized frames.");
        return;
    }

    tf2::Quaternion q(
        odom_msg->pose.pose.orientation.x,
        odom_msg->pose.pose.orientation.y,
        odom_msg->pose.pose.orientation.z,
        odom_msg->pose.pose.orientation.w);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);    
    // RCLCPP_INFO(this->get_logger(), "Gimbal State (deg): R:%.2f P:%.2f Y:%.2f", 
    //         roll * 180.0 / M_PI, 
    //         pitch * 180.0 / M_PI, 
    //         yaw * 180.0 / M_PI);    
    gimbal_state.yaw = yaw;
    gimbal_state.pitch = pitch;

    // 3. Process Timing
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - last_time).count();
    last_time = now;

    if (dt <= 0.0001) {
        dt = 0.0166; // Default to ~60fps expected time
    }

    // 4. Convert ROS Image to OpenCV Mat
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(*image_msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat img = cv_ptr->image;

    // 5. Horizon Cropping Logic
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
        RCLCPP_INFO(this->get_logger(), "Cropped everthing");
        return; 
    } 
    else {
        cv::Rect roi(0, y_crop, width, height - y_crop);
        img = img(roi);
    }

    if (pnp_solver) {
        pnp_solver->setCropOffset(y_crop);
    }
    // RCLCPP_INFO(this->get_logger(), "Croping: y_horizon:%d y_crop:%d", 
    //     y_horizon, 
    //     y_crop);    


    // 6. Run Detection
    // std::cout<< dt << '\n';

    detector->run_detection(img, gimbal_state, dt);

    // 7. Extract Data and Publish
    src::uart::AutoAimMessage auto_aim_data = detector->getAutoAimData();

    // Get detector feedback image and publish it
    cv::Mat detector_feedback = detector->getDetectorFeedback();

    cv_bridge::CvImage img_bridge;
    sensor_msgs::msg::Image ros_image; 
    std_msgs::msg::Header header; 
    header.frame_id = "camera_frame"; 
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, detector_feedback);
    img_bridge.toImageMsg(ros_image);

    image_pub_->publish(ros_image);

    RCLCPP_DEBUG(this->get_logger(), "autoaim_topic_sent");

    uart_bridge::msg::AutoAim mymsg;
    mymsg.distance = auto_aim_data.distance;
    mymsg.yaw_error = auto_aim_data.yaw_error;
    mymsg.pitch_error = auto_aim_data.pitch_error;
    mymsg.target_id = auto_aim_data.target_id;

    autoaim_pub_->publish(mymsg); 

    // 8. Calculate and print FPS
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
