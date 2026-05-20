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
#include <cv_bridge/cv_bridge.hpp>

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

    const std::array<double, 9> K = {608.15084105623157,0.0,457.71223189128261,0.0,609.63354694069642,317.6449703796784,0.0,0.0,1.0};
    const std::vector<double> dist_coeffs = {
        -0.0818101598070618,      // k1
        -0.044298378963030155,    // k2
        0.00047792942060084727,   // p1
        -0.00029737171894225013,  // p2
        0.027074428106673747      // k3
    };


    // void info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr camInfo);
    // void odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);
    // void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr image_msg);
    void synced_callback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, 
                     const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg);


public:
    DetectorNode() : Node("detector") {
        autoaim_pub_ = this->create_publisher<uart_bridge::msg::AutoAim>(
            "/autoaim", 10);

        // camera_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        //     "/camera_info", 10,
        //     std::bind(&DetectorNode::info_callback, this, std::placeholders::_1));

        image_sub_.subscribe(this, "/image_raw", rmw_qos_profile_sensor_data);
        odom_sub_.subscribe(this, "uart/odometry", rmw_qos_profile_sensor_data);

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(1000), image_sub_, odom_sub_);
        sync_->registerCallback(std::bind(&DetectorNode::synced_callback, this, std::placeholders::_1, std::placeholders::_2));

        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/detection_image", 10);

        l_params = {0.04, 0.4, 35.0, 0.7};
        a_params = {0.7, 0.8, 3.2, 3.2, 5.5, 35.0};

        intrinsics.K = K;
        intrinsics.dist_coeffs = dist_coeffs;
        pnp_solver = std::make_unique<rm_auto_aim::PnPSolver>(
            intrinsics.K, intrinsics.dist_coeffs);
        detector = std::make_unique<rm_auto_aim::Detector>(
            PF::pfParams, pnp_solver.get(), 150, 0, l_params, a_params);
        detector->classifier = std::make_unique<rm_auto_aim::NumberClassifier>(
            "src/detector/detector_submodule/model/mlp.onnx",
            "src/detector/detector_submodule/model/label.txt",
            0.7,
            std::vector<std::string>{"negative"});


        last_time = std::chrono::steady_clock::now();
    }
};

// void DetectorNode::info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr camInfo) {
//     if (!recievedCameraInfo) {
//         recievedCameraInfo = true;
//         // intrinsics.K = camInfo->k;
//         // intrinsics.dist_coeffs = camInfo->d;
//         //NOTE: hardcoded intrinsics for arducam
//         intrinsics.K = K;
//         intrinsics.dist_coeffs = dist_coeffs;
//         pnp_solver = std::make_unique<rm_auto_aim::PnPSolver>(
//             intrinsics.K, intrinsics.dist_coeffs);
//         detector = std::make_unique<rm_auto_aim::Detector>(
//             PF::pfParams, pnp_solver.get(), 150, 0, l_params, a_params);
//         detector->classifier = std::make_unique<rm_auto_aim::NumberClassifier>(
//             "src/detector/detector_submodule/model/mlp.onnx",
//             "src/detector/detector_submodule/model/label.txt",
//             0.7,
//             std::vector<std::string>{"negative"});

//         RCLCPP_INFO(this->get_logger(), "Camera info received. Detector initialized.");
//     }
// }

void DetectorNode::synced_callback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, 
                     const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg) 
{

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
    gimbal_state.yaw = -pitch;
    gimbal_state.pitch = roll - 3.14159f/2;

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

    // if (y_crop >= height) {
    //     // Prevent passing an empty Mat to detector, which causes OpenCV assertions
    //     RCLCPP_INFO(this->get_logger(), "Cropped everthing");
    //     return; 
    // } 
    // else {
    //     cv::Rect roi(0, y_crop, width, height - y_crop);
    //     img = img(roi);
    // }

    // if (pnp_solver) {
    //     pnp_solver->setCropOffset(y_crop);
    // }

    // 6. Run Detection
    // std::cout<< dt << '\n';

    detector->run_detection(img, gimbal_state, dt);

    // 7. Extract Data and Publish
    auto auto_aim_data = detector->getAutoAimData();

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
    mymsg.target_id = auto_aim_data.id;

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

// void DetectorNode::odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg) {
//     std::lock_guard<std::mutex> lock(odom_mutex_);
//     odom_buffer_.push_back(odom_msg);

//     // Keep buffer size manageable (e.g., last 100 messages)
//     if (odom_buffer_.size() > 100) {
//         odom_buffer_.pop_front();
//     }
// }

// void DetectorNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr image_msg) {
//     if (!recievedCameraInfo || !detector) {
//         return;
//     }

//     nav_msgs::msg::Odometry::SharedPtr closest_odom;

//     // 1. Find the closest odometry message in the buffer
//     {
//         std::lock_guard<std::mutex> lock(odom_mutex_);
//         if (odom_buffer_.empty()) {
//             RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Odometry buffer empty, dropping frame.");
//             return;
//         }

//         auto img_time = rclcpp::Time(image_msg->header.stamp).nanoseconds();
//         auto closest_it = odom_buffer_.begin();
//         long long min_diff = std::abs(img_time - rclcpp::Time((*closest_it)->header.stamp).nanoseconds());

//         for (auto it = odom_buffer_.begin(); it != odom_buffer_.end(); ++it) {
//             long long diff = std::abs(img_time - rclcpp::Time((*it)->header.stamp).nanoseconds());
//             if (diff < min_diff) {
//                 min_diff = diff;
//                 closest_it = it;
//             }
//         }

//         closest_odom = *closest_it;
        
//         // Optional cleanup: erase messages older than the closest one to prevent buffer bloating
//         // We keep the closest one in case the next image is also very close to it.
//         odom_buffer_.erase(odom_buffer_.begin(), closest_it);
//     }

//     // 2. Calculate orientation from the matched odometry message
//     double w = closest_odom->pose.pose.orientation.w;
//     double x = closest_odom->pose.pose.orientation.x;
//     double y = closest_odom->pose.pose.orientation.y;
//     double z = closest_odom->pose.pose.orientation.z;
    
//     gimbal_state.yaw = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
//     gimbal_state.pitch = std::asin(2.0 * (w * y - z * x));

//     // 3. Process Timing
//     auto now = std::chrono::steady_clock::now();
//     double dt = std::chrono::duration<double>(now - last_time).count();
//     last_time = now;

//     if (dt <= 0.0001) {
//         dt = 0.0166; // Default to ~60fps expected time
//     }

//     // 4. Convert ROS Image to OpenCV Mat
//     cv_bridge::CvImagePtr cv_ptr;
//     try {
//         cv_ptr = cv_bridge::toCvCopy(*image_msg, sensor_msgs::image_encodings::BGR8);
//     } catch (cv_bridge::Exception &e) {
//         RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
//         return;
//     }
//     cv::Mat img = cv_ptr->image;

//     // 5. Horizon Cropping Logic
//     int y_crop = 0;
//     double fy = intrinsics.K[4];
//     double cy = intrinsics.K[5];

//     int height = img.rows;
//     int width = img.cols;

//     double y_horizon_calc = cy - (fy * std::tan(gimbal_state.pitch));
//     int y_horizon = std::max(0, std::min(height, static_cast<int>(y_horizon_calc)));
//     int safety_buffer = 15;
//     y_crop = std::max(0, y_horizon - safety_buffer);

//     // if (y_crop >= height) {
//     //     // Prevent passing an empty Mat to detector, which causes OpenCV assertions
//     //     RCLCPP_INFO(this->get_logger(), "Cropped everthing");
//     //     return; 
//     // } 
//     // else {
//     //     cv::Rect roi(0, y_crop, width, height - y_crop);
//     //     img = img(roi);
//     // }

//     // if (pnp_solver) {
//     //     pnp_solver->setCropOffset(y_crop);
//     // }

//     // 6. Run Detection
//     // std::cout<< dt << '\n';
//     detector->run_detection(img, gimbal_state, dt);

//     // 7. Extract Data and Publish
//     auto auto_aim_data = detector->getAutoAimData();

//     // Get detector feedback image and publish it
//     cv::Mat detector_feedback = detector->getDetectorFeedback();

//     cv_bridge::CvImage img_bridge;
//     sensor_msgs::msg::Image ros_image; 
//     std_msgs::msg::Header header; 
//     header.frame_id = "camera_frame"; 
//     img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, detector_feedback);
//     img_bridge.toImageMsg(ros_image);

//     image_pub_->publish(ros_image);

//     RCLCPP_DEBUG(this->get_logger(), "autoaim_topic_sent");

//     uart_bridge::msg::AutoAim mymsg;
//     mymsg.distance = auto_aim_data.distance;
//     mymsg.yaw_error = auto_aim_data.yaw_error;
//     mymsg.pitch_error = auto_aim_data.pitch_error;
//     mymsg.target_id = auto_aim_data.id;

//     autoaim_pub_->publish(mymsg); 

//     // 8. Calculate and print FPS
//     static double accumulated_dt = 0.0;
//     static int frame_count = 0;

//     accumulated_dt += dt;
//     frame_count++;

//     if (frame_count >= 60) {
//         double average_fps = frame_count / accumulated_dt;
//         RCLCPP_INFO(this->get_logger(), "Average FPS: %.2f", average_fps);
//         accumulated_dt = 0.0;
//         frame_count = 0;
//     }
// }

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectorNode>());
    rclcpp::shutdown();
    return 0;
}
