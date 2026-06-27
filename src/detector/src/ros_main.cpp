#include <algorithm>
#include <chrono>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/synchronizer.hpp>
#include <opencv4/opencv2/core/mat.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/utilities.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "camera/camera.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/int32.hpp"
#include "uart/messages/autoaim_message.hpp"
#include "uart_bridge/msg/auto_aim.hpp"

#include "constants.hpp"
#include "detector.hpp"
#include "kinematics.hpp"
#include "pf_params.hpp"
#include "pnp_solver.hpp"
#include "types.hpp"

std::atomic<long long> jetson_to_mcb_offset_us{0};

class DetectorNode : public rclcpp::Node
{
private:
    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
    message_filters::Subscriber<geometry_msgs::msg::Vector3Stamped> gimbal_data_sub_;

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr robotid_sub_;
    rclcpp::Publisher<uart_bridge::msg::AutoAim>::SharedPtr autoaim_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

    typedef message_filters::sync_policies::
        ApproximateTime<sensor_msgs::msg::Image, geometry_msgs::msg::Vector3Stamped>
            SyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    rclcpp::Time last_image_stamp_{};
    src::Kinematics::Robot::GimbalState gimbal_state;

    src::camera::Camera::CameraIntrinsics intrinsics;
    std::unique_ptr<rm_auto_aim::PnPSolver> pnp_solver;
    rm_auto_aim::Detector::LightParams l_params;
    rm_auto_aim::Detector::ArmorParams a_params;
    std::unique_ptr<rm_auto_aim::Detector> detector;

    bool recievedCameraInfo = false;
    bool color_initialized_ = false;
    bool color_ = false;  // false=blue, true=red (matches main.cpp: robot_id / 100)

    // Parameter client for the camera driver node — set "camera_node_name" param at launch to match
    std::shared_ptr<rclcpp::AsyncParametersClient> camera_param_client_;

    void robotid_callback(const std_msgs::msg::Int32::ConstSharedPtr msg);
    void apply_camera_color_config();
    void try_initialize_detector();
    void info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr camInfo);
    void synced_callback(
        const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
        const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr& gimbal_data_msg);

public:
    DetectorNode() : Node("detector")
    {
        autoaim_pub_ = this->create_publisher<uart_bridge::msg::AutoAim>("/autoaim", 10);

        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/front/camera/color/camera_info",
            10,
            std::bind(&DetectorNode::info_callback, this, std::placeholders::_1));

        robotid_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/uart/robotid",
            10,
            std::bind(&DetectorNode::robotid_callback, this, std::placeholders::_1));

        image_sub_.subscribe(this, "/front/camera/color/image_raw", rmw_qos_profile_sensor_data);
        gimbal_data_sub_.subscribe(this, "/uart/gimbal_data", rmw_qos_profile_sensor_data);

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(1000),
            image_sub_,
            gimbal_data_sub_);
        sync_->registerCallback(std::bind(
            &DetectorNode::synced_callback,
            this,
            std::placeholders::_1,
            std::placeholders::_2));

        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/detection_image", 10);

        // "camera_node_name" ROS parameter lets you override the target at launch time.
        std::string camera_node = this->declare_parameter("camera_node_name", "/arducam_node");
        camera_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, camera_node);

        l_params = {0.04, 0.55, 35.0, 0.7};
        a_params = {0.7, 0.8, 3.2, 3.2, 5.5, 35.0};
    }
};

void DetectorNode::robotid_callback(const std_msgs::msg::Int32::ConstSharedPtr msg)
{
    if (color_initialized_) return;

    int robot_id = msg->data;
    if (robot_id == 0) return;  // INVALID — same check as main.cpp

    color_ = static_cast<bool>(robot_id / 100);  // 1xx=red, 0xx=blue
    uint8_t robot_type = static_cast<uint8_t>(robot_id - static_cast<int>(color_) * 100);

    src::Kinematics::Robot::init(src::Constants::Robot::selectConstants(robot_type));

    color_initialized_ = true;

    RCLCPP_INFO(
        this->get_logger(),
        "Robot ID: %d | Team: %s | Detecting: %s armor",
        robot_id,
        color_ ? "RED" : "BLUE",
        color_ ? "BLUE" : "RED");

    apply_camera_color_config();
    try_initialize_detector();
}

void DetectorNode::apply_camera_color_config()
{
    if (!camera_param_client_->service_is_ready())
    {
        RCLCPP_WARN(
            this->get_logger(),
            "Camera parameter service not ready — camera color config skipped. "
            "Verify camera_node_name param matches the camera driver's node name.");
        return;
    }

    std::vector<rclcpp::Parameter> params;
    if (color_)
    {  // red team
        params = {
            rclcpp::Parameter("auto_exposure", 1),  // 1=manual, 3=auto
            rclcpp::Parameter("white_balance_automatic", false),
            rclcpp::Parameter("brightness", 2),
            rclcpp::Parameter("contrast", 8),
            rclcpp::Parameter("saturation", 3),
            rclcpp::Parameter("white_balance_temperature", 4700),
            rclcpp::Parameter("gain", 168),
            rclcpp::Parameter("exposure_time_absolute", 39),
        };
    }
    else
    {  // blue team
        params = {
            rclcpp::Parameter("auto_exposure", 1),  // 1=manual, 3=auto
            rclcpp::Parameter("white_balance_automatic", false),
            rclcpp::Parameter("brightness", -3),
            rclcpp::Parameter("contrast", 7),
            rclcpp::Parameter("saturation", 3),
            rclcpp::Parameter("white_balance_temperature", 5948),
            rclcpp::Parameter("gain", 168),
            rclcpp::Parameter("exposure_time_absolute", 39),
        };
    }
    camera_param_client_->set_parameters(params);
    RCLCPP_INFO(
        this->get_logger(),
        "Camera color config applied for %s team.",
        color_ ? "RED" : "BLUE");
}

void DetectorNode::try_initialize_detector()
{
    // Wait for both camera info and robot ID before creating the detector so
    // !color_ is known and correct (same ordering guarantee as main.cpp).
    if (!recievedCameraInfo || !color_initialized_ || detector) return;

    pnp_solver = std::make_unique<rm_auto_aim::PnPSolver>(intrinsics.K, intrinsics.dist_coeffs);

    detector = std::make_unique<rm_auto_aim::Detector>(
        PF::pfParams,
        pnp_solver.get(),
        150,
        !color_,
        l_params,
        a_params,
        true);

    detector->classifier = std::make_unique<rm_auto_aim::NumberClassifier>(
        "src/detector/detector_submodule/model/mlp.onnx",
        "src/detector/detector_submodule/model/label.txt",
        0.7,
        std::vector<std::string>{"negative"});

    RCLCPP_INFO(
        this->get_logger(),
        "Detector initialized. Detecting %s armor.",
        color_ ? "BLUE" : "RED");
}

void DetectorNode::info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr camInfo)
{
    if (recievedCameraInfo) return;

    intrinsics.K = camInfo->k;
    intrinsics.dist_coeffs = camInfo->d;
    recievedCameraInfo = true;

    RCLCPP_INFO(this->get_logger(), "Camera info received.");
    try_initialize_detector();
}

void DetectorNode::synced_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
    const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr& gimbal_data_msg)
{
    if (!recievedCameraInfo || !color_initialized_ || !detector || !pnp_solver)
    {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Waiting for camera_info and robot ID... dropping synchronized frames.");
        return;
    }

    gimbal_state.yaw = gimbal_data_msg->vector.z;
    gimbal_state.pitch = gimbal_data_msg->vector.y;

    rclcpp::Time current_stamp = image_msg->header.stamp;
    double dt = (last_image_stamp_.nanoseconds() > 0)
                    ? (current_stamp - last_image_stamp_).seconds()
                    : 0.0166;
    last_image_stamp_ = current_stamp;

    if (dt <= 0.0001) dt = 0.0166;

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(*image_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat img = cv_ptr->image;

    int y_crop = 0;
    double fy = intrinsics.K[4];
    double cy = intrinsics.K[5];

    int height = img.rows;
    int width = img.cols;

    double y_horizon_calc = cy - (fy * std::tan(gimbal_state.pitch));
    int y_horizon = std::max(0, std::min(height, static_cast<int>(y_horizon_calc)));
    int safety_buffer = 15;
    y_crop = std::max(0, y_horizon - safety_buffer);

    if (y_crop >= height)
    {
        RCLCPP_INFO(this->get_logger(), "Cropped everything");
        return;
    }
    else
    {
        cv::Rect roi(0, y_crop, width, height - y_crop);
        img = img(roi);
    }

    pnp_solver->setCropOffset(y_crop);

    detector
        ->run_detection(img, gimbal_state, dt, rclcpp::Time(image_msg->header.stamp).nanoseconds());

    src::uart::AutoAimMessage auto_aim_data = detector->getAutoAimData();

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

    static double accumulated_dt = 0.0;
    static int frame_count = 0;

    accumulated_dt += dt;
    frame_count++;

    if (frame_count >= 60)
    {
        double average_fps = frame_count / accumulated_dt;
        RCLCPP_INFO(this->get_logger(), "Average FPS: %.2f", average_fps);
        accumulated_dt = 0.0;
        frame_count = 0;
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectorNode>());
    rclcpp::shutdown();
    return 0;
}
