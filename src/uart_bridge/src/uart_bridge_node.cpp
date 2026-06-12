#include "uart/messages/vision_localization_message.hpp"
#include "uart_bridge/msg/auto_aim.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>


#include <cstdio>
#include <memory> // Added for std::unique_ptr
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/time.hpp>
#include <tf2/utils.h>
#include <thread>

#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/utilities.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "uart/ibus_reader.hpp"
#include <uart/handlers/odometry_handler.hpp>
#include <uart/messages/alive_message.hpp>
#include <uart/messages/auto_path_message.hpp>
#include <uart/messages/autoaim_message.hpp>
#include <uart/messages/odometry_message.hpp>
#include "uart/messages/fly_sky_meassage.hpp"
#include <uart/uart.hpp>

using namespace std::chrono_literals;

std::atomic<long long> jetson_to_mcb_offset_us{0};

class UartBridge : public rclcpp::Node {
private:
    // FIX: transfrom subscriber to rig form odom. We need its
    // positon

    double pos_x_ = 0.0;
    double pos_y_ = 0.0;
    rclcpp::Time last_odom_time_;
    bool have_last_odom_ = false;


    std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf2_listener;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
        bezier_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        vision_localization_sub_;

    rclcpp::Subscription<uart_bridge::msg::AutoAim>::SharedPtr autoaim_sub_;

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr alive_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_tagslam_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr gimbal_data_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr autopath_pub_;

    rclcpp::TimerBase::SharedPtr odom_from_mcb_timer_;
    rclcpp::TimerBase::SharedPtr odom_to_mcb_timer_;

    // uart thingies
    src::uart::OdometryMessage odom_msg;
    src::uart::AutoAimMessage autoaim_msg;
    src::uart::AliveMessage alive_msg;
    src::uart::AutoPathMessage auto_path_msg;
    src::uart::VisionLocalizationMessage vision_localization_msg;

    std::unique_ptr<src::uart::Uart> uart_;
    std::unique_ptr<src::uart::IBusReader> flysky_reader_;

    std::thread rx_thread_;

    void auto_path_message_callback(
        std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        RCLCPP_DEBUG(this->get_logger(), "autopath_callback");
        auto_path_msg.start_point_x = msg->data[0];
        auto_path_msg.start_point_y = msg->data[1];
        auto_path_msg.end_point_x = msg->data[2];
        auto_path_msg.end_point_y = msg->data[3];
        auto_path_msg.control_point1_x = msg->data[4];
        auto_path_msg.control_point1_y = msg->data[5];
        auto_path_msg.control_point2_x = msg->data[6];
        auto_path_msg.control_point2_y = msg->data[7];
        auto_path_msg.length = msg->data[8];
        uart_->send(
            std::make_unique<src::uart::AutoPathMessage>(auto_path_msg));
    }

    void autoaim_callback(uart_bridge::msg::AutoAim::SharedPtr msg) {
        RCLCPP_DEBUG(this->get_logger(), "received autoaim msg");
        autoaim_msg.yaw_error = msg->yaw_error;
        autoaim_msg.pitch_error = msg->pitch_error;
        autoaim_msg.distance = msg->distance;
        autoaim_msg.target_id = msg->target_id;
        uart_->send(std::make_unique<src::uart::AutoAimMessage>(autoaim_msg));
    }

void publish_odom() {
    nav_msgs::msg::Odometry odom_msg_ros;
    geometry_msgs::msg::Vector3Stamped gimbal_data;

    long long mcb_generation_time_us = odom_msg.timestamp;
    long long offset_us = jetson_to_mcb_offset_us.load();
    long long true_jetson_time_us = mcb_generation_time_us + offset_us;
    rclcpp::Time true_ros_time(true_jetson_time_us * 1000);

    double yaw = -odom_msg.yaw;

    gimbal_data.header.stamp = true_ros_time;
    gimbal_data.header.frame_id = "base_link";
    gimbal_data.vector.x = 0.0;               // roll (unused)
    gimbal_data.vector.y = odom_msg.pitch;    // pitch
    gimbal_data.vector.z = yaw;               // yaw
    gimbal_data_pub_->publish(gimbal_data);


    odom_msg_ros.header.stamp = true_ros_time;
    odom_msg_ros.header.frame_id = "odom";
    odom_msg_ros.child_frame_id = "base_link";

    // velocities in the fixed odom frame (your existing "global" convention)
    double global_vx = odom_msg.vel_y;
    double global_vy = -odom_msg.vel_x;

    // --- integrate global velocity into an accumulated odom-frame position ---
    if (have_last_odom_) {
        double dt = (true_ros_time - last_odom_time_).seconds();
        if (dt > 0.0 && dt < 1.0) {            // guard against bad/duplicate stamps
            pos_x_ += global_vx * dt;
            pos_y_ += global_vy * dt;
        }
    }
    last_odom_time_ = true_ros_time;
    have_last_odom_ = true;

    odom_msg_ros.pose.pose.position.x = pos_x_;
    odom_msg_ros.pose.pose.position.y = pos_y_;
    odom_msg_ros.pose.pose.position.z = 0.0;

    // orientation: yaw only (see note on pitch below)
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    odom_msg_ros.pose.pose.orientation.x = q.getX();
    odom_msg_ros.pose.pose.orientation.y = q.getY();
    odom_msg_ros.pose.pose.orientation.z = q.getZ();
    odom_msg_ros.pose.pose.orientation.w = q.getW();

    // twist still fine to publish for other consumers; TagSLAM ignores it
    double local_vx =  (global_vx * cos(yaw)) + (global_vy * sin(yaw));
    double local_vy = -(global_vx * sin(yaw)) + (global_vy * cos(yaw));
    odom_msg_ros.twist.twist.linear.x  = local_vx;
    odom_msg_ros.twist.twist.linear.y  = local_vy;
    odom_msg_ros.twist.twist.angular.z = -odom_msg.yaw_vel;

    odometry_tagslam_pub_->publish(odom_msg_ros);
}

void transform_callback() {
        geometry_msgs::msg::TransformStamped transform;

        try {
            // Get the absolute newest transform available right now
            transform = tf2_buffer_->lookupTransform(
                "map", "base_link",
                tf2::TimePointZero // This means "give me the latest"
            );

            rclcpp::Time ros_time = transform.header.stamp;
            long long ros_time_us = ros_time.nanoseconds() / 1000;

            long long offset_us = jetson_to_mcb_offset_us.load();

            long long mcb_historical_time_us = ros_time_us - offset_us;
            // ----------------------------

            vision_localization_msg.x = transform.transform.translation.x;
            vision_localization_msg.y = transform.transform.translation.y;
            vision_localization_msg.heading = tf2::getYaw(transform.transform.rotation);

            vision_localization_msg.timestamp = (uint32_t)mcb_historical_time_us;

            uart_->send(std::make_unique<src::uart::VisionLocalizationMessage>(
                vision_localization_msg));

        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Could not get transform: %s", ex.what());
        }
    };

public:
    UartBridge() : Node("uart_bridge") {
        alive_pub_ = this->create_publisher<std_msgs::msg::Empty>(
            "uart/heartbeat", rclcpp::SensorDataQoS());
        rclcpp::QoS qos_profile(10);
        qos_profile.reliable(); // needed like this for tagslam
        odometry_tagslam_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "uart/odometry", qos_profile);

        gimbal_data_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("uart/gimbal_data", 10);



        bezier_sub_ =
            this->create_subscription<std_msgs::msg::Float32MultiArray>(
                "/planner/bezier_curve", 10,
                std::bind(&UartBridge::auto_path_message_callback, this,
                          std::placeholders::_1));

        autoaim_sub_ = this->create_subscription<uart_bridge::msg::AutoAim>(
            "/autoaim", 10,
            std::bind(&UartBridge::autoaim_callback, this,
                      std::placeholders::_1));

        // vision_localization_sub_ =
        //     this->create_subscription<nav_msgs::msg::Odometry>(
        //         "/odom/body_rig", 10,
        //         std::bind(&UartBridge::vis,
        //         this,
        //                   std::placeholders::_1));

        odom_to_mcb_timer_ = this->create_wall_timer(
            333ms, std::bind(&UartBridge::transform_callback, this));

        tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf2_listener =
            std::make_unique<tf2_ros::TransformListener>(*tf2_buffer_);

        start();
    }

    // Clean up the thread safely when the node shuts down
    ~UartBridge() {
        if (uart_) {
            uart_->close(); // Signal
                            // the
                            // UART
                            // to
                            // stop
                            // its
                            // receive
                            // loop
        }
        if (rx_thread_.joinable()) {
            rx_thread_.join(); // or
                               // detach(),
                               // depending
                               // on how
                               // your
                               // UART
                               // stops
                               // reading
        }
    }

    void start() {
        auto odom_handler =
            std::make_unique<src::uart::OdometryHandler>(odom_msg);

        std::map<src::uart::UartMessage::MessageType,
                 std::unique_ptr<src::uart::UartRxHandler>>
            handlers;
        handlers[src::uart::UartMessage::ODOMETRY] = std::move(odom_handler);

        src::uart::UartConfig uart_config{.port = "/dev/ttyTHS1",
                                          .baud = 115200};

        uart_ = std::make_unique<src::uart::Uart>(uart_config,
                                                  std::move(handlers), true);

        flysky_reader_ = std::make_unique<src::uart::IBusReader>("/dev/ttyUSB0", uart_.get());
        flysky_reader_->start();


        rx_thread_ = std::thread([this]() {
            if (this->uart_) {
                this->uart_->receive();
            }
        });

        odom_from_mcb_timer_ = this->create_wall_timer(
            10ms, std::bind(&UartBridge::publish_odom, this));
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UartBridge>());
    rclcpp::shutdown();
}
