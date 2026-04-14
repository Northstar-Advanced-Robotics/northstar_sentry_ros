#include "nav_msgs/msg/odometry.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/time.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/empty.hpp>

#include <cstdio>
#include <thread>

#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/utilities.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <uart/messages/odometry_message.hpp>
#include <uart/messages/autoaim_message.hpp>
#include <uart/messages/alive_message.hpp>
#include <uart/messages/auto_path_message.hpp>
#include <uart/handlers/odometry_handler.hpp>
#include <uart/uart.hpp>

class UartBridge : public rclcpp::Node {
private: 
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr alive_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr autoaim_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr autopath_pub_;

  // uart thingies
  src::uart::OdometryMessage odom_msg;
  src::uart::AutoAimMessage autoaim_msg;
  src::uart::AliveMessage alive_msg;
  src::uart::AutoPathMessage auto_path_msg;

  void publish_odom() {
      nav_msgs::msg::Odometry odom_msg_ros;
      odom_msg_ros.header.stamp = this->now();
      odom_msg_ros.header.frame_id = "odom";
      odom_msg_ros.child_frame_id = "base_link";

      double global_vx = odom_msg.vel_y;
      double global_vy = -odom_msg.vel_x;

      // 2. Get Robot Yaw (Rotation)
      double yaw = -odom_msg.yaw; 

      // 3. Rotate Global -> Local (Inverse Rotation Matrix)
      // local_x = x * cos(yaw) + y * sin(yaw)
      // local_y = -x * sin(yaw) + y * cos(yaw)
      double local_vx = (global_vx * cos(yaw)) + (global_vy * sin(yaw));
      double local_vy = -(global_vx * sin(yaw)) + (global_vy * cos(yaw));

      tf2::Quaternion q;
      q.setRPY(0, 0, -odom_msg.yaw); // Roll, Pitch, Yaw
      odom_msg_ros.pose.pose.orientation = tf2::toMsg(q);

      odom_msg_ros.twist.twist.linear.x = local_vx;   // Send Local Forward
      odom_msg_ros.twist.twist.linear.y = local_vy;   // Send Local Strafe
      odom_msg_ros.pose.pose.position.z = 0.0;

      // odom_msg.twist.twist.angular.x = odom_msg.roll_vel;
      // odom_msg.twist.twist.angular.y = odom_msg.pitch_vel;
      odom_msg_ros.twist.twist.angular.z = -odom_msg.yaw_vel;

      odom_msg_ros.pose.covariance[0]  = 0.01; // X pos covariance
      odom_msg_ros.pose.covariance[7]  = 0.01; // Y pos covariance
      odom_msg_ros.pose.covariance[14] = 1000.0; // Z (High error because we don't measure Z)
      odom_msg_ros.pose.covariance[21] = 1000.0; // Roll
      odom_msg_ros.pose.covariance[28] = 1000.0; // Pitch
      odom_msg_ros.pose.covariance[35] = 0.01;  // Yaw (Turning error accumulates fast)

      odom_msg_ros.twist.covariance[0]  = 0.01; // Vx
      odom_msg_ros.twist.covariance[7]  = 0.01; // Vy
      odom_msg_ros.twist.covariance[14] = 1000.0; 
      odom_msg_ros.twist.covariance[21] = .01; 
      odom_msg_ros.twist.covariance[28] = .01; 
      odom_msg_ros.twist.covariance[35] = .01; // Vyaw

      odometry_pub_->publish(odom_msg_ros);
  }
  
public:
  UartBridge() : Node("uart_bridge") {
    alive_pub_ = this->create_publisher<std_msgs::msg::Empty>("heartbeat", rclcpp::SensorDataQoS());
    odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry", rclcpp::SensorDataQoS());    
    autoaim_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("autoaim", rclcpp::SensorDataQoS());

    start();
  }

  void start() {
    // uart setup
    auto odom_handler = std::make_unique<src::uart::OdometryHandler>(odom_msg);
    std::map<src::uart::UartMessage::MessageType, std::unique_ptr<src::uart::UartRxHandler>> handlers;
    handlers[src::uart::UartMessage::ODOMETRY] = std::move(odom_handler);
    src::uart::UartConfig uart_config{.port = "/dev/ttyTHS1", .baud = 115200};
    src::uart::Uart uart(uart_config, std::move(handlers), true);
    std::thread rx_thread([&uart]() {
        uart.receive();
    });
    
    while(rclcpp::ok()) {
      using namespace std::chrono_literals;
      std::this_thread::sleep_for(33ms);
      publish_odom();
    }
  }

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UartBridge>());
  rclcpp::shutdown();
}
