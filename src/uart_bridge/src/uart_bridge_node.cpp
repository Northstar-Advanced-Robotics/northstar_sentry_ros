#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <cstdio>
#include <thread>
#include <memory> // Added for std::unique_ptr

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

using namespace std::chrono_literals;

std::atomic<long long> jetson_to_mcb_offset_us{0};

class UartBridge : public rclcpp::Node {
private: 
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr bezier_sub_;

  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr alive_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr autoaim_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr autopath_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  // uart thingies
  src::uart::OdometryMessage odom_msg;
  src::uart::AutoAimMessage autoaim_msg;
  src::uart::AliveMessage alive_msg;
  src::uart::AutoPathMessage auto_path_msg;
  
  std::unique_ptr<src::uart::Uart> uart_;
  
  std::thread rx_thread_;

  void auto_path_message_callback(std_msgs::msg::Float32MultiArray::SharedPtr msg){
    RCLCPP_DEBUG(this->get_logger(), "running callback");
    auto_path_msg.start_point_x = msg->data[0];
    auto_path_msg.start_point_y = msg->data[1];
    auto_path_msg.end_point_x = msg->data[2];
    auto_path_msg.end_point_y = msg->data[3];
    auto_path_msg.control_point1_x = msg->data[4];
    auto_path_msg.control_point1_y = msg->data[5];
    auto_path_msg.control_point2_x = msg->data[6];
    auto_path_msg.control_point2_y = msg->data[7];
    auto_path_msg.length = msg->data[8];
    uart_->send(std::make_unique<src::uart::AutoPathMessage>(auto_path_msg));
  }

  void publish_odom() {
      nav_msgs::msg::Odometry odom_msg_ros;
      odom_msg_ros.header.stamp = this->now();
      odom_msg_ros.header.frame_id = "odom";
      odom_msg_ros.child_frame_id = "base_link";

      double global_vx = odom_msg.vel_y;
      double global_vy = -odom_msg.vel_x;

      double yaw = -odom_msg.yaw; 

      double local_vx = (global_vx * cos(yaw)) + (global_vy * sin(yaw));
      double local_vy = -(global_vx * sin(yaw)) + (global_vy * cos(yaw));

      tf2::Quaternion q;
      q.setRPY(0, 0, -odom_msg.yaw); 
      odom_msg_ros.pose.pose.orientation = tf2::toMsg(q);

      odom_msg_ros.twist.twist.linear.x = local_vx;  
      odom_msg_ros.twist.twist.linear.y = local_vy;  
      
      odom_msg_ros.pose.pose.position.z = 0.0;

      // odom_msg.twist.twist.angular.x = odom_msg.roll_vel;
      // odom_msg.twist.twist.angular.y = odom_msg.pitch_vel;      
      // odom_msg_ros.twist.twist.angular.z = -odom_msg.yaw_vel;

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
    alive_pub_ = this->create_publisher<std_msgs::msg::Empty>("uart/heartbeat", rclcpp::SensorDataQoS());
    odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("uart/odometry", rclcpp::SensorDataQoS());    
    autoaim_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("uart/autoaim", rclcpp::SensorDataQoS());
    
    // Fixed the bind target: changed from `Detector` to `UartBridge`
    bezier_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/planner/bezier_curve", 10,
      std::bind(&UartBridge::auto_path_message_callback, this, std::placeholders::_1));

    start();
  }

  // Clean up the thread safely when the node shuts down
  ~UartBridge() {
      if (uart_) {
          uart_->close(); // Signal the UART to stop its receive loop
      }
      if (rx_thread_.joinable()) {
          rx_thread_.join(); // or detach(), depending on how your UART stops reading
      }
  }

  void start() {
    auto odom_handler = std::make_unique<src::uart::OdometryHandler>(odom_msg);
    std::map<src::uart::UartMessage::MessageType, std::unique_ptr<src::uart::UartRxHandler>> handlers;
    handlers[src::uart::UartMessage::ODOMETRY] = std::move(odom_handler);
    
    src::uart::UartConfig uart_config{.port = "/dev/ttyTHS1", .baud = 115200};
    
    uart_ = std::make_unique<src::uart::Uart>(uart_config, std::move(handlers), true);
    
    rx_thread_ = std::thread([this]() {
        if (this->uart_) {
            this->uart_->receive();
        }
    });
    
    timer_ = this->create_wall_timer(33ms, std::bind(&UartBridge::publish_odom, this));
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UartBridge>());
  rclcpp::shutdown();
}
