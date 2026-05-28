#include <rclcpp/rclcpp.hpp>
#include <isaac_ros_apriltag_interfaces/msg/april_tag_detection_array.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>

using std::placeholders::_1;

class TagTranslator : public rclcpp::Node {
private:
    rclcpp::Subscription<isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>::SharedPtr sub_;
    rclcpp::Publisher<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr pub_;

    void tag_callback(const isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray::SharedPtr isaac_msg) {
        // Create the standard ROS 2 message
        apriltag_msgs::msg::AprilTagDetectionArray std_msg;
        
        // Copy the header (crucial for TF timing)
        std_msg.header = isaac_msg->header;

        // Copy each detection
        for (const auto& isaac_det : isaac_msg->detections) {
            apriltag_msgs::msg::AprilTagDetection std_det;

            std_det.family = isaac_det.family;
            std_det.id = isaac_det.id;

            // British spelling in apriltag_msgs
            std_det.centre.x = isaac_det.center.x;
            std_det.centre.y = isaac_det.center.y;

            for (int i = 0; i < 4; ++i) {
                std_det.corners[i].x = isaac_det.corners[i].x;
                std_det.corners[i].y = isaac_det.corners[i].y;
            }

            std_msg.detections.push_back(std_det);
        }
        // Publish to TagSLAM
        pub_->publish(std_msg);
    }

public:
    TagTranslator(const std::string& node_name, const std::string& input_topic, const std::string& output_topic) 
    : Node(node_name) {
        sub_ = this->create_subscription<isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>(
            input_topic, rclcpp::SensorDataQoS(), std::bind(&TagTranslator::tag_callback, this, _1));

        pub_ = this->create_publisher<apriltag_msgs::msg::AprilTagDetectionArray>(
            output_topic, rclcpp::QoS(10).reliable());
            
        RCLCPP_INFO(this->get_logger(), "Translating %s -> %s", input_topic.c_str(), output_topic.c_str());
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    
    // Create an executor to run multiple translators if you have multiple cameras
    rclcpp::executors::SingleThreadedExecutor executor;
    
    // Example: Instantiate one translator per camera
    auto trans_left = std::make_shared<TagTranslator>("trans_left", "/left/detector/tags_nvidia", "/left/detector/tags");
    auto trans_back = std::make_shared<TagTranslator>("trans_back", "/back/detector/tags_nvidia", "/back/detector/tags");
    auto trans_right = std::make_shared<TagTranslator>("trans_right", "/right/detector/tags_nvidia", "/right/detector/tags");
    
    executor.add_node(trans_left);
    executor.add_node(trans_back);
    executor.add_node(trans_right);
    
    executor.spin();
    rclcpp::shutdown();
    return 0;
}