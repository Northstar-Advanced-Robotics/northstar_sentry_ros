#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <vector>
#include <cmath>
#include <algorithm>
#include <string>
#include <map>

using std::placeholders::_1;

class COVFilter : public rclcpp::Node {
private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr filtered_odom_pub_;
    
    // Store subscriptions for all cameras
    std::vector<rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr> tag_subs_;

    // Struct to hold the latest state of each camera
    struct CameraData {
        int count = 0;
        double total_area = 0.0;
        rclcpp::Time last_seen;
    };
    std::map<std::string, CameraData> camera_states_;

    // --- Tuning Constants ---
    const double MIN_COV_MULTIPLIER = 0.1;   
    const double MAX_COV_MULTIPLIER = 500.0;  
    const double REFERENCE_AREA = 10000.0; // Typical area in px^2 at ~1.5m. Tune this!
    const double STALE_DATA_TIMEOUT = 0.5; // Seconds before ignoring a camera's old data

    void tag_callback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg, const std::string& cam_name) {
        CameraData data;
        data.count = msg->detections.size();
        data.last_seen = this->now();
        
        if (data.count > 0) {
            for (const auto & det : msg->detections) {
                // Approximate area using the corners (0: top-left, 1: top-right, 2: bot-right, 3: bot-left)
                double width = std::abs(det.corners[1].x - det.corners[0].x);
                double height = std::abs(det.corners[3].y - det.corners[0].y);
                data.total_area += (width * height);
            }
        }
        
        // Update the global state for this specific camera
        camera_states_[cam_name] = data;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        auto filtered_msg = *msg; 
        auto current_time = this->now();

        int total_tags = 0;
        double total_area = 0.0;

        // Aggregate data from all cameras, ignoring stale data
        for (const auto& [name, data] : camera_states_) {
            if ((current_time - data.last_seen).seconds() < STALE_DATA_TIMEOUT) {
                total_tags += data.count;
                total_area += data.total_area;
            }
        }

        double avg_area = (total_tags > 0) ? (total_area / total_tags) : 1.0;

        // Area is inversely proportional to distance squared. 
        // We want covariance (uncertainty) to go UP as area goes DOWN.
        double area_factor = REFERENCE_AREA / std::max(avg_area, 1.0);
        double count_factor = std::max(total_tags, 1);
        
        // Multiplier scales quadratically with the area-to-distance relationship
        double multiplier = count_factor > 1 ? .1 : std::pow(area_factor, 2);
        multiplier = std::clamp(multiplier, MIN_COV_MULTIPLIER, MAX_COV_MULTIPLIER);

        // Diagonal of the 6x6 Pose Covariance matrix: [X, Y, Z, R, P, Yaw]
        const std::vector<int> diagonal_indices = {0, 7, 14, 21, 28, 35};
        for (int index : diagonal_indices) {
            filtered_msg.pose.covariance[index] = multiplier;
        }

        filtered_odom_pub_->publish(filtered_msg);

        RCLCPP_INFO(this->get_logger(), "Tags: %d, Area: %.1f, Mult: %.2f", 
                     total_tags, avg_area, multiplier);
    }

public:
    COVFilter() : Node("cov_filter") {
        // Subscribe to the odometry output from TagSLAM
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom/body_rig", 10, std::bind(&COVFilter::odom_callback, this, _1));

        filtered_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/odom/body_rig/filtered", 10);

        // Dynamically create subscriptions for all three cameras using lambdas
        std::vector<std::string> cameras = {"left", "back", "right"};
        for (const auto& cam : cameras) {
            std::string topic = "/" + cam + "/detector/tags";
            
            auto sub = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
                topic, 10, 
                [this, cam](const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg) {
                    this->tag_callback(msg, cam);
                }
            );
            tag_subs_.push_back(sub);
            
            // Initialize the state dictionary for this camera
            camera_states_[cam].last_seen = this->now();
        }

        RCLCPP_INFO(this->get_logger(), "Covariance Filter Node Started. Listening to %zu cameras.", cameras.size());
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<COVFilter>());
    rclcpp::shutdown();
    return 0;
}