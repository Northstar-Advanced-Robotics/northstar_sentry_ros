#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <vector>
#include <cmath>
#include <algorithm>

using std::placeholders::_1;

class COVFilter : public rclcpp::Node {
private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr tag_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr filtered_odom_pub_;

    int last_tag_count_ = 0;
    double last_tag_area_avg_ = 1.0;

    // --- Tuning Constants ---
    const double MIN_COV_MULTIPLIER = 0.1;   
    const double MAX_COV_MULTIPLIER = 50.0;  
    const double REFERENCE_AREA = 5000.0; // Typical area in px^2 at ~1.5m. Tune this!

    void tag_callback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg) {
        last_tag_count_ = msg->detections.size();
        
        if (last_tag_count_ > 0) {
            double total_area = 0.0;
            for (const auto & det : msg->detections) {
                // Approximate area using the corners (0: top-left, 1: top-right, 2: bot-right, 3: bot-left)
                double width = std::abs(det.corners[1].x - det.corners[0].x);
                double height = std::abs(det.corners[3].y - det.corners[0].y);
                total_area += (width * height);
            }
            last_tag_area_avg_ = total_area / last_tag_count_;
        } else {
            last_tag_area_avg_ = 1.0; 
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        auto filtered_msg = *msg; 

        // Area is inversely proportional to distance squared. 
        // We want covariance (uncertainty) to go UP as area goes DOWN.
        double area_factor = REFERENCE_AREA / std::max(last_tag_area_avg_, 1.0);
        double count_factor = 1.0 / std::max(last_tag_count_, 1);
        
        // Multiplier scales quadratically with the area-to-distance relationship
        double multiplier = std::pow(area_factor, 2) * count_factor;
        multiplier = std::clamp(multiplier, MIN_COV_MULTIPLIER, MAX_COV_MULTIPLIER);

        // Diagonal of the 6x6 Pose Covariance matrix: [X, Y, Z, R, P, Yaw]
        const std::vector<int> diagonal_indices = {0, 7, 14, 21, 28, 35};
        for (int index : diagonal_indices) {
            filtered_msg.pose.covariance[index] *= multiplier;
        }

        filtered_odom_pub_->publish(filtered_msg);

        RCLCPP_DEBUG(this->get_logger(), "Tags: %d, Area: %.1f, Mult: %.2f", 
                     last_tag_count_, last_tag_area_avg_, multiplier);
    }

public:
    COVFilter() : Node("cov_filter") {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom/body_rig", 10, std::bind(&COVFilter::odom_callback, this, _1));

        tag_sub_ = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
            "/tag_detections", 10, std::bind(&COVFilter::tag_callback, this, _1));

        filtered_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/odom/body_rig/filtered", 10);

        RCLCPP_INFO(this->get_logger(), "Covariance Filter Node Started.");
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<COVFilter>());
    rclcpp::shutdown();
    return 0;
}