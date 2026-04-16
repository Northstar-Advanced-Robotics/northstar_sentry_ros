#ifndef ROS_VISUALIZER_HPP
#define ROS_VISUALIZER_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <eigen3/Eigen/Dense>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include "auto_drive/theta_star.hpp"
#include "auto_drive/cubic_bezier.hpp"
#include "auto_drive/cubic_bezier_fitter.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

namespace src::viz {

class RosVisualizer : public rclcpp::Node {
public:
    RosVisualizer() : Node("path_planner_visualizer") {
        position_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom/body_rig", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                currentRobotWorldPos = Eigen::Vector2f(msg->pose.pose.position.x, msg->pose.pose.position.y); 
            });
        // health_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        //     "/health", 10,
        //     [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        //         // Store the clicked point (World Coordinates)
        //         current_goal_ = Eigen::Vector2f(msg->point.x, msg->point.y);
        //         RCLCPP_INFO(this->get_logger(), "Goal Updated: x=%f, y=%f", msg->point.x, msg->point.y);
        //     });
        curve_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/bezier_curve", 10);

        thetaStar = new src::AutoPathing::ThetaStar();
    }

    void publishBezierCurve(AutoPathing::CubicBezier& bezierToPub)
    {
        publishedBezier = bezierToPub;

        std_msgs::msg::Float32MultiArray arr = std_msgs::msg::Float32MultiArray();
        arr.data = {bezierToPub.start.x(), bezierToPub.start.y(), bezierToPub.end.x(), bezierToPub.end.y(), bezierToPub.controlStart.x(), bezierToPub.controlStart.y(), bezierToPub.controlEnd.x(), bezierToPub.controlEnd.y(), bezierToPub.length};
        curve_pub_->publish(arr);
    }

    void update()
    {
        thetaStar->updateDynamicObstacles();
        currentWorldGoalPos = calculateGoalPos();
        currentGridGoalPos = thetaStar->ConvertWorldToGrid(currentWorldGoalPos);

        AutoPathing::CubicBezier currentBezier = calculateCurrentBezierToGoal();

        if (currentBezier.end != publishedBezier.end)
        {
            this->publishBezierCurve(currentBezier);
        }
    }

    Eigen::Vector2f calculateGoalPos()
    {
        Eigen::Vector2f goalPos = {0, 0};
        return goalPos;
    }

    AutoPathing::CubicBezier calculateCurrentBezierToGoal()
    {
        std::vector<Eigen::Vector2i> fullGridPath = thetaStar->FindPath(thetaStar->ConvertWorldToGrid({currentRobotWorldPos.x(), currentRobotWorldPos.y()}), currentGridGoalPos);
        std::vector<Eigen::Vector2i> gridPathUntilConcavityFlip = std::vector<Eigen::Vector2i>(fullGridPath.begin(), fullGridPath.begin() + src::AutoPathing::CubicBezierFitter::findFirstConcavityFlip(fullGridPath));
        std::vector<Eigen::Vector2f> populatedPathUntilConcavityFlip = src::AutoPathing::CubicBezierFitter::PopulatePointsNew(gridPathUntilConcavityFlip);

        return src::AutoPathing::CubicBezierFitter::FitCubic(populatedPathUntilConcavityFlip, 30, 1, .001);
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr position_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr health_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr curve_pub_;

    src::AutoPathing::ThetaStar* thetaStar;
    src::AutoPathing::CubicBezier publishedBezier;

    Eigen::Vector2f currentWorldGoalPos;
    Eigen::Vector2i currentGridGoalPos;

    Eigen::Vector2f currentRobotWorldPos;
};

} // namespace src::viz

#endif
