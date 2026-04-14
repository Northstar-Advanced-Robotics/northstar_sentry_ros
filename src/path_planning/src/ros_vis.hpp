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

#include "auto_drive/theta_star.hpp"
#include "auto_drive/cubic_bezier.hpp"
#include "auto_drive/cubic_bezier_fitter.hpp"

namespace src::viz {

class RosVisualizer : public rclcpp::Node {
public:
    RosVisualizer() : Node("path_planner_visualizer") {
        /*position_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/position", 10,
            [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
                // Store the clicked point (World Coordinates)
                current_goal_ = Eigen::Vector2f(msg->point.x, msg->point.y);
                RCLCPP_INFO(this->get_logger(), "Goal Updated: x=%f, y=%f", msg->point.x, msg->point.y);
            });
        health_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/health", 10,
            [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
                // Store the clicked point (World Coordinates)
                current_goal_ = Eigen::Vector2f(msg->point.x, msg->point.y);
                RCLCPP_INFO(this->get_logger(), "Goal Updated: x=%f, y=%f", msg->point.x, msg->point.y);
            });*/
        curve_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/bezier_curve", 10);
    }

    void setThetaStarInstance(src::AutoPathing::ThetaStar& thetaStar)
    {
        this->thetaStar = &thetaStar;
    }

    void publishBezierCurve(AutoPathing::CubicBezier& bezierToPub)
    {
        //curve_pub_->publish(bezierToPub); // this is probably going to have to be translated from our cubic bezier def to ROS'
    }

    void receiveRobotPosition(Eigen::Vector2f worldSpaceRobotPos)
    {
        Eigen::Vector2i goalGridSpace = {0, 0};

        std::vector<Eigen::Vector2i> fullGridPath = thetaStar->FindPath(thetaStar->ConvertWorldToGrid({0,0/*odom_msg.pos_x,odom_msg.pos_y*/}),goalGridSpace);
        std::vector<Eigen::Vector2i> gridPathUntilConcavityFlip;
        std::vector<Eigen::Vector2f> populatedPathUntilConcavityFlip;

        gridPathUntilConcavityFlip = std::vector<Eigen::Vector2i>(fullGridPath.begin(), fullGridPath.begin() + src::AutoPathing::CubicBezierFitter::findFirstConcavityFlip(fullGridPath));
        populatedPathUntilConcavityFlip = src::AutoPathing::CubicBezierFitter::PopulatePointsNew(gridPathUntilConcavityFlip);

        AutoPathing::CubicBezier calculatedBezier = src::AutoPathing::CubicBezierFitter::FitCubic(populatedPathUntilConcavityFlip, 30, 1, .001);
        this->publishBezierCurve(calculatedBezier);
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr position_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr health_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr curve_pub_;

    src::AutoPathing::ThetaStar* thetaStar;
};

} // namespace src::viz

#endif
