#ifndef ROS_VISUALIZER_HPP
#define ROS_VISUALIZER_HPP

#include <Eigen/src/Core/Matrix.h>
#include <chrono>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <eigen3/Eigen/Dense>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

// Correct TF2 headers for ROS 2 Jazzy
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "auto_drive/theta_star.hpp"
#include "auto_drive/cubic_bezier.hpp"
#include "auto_drive/cubic_bezier_fitter.hpp"

using namespace std::chrono_literals;

namespace src::viz {

class RosVisualizer : public rclcpp::Node {
public:
    RosVisualizer() : Node("path_planner_visualizer") {
        // Initialize TF2 Buffer and Listener
        tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

        grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("planner/map", 1);
        bezier_data_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("planner/bezier_curve", 10);

        bezier_published_visualizer_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("planner/published_bezier", 10);
        bezier_current_visualizer_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("planner/current_bezier", 10);

        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("goal_pose", 10, std::bind(&RosVisualizer::goal_callback, this, std::placeholders::_1)
);

        thetaStar = new src::AutoPathing::ThetaStar();
        lastGridPublishTime = this->now();

        // Timer to call the transform callback periodically (approx 30Hz)
        odom_timer_ = this->create_wall_timer(
            33ms, std::bind(&RosVisualizer::transform_callback, this));
    }

    void transform_callback() {
        geometry_msgs::msg::TransformStamped transform;

        try {
            // Lookup the latest transform from map to rig
            transform = tf2_buffer_->lookupTransform(
                "map", "rig",
                tf2::TimePointZero
            );

            double x_offset = transform.transform.translation.x;
            double y_offset = transform.transform.translation.y;

            // Map TF to internal state (maintaining your specific coordinate mapping)
            currentRobotWorldPos = Eigen::Vector2f(-y_offset, x_offset);
            hasRobotPos = true;

            RCLCPP_DEBUG(this->get_logger(), "Robot Pos Updated: X: %f, Y: %f",
                        currentRobotWorldPos.x(), currentRobotWorldPos.y());

        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Could not get transform: %s", ex.what());
        }
    }

    AutoPathing::CubicBezier translateBezierFromGridToWorld(AutoPathing::CubicBezier bezier) {
        return AutoPathing::CubicBezier(
            thetaStar->ConvertGridToWorld(bezier.start), 
            thetaStar->ConvertGridToWorld(bezier.end), 
            thetaStar->ConvertGridToWorld(bezier.controlEnd), 
            thetaStar->ConvertGridToWorld(bezier.controlStart)
        );
    }

    void publishBezierData(AutoPathing::CubicBezier& bezierToPub) {
        publishedBezier = bezierToPub;

        std_msgs::msg::Float32MultiArray arr;
        arr.data = {
            bezierToPub.start.x(), bezierToPub.start.y(), 
            bezierToPub.end.x(), bezierToPub.end.y(), 
            bezierToPub.controlStart.x(), bezierToPub.controlStart.y(), 
            bezierToPub.controlEnd.x(), bezierToPub.controlEnd.y(), 
            bezierToPub.length
        };
        bezier_data_pub_->publish(arr);
    }

    void publishBezierVisualizer(AutoPathing::CubicBezier& currentBezier, rclcpp::Time& currentTime) {
        visualization_msgs::msg::Marker bezier_published_marker;
        bezier_published_marker.header.frame_id = "map";
        bezier_published_marker.header.stamp = currentTime;
        bezier_published_marker.ns = "bezier_published";
        bezier_published_marker.id = 0;
        bezier_published_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        bezier_published_marker.action = visualization_msgs::msg::Marker::ADD;
        bezier_published_marker.scale.x = 0.02; 
        bezier_published_marker.color.r = 0.0; bezier_published_marker.color.g = 1.0; bezier_published_marker.color.b = 0.0; bezier_published_marker.color.a = 1.0; 

        for (int i = 0; i < BEZIER_VIS_SAMPLES; i++) {
            float t = (float)i / (float)(BEZIER_VIS_SAMPLES - 1);
            Eigen::Vector2f pt = publishedBezier.Evaluate(t);
            geometry_msgs::msg::Point p;
            p.x = pt.x(); p.y = pt.y(); p.z = 0.1;
            bezier_published_marker.points.push_back(p);
        }

        visualization_msgs::msg::Marker bezier_current_marker;
        bezier_current_marker.header.frame_id = "map";
        bezier_current_marker.header.stamp = currentTime;
        bezier_current_marker.ns = "bezier_current";
        bezier_current_marker.id = 0;
        bezier_current_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        bezier_current_marker.action = visualization_msgs::msg::Marker::ADD;
        bezier_current_marker.scale.x = 0.02; 
        bezier_current_marker.color.r = 1.0; bezier_current_marker.color.g = 0.0; bezier_current_marker.color.b = 0.0; bezier_current_marker.color.a = 0.8; 

        for (int i = 0; i < BEZIER_VIS_SAMPLES; i++) {
            float t = (float)i / (float)(BEZIER_VIS_SAMPLES - 1);
            Eigen::Vector2f pt = currentBezier.Evaluate(t);
            geometry_msgs::msg::Point p;
            p.x = pt.x(); p.y = pt.y(); p.z = 0.1;
            bezier_current_marker.points.push_back(p);
        }

        bezier_published_visualizer_pub_->publish(bezier_published_marker);
        bezier_current_visualizer_pub_->publish(bezier_current_marker);
    }

    void publishGrid(rclcpp::Time& currentTime) {
        int gridSizeX = thetaStar->GetSizeX();
        int gridSizeY = thetaStar->GetSizeY();
        float realSizeX = thetaStar->GetRealSizeX();

        nav_msgs::msg::OccupancyGrid grid_msg;
        grid_msg.header.stamp = currentTime;
        grid_msg.header.frame_id = "map";
        grid_msg.info.resolution = realSizeX / gridSizeX;
        grid_msg.info.width = gridSizeX;
        grid_msg.info.height = gridSizeY;
        grid_msg.info.origin.position.x = -realSizeX / 2.0f;
        grid_msg.info.origin.position.y = -thetaStar->GetRealSizeY() / 2.0f;
        grid_msg.info.origin.orientation.w = 1.0;

        grid_msg.data.resize(grid_msg.info.width * grid_msg.info.height);
        for (int y = 0; y < gridSizeY; ++y) {
            for (int x = 0; x < gridSizeX; ++x) {
                int val = (int)thetaStar->getWorkingGrid(x, y);
                int index = x + y * gridSizeX;
                if (val == 0) grid_msg.data[index] = 0;
                else if (val == 1) grid_msg.data[index] = 100;
                else if (val == 2) grid_msg.data[index] = 80;
                else if (val == 3) grid_msg.data[index] = 60;
                else grid_msg.data[index] = 20;
            }
        }
        grid_pub_->publish(grid_msg);
        lastGridPublishTime = currentTime;
    }

    void update() {
        if (!hasRobotPos) return;

        rclcpp::Time currentTime = this->now();
        thetaStar->updateDynamicObstacles();
        currentWorldGoalPos = calculateWorldGoalPos();
        currentGridGoalPos = thetaStar->ConvertWorldToGrid(currentWorldGoalPos);

        AutoPathing::CubicBezier currentBezier = calculateCurrentBezierToGoal();

        if (currentBezier.end != publishedBezier.end) {
            this->publishBezierData(currentBezier);
        }

        this->publishBezierVisualizer(currentBezier, currentTime);

        if ((currentTime - lastGridPublishTime).seconds() > GRID_PUB_WAIT) {
            this->publishGrid(currentTime);
        }
    }

    std::vector<Eigen::Vector2f> goalPoses = { Eigen::Vector2f(-1, 1), Eigen::Vector2f(1, -1), Eigen::Vector2f(-2.75, -1.15), Eigen::Vector2f(-5.25, 0), Eigen::Vector2f(-2.5, 2.5) };
    int goalIndex = 0;
    float goalError = 0.025; // meters
    Eigen::Vector2f calculateWorldGoalPos() {
        Eigen::Vector2f goal = goalPoses[goalIndex];
        float distanceToGoal = (goal - currentRobotWorldPos).norm();

        if (distanceToGoal <= goalError)
        {
            goalIndex++; 
            if (goalIndex >= goalPoses.size()) { goalIndex = 0; }

            goal = goalPoses[goalIndex];
        }
        
        return goal;
    }

    AutoPathing::CubicBezier calculateCurrentBezierToGoal() {
        std::vector<Eigen::Vector2i> fullGridPath = thetaStar->FindPath(
            thetaStar->ConvertWorldToGrid({currentRobotWorldPos.x(), currentRobotWorldPos.y()}), 
            currentGridGoalPos
        );
        
        size_t flipIdx = src::AutoPathing::CubicBezierFitter::findFirstConcavityFlip(fullGridPath);
        std::vector<Eigen::Vector2i> gridPathUntilConcavityFlip(fullGridPath.begin(), fullGridPath.begin() + flipIdx);
        std::vector<Eigen::Vector2f> populatedPath = src::AutoPathing::CubicBezierFitter::PopulatePointsNew(gridPathUntilConcavityFlip);

        return translateBezierFromGridToWorld(src::AutoPathing::CubicBezierFitter::FitCubic(populatedPath, 30, 1, .001));
    }

private:
    const int BEZIER_VIS_SAMPLES = 24;
    const float GRID_PUB_WAIT = 3.0f;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr bezier_data_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr bezier_published_visualizer_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr bezier_current_visualizer_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        currentWorldGoalPos = Eigen::Vector2f(-msg->pose.position.y, msg->pose.position.x);

        RCLCPP_INFO(this->get_logger(), "New Goal Set: X: %f, Y: %f", 
                    currentWorldGoalPos.x(), currentWorldGoalPos.y());
    }

    std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
    rclcpp::TimerBase::SharedPtr odom_timer_;

    src::AutoPathing::ThetaStar* thetaStar;
    src::AutoPathing::CubicBezier publishedBezier;

    rclcpp::Time lastGridPublishTime;

    Eigen::Vector2f currentWorldGoalPos;
    Eigen::Vector2i currentGridGoalPos;

    bool hasRobotPos = false;
    Eigen::Vector2f currentRobotWorldPos;
};

} // namespace src::viz

#endif