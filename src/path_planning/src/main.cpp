#include <rclcpp/rclcpp.hpp>

#include "auto_drive/theta_star.hpp"
#include "ros_vis.hpp"

int main(int argc, char** argv)
{
    // 1. Initialize ROS 2
    rclcpp::init(argc, argv);
    auto ros_viz_node = std::make_shared<src::viz::RosVisualizer>();

    src::AutoPathing::ThetaStar thetaStar;

    while(rclcpp::ok()) // Check if ROS is still running
    {
        thetaStar.updateDynamicObstacles();
        rclcpp::spin_some(ros_viz_node);
    }

    rclcpp::shutdown();
    return 0;
}