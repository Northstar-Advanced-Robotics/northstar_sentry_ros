#include <rclcpp/rclcpp.hpp>
#include "ros_vis.hpp"

int main(int argc, char** argv)
{
    // 1. Initialize ROS 2
    rclcpp::init(argc, argv);
    auto ros_viz_node = std::make_shared<src::viz::RosVisualizer>();

    while(rclcpp::ok()) // Check if ROS is still running
    {
        ros_viz_node->update();
        rclcpp::spin_some(ros_viz_node);
    }

    rclcpp::shutdown();
    return 0;
}