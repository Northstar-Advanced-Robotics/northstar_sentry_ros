#include <rclcpp/rclcpp.hpp>

#include "ros_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<src::viz::RosVisualizer> ros_viz_node =
        std::make_shared<src::viz::RosVisualizer>();

    while (rclcpp::ok())
    {
        rclcpp::spin_some(ros_viz_node);
        ros_viz_node->update();
    }

    rclcpp::shutdown();
    return 0;
}