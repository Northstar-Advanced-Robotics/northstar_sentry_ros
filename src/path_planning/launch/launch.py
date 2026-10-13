import launch
from launch_ros.actions import Node

def generate_launch_description():
    path_planning = Node(
        package="path_planning",
        executable="path_planning_node",
        output="screen",
        name="path_planning",
    )

    return launch.LaunchDescription([path_planning])
