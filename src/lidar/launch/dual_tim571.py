"""
Dual SICK TiM571 launch for northstar-robomaster.

Front scanner faces +x, back scanner is mounted behind base_link and rotated
180 deg so its +x points aft.

Each driver instance needs a unique nodename, hostname and topic names --
sick_scan_xd defaults every node to "sick_scan" on 192.168.0.1 publishing to
"cloud"/"scan", so without overrides the second node collides with the first.

TF is published by the driver itself (tf_base_frame_id -> frame_id) using
tf_base_lidar_xyz_rpy.  Point cloud data is NOT pre-rotated: add_transform_xyz_rpy
is left at identity so the raw scan stays in each sensor's own frame and all
geometry lives in TF, which is what robot_localization and the AprilTag stack
expect.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ---------------------------------------------------------------------------
# EDIT THESE
# ---------------------------------------------------------------------------
FRONT_IP = "192.168.10.1"
BACK_IP = "192.168.10.2"

# Mounting offsets relative to base_link, metres and radians.
# x forward, y left, z up, yaw CCW-positive looking down.
#   0.1016 m == 4 inches.  Measure yours and replace.
FRONT_XYZ_RPY = "0.10,0.0,0.0,0.0,0.0,0.0"
BACK_XYZ_RPY = "-0.10,0.0,0.0,0.0,0.0,3.14159265"

BASE_FRAME = "base_link"
# ---------------------------------------------------------------------------

SICK_LAUNCH = os.path.join(
    get_package_share_directory("sick_scan_xd"),
    "launch",
    "sick_tim_5xx.launch",
)


def tim_node(name, hostname, frame_id, xyz_rpy):
    """One sick_generic_caller instance with all per-sensor overrides."""
    return Node(
        package="sick_scan_xd",
        executable="sick_generic_caller",
        name=name,
        output="screen",
        arguments=[
            SICK_LAUNCH,
            f"nodename:={name}",
            f"hostname:={hostname}",
            f"frame_id:={frame_id}",
            f"cloud_topic:=cloud_{name}",
            f"laserscan_topic:=scan_{name}",
            # Driver-published TF: BASE_FRAME -> frame_id
            f"tf_base_frame_id:={BASE_FRAME}",
            f"tf_base_lidar_xyz_rpy:={xyz_rpy}",
            "tf_publish_rate:=10.0",
            # Leave the cloud itself untransformed.
            "add_transform_xyz_rpy:=0,0,0,0,0,0",
        ],
    )


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "front_ip", default_value=FRONT_IP,
            description="IP of the forward-facing TiM571",
        ),
        DeclareLaunchArgument(
            "back_ip", default_value=BACK_IP,
            description="IP of the rear-facing TiM571",
        ),
        tim_node("tim_front", FRONT_IP, "laser_front", FRONT_XYZ_RPY),
        tim_node("tim_back", BACK_IP, "laser_back", BACK_XYZ_RPY),
    ])