import launch
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('localization')
    camera_config = os.path.join(pkg_share, 'config', 'camera.yaml')
    cameras_config = os.path.join(pkg_share, 'config', 'cameras.yaml')
    tagslam_config = os.path.join(pkg_share, 'config', 'tagslam.yaml')
    camera_poses_config = os.path.join(pkg_share, 'config', 'camera_poses.yaml')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')
      
    launch_nodes = []

    cameras = {
        'front': {'serial': '827312073868', 'x': '0.083',  'y': '0.011',  'z': '0.595', 'yaw': '0.0'},
        'left':  {'serial': '851112061763', 'x': '-0.031', 'y': '0.080',  'z': '0.595', 'yaw': '1.5708'},
        'right': {'serial': '827312073427', 'x': '0.036',  'y': '-0.081', 'z': '0.595', 'yaw': '-1.5708'}
    }

    sync_and_detect = Node(
        package="tagslam",
        executable="sync_and_detect_node",
        output="screen",
        name="sync_and_detect",
        parameters=[{"cameras": cameras_config,
                     "tagslam_config": tagslam_config,
                     "use_sim_time": False,
                     "use_approximate_sync": True}],
        remappings=[],
    )

    tagslam = Node(
        package="tagslam",
        executable="tagslam_node",
        output="screen",
        name="tagslam",
        parameters=[{"cameras": cameras_config,
                     "tagslam_config": tagslam_config,
                     "camera_poses": camera_poses_config,
                     "use_sim_time": False,
                     "use_approximate_sync": True}],
        remappings=[],
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config]
        # remappings=[('/odometry/filtered', '/odom')]
    )

    launch_nodes.extend([sync_and_detect, tagslam, ekf_node])
   
    detection_topics = []
    for name, config in cameras.items():

        tf = Node(
            package='tf2_ros',
            name=f'tf_{name}',
            executable='static_transform_publisher',
            arguments=[
                '--x', config['x'], '--y', config['y'], '--z', config['z'],
                '--yaw', config['yaw'], '--pitch', '0', '--roll', '0',
                '--frame-id', 'rig', '--child-frame-id', f'{name}_cam_link',
            ]
        )

        realsense_node = Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace=name,
            output="screen",
            parameters=[camera_config, {
                'serial_no': config['serial'],
                'camera_name': f'{name}_cam',
            }],
        )

        launch_nodes.extend([realsense_node, tf])


    return launch.LaunchDescription(launch_nodes)
