import launch
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    pkg_share = get_package_share_directory('localization')
    camera_config = os.path.join(pkg_share, 'config', 'camera.yaml')
    cameras_config = os.path.join(pkg_share, 'config', 'cameras.yaml')
    tagslam_config = os.path.join(pkg_share, 'config', 'tagslam.yaml')
    camera_poses_config = os.path.join(pkg_share, 'config', 'camera_poses.yaml')
    ekf_local_config = os.path.join(pkg_share, 'config', 'ekf_local.yaml')
    ekf_global_config = os.path.join(pkg_share, 'config', 'ekf_global.yaml')
      
    launch_nodes = []

    cameras = {
        'front': {'serial': '827312073868', 'x': '0.083',  'y': '0.011',  'z': '0.595', 'yaw': '0.0'},
        'left':  {'serial': '851112061763', 'x': '-0.031', 'y': '0.080',  'z': '0.595', 'yaw': '1.5708'},
        'right': {'serial': '827312073427', 'x': '0.036',  'y': '-0.081', 'z': '0.595', 'yaw': '-1.5708'}
    }

    uart_bridge = Node(
        package="uart_bridge",
        executable="uart_bridge_node",
        output="screen",
        name="uart_bridge",
    )

    sync_and_detect = ComposableNode(
        package="tagslam",
        plugin="tagslam::SyncAndDetect",
        name="sync_and_detect",
        parameters=[{"cameras": cameras_config,
                     "tagslam_config": tagslam_config,
                     "use_sim_time": False,
                     "use_approximate_sync": True}],
        remappings=[],
        extra_arguments=[{'use_intra_process_comms' : True}],
    )

    tagslam = ComposableNode(
        package="tagslam",
        plugin="tagslam::TagSLAM",
        name="tagslam",
        parameters=[{"cameras": cameras_config,
                     "tagslam_config": tagslam_config,
                     "camera_poses": camera_poses_config,
                     "use_sim_time": False,
                     "use_approximate_sync": True}],
        remappings=[],
        extra_arguments=[{'use_intra_process_comms' : True}],
    )

    # ekf_node = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekf_filter_node',
    #     output='screen',
    #     parameters=[ekf_config]
    # )

    ekf_local_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local_node',
        output='screen',
        parameters=[ekf_local_config],
        remappings=[('odometry/filtered', 'odometry/local')]
    )

    # Global EKF (map -> odom)
    ekf_global_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global_node',
        output='screen',
        parameters=[ekf_global_config],
        remappings=[('odometry/filtered', 'odometry/global')] # Prevents topic collision with local EKF
    )

    cov_filter = Node(
        package="localization",
        executable="cov_filter_node",
        output="screen",
        name="cov_filter",
    )


    launch_nodes.extend([sync_and_detect, tagslam])
   
    detection_topics = []
    for name, config in cameras.items():

        tf = ComposableNode(
            package='tf2_ros',
            name=f'tf_{name}',
            plugin='tf2_ros::StaticTransformBroadcasterNode',
            parameters=[{"x": config['x'],
                         "y": config['y'],
                         "z": config['z'],
                         "yaw": config['yaw'],
                         "pitch": '0',
                         "roll": '0',
                         "frame-id": 'rig',
                         "child-frame-id": f'{name}_cam_link',
                         "use_sim_time": False,
                         "use_approximate_sync": True}],
            # arguments=[
            #     '--x', config['x'], '--y', config['y'], '--z', config['z'],
            #     '--yaw', config['yaw'], '--pitch', '0', '--roll', '0',
            #     '--frame-id', 'rig', '--child-frame-id', f'{name}_cam_link',
            # ],
            extra_arguments=[{'use_intra_process_comms' : True}],
        )

        realsense_node = ComposableNode(
            package='realsense2_camera',
            plugin='realsense2_camera::RealSenseNodeFactory',
            name='camera',
            namespace=name,
            parameters=[camera_config, {
                'serial_no': config['serial'],
                'camera_name': f'{name}_cam',
            }],
            extra_arguments=[{'use_intra_process_comms' : True}],
        )

        launch_nodes.extend([realsense_node, tf])


    container = ComposableNodeContainer(
        name='sentry_stack',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=launch_nodes,
        output='both',
    )

    return launch.LaunchDescription([container, uart_bridge, ekf_local_node, ekf_global_node, cov_filter])
