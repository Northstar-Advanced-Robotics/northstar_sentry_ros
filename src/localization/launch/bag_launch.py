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
    ekf_config = os.path.join(pkg_share, 'config', 'ekf_combined.yaml')
    arducam_config = os.path.join(pkg_share, 'config', 'arducam.yaml')
    qos_overrides_config = os.path.join(pkg_share, 'config', 'qos_overrides.yaml')
    rviz_config = '/home/ubuntu/realsense_ws/good_config.rviz'

    composable_nodes = []
    tf_nodes = []

    # camera positions relative to robot center
    cameras = {
        'left':  {'serial': '827312073427', 'x': '-0.120', 'y': '0.09355',  'z': '0.320', 'yaw': '1.5708'},
        'back': {'serial': '851112061763', 'x': '-0.116574',  'y': '-0.024074',  'z': '0.319', 'yaw': '3.14159'},
        'right': {'serial': '827312073868', 'x': '-0.055',  'y': '-0.09355', 'z': '0.320', 'yaw': '-1.5708'}
    }

    tf_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'tf_base_link',
            arguments=[
                '--frame-id', 'odom', 
                '--child-frame-id', f'base_link'
            ],
            parameters=[{'use_sim_time': True}],
        )
    
    tf_nodes.append(tf_node)

    uart_bridge = Node(
        package="uart_bridge",
        executable="uart_bridge_node",
        output="screen",
        name="uart_bridge",
            parameters=[{'use_sim_time': True}],
    )

    detector = Node(
        package='detector',
        executable='detector',
        output='screen',
        name='detector_node',
            parameters=[{'use_sim_time': True}],
    )

    path_planning = Node(
        package='path_planning',
        executable='path_planning_node',
        output='screen',
        name='path_planning',
            parameters=[{'use_sim_time': True}],
    )

    # sync_and_detect = ComposableNode(
    #     package="tagslam",
    #     plugin="tagslam::SyncAndDetect",
    #     name="sync_and_detect",
    #     parameters=[{"cameras": cameras_config,
    #                  "tagslam_config": tagslam_config,
    #                  "use_sim_time": False,
    #                  "use_approximate_sync": True,
    #                  "sync_queue_size": 2},
    #                  qos_overrides_config],
    #     remappings=[],
    #     extra_arguments=[{'use_intra_process_comms' : True}],
    # )

    tagslam = ComposableNode(
        package="tagslam",
        plugin="tagslam::TagSLAM",
        name="tagslam",
        parameters=[{"cameras": cameras_config,
                     "tagslam_config": tagslam_config,
                     "camera_poses": camera_poses_config,
                     "use_sim_time": True,
                     "use_approximate_sync": True},
                     qos_overrides_config],
        remappings=[],
        extra_arguments=[{'use_intra_process_comms' : True}],
    )

    ekf_local_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local_node',
        output='screen',
        parameters=[ekf_local_config, {'use_sim_time': True}],
        remappings=[('odometry/filtered', 'odometry/local')]
    )

    # Only EKF (map -> base_link)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': True}],
        remappings=[('odometry/filtered', 'odometry/global')] # Prevents topic collision with local EKF
    )

    cov_filter = Node(
        package="localization",
        executable="cov_filter_node",
        output="screen",
        name="cov_filter",
        parameters=[{'use_sim_time': True}]
    )

    arducam = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='arducam_node',
        output='screen',
        parameters=[arducam_config],
        remappings=[
            ('image_raw', f'/front/camera/color/image_raw'),
            ('camera_info', f'/front/camera/color/camera_info')]

    )

    composable_nodes.extend([tagslam]) # sync_and_detect, 
   
    for name, config in cameras.items():

        # tf = ComposableNode(
        #     package='tf2_ros',
        #     name=f'tf_{name}',
        #     plugin='tf2_ros::StaticTransformBroadcasterNode',
        #     parameters=[{"x": config['x'],
        #                  "y": config['y'],
        #                  "z": config['z'],
        #                  "yaw": config['yaw'],
        #                  "pitch": '0',
        #                  "roll": '0',
        #                  "frame-id": 'rig',
        #                  "child-frame-id": f'{name}_cam_link',
        #                  "use_sim_time": False,
        #                  "use_approximate_sync": True}],
        #     # arguments=[
        #     #     '--x', config['x'], '--y', config['y'], '--z', config['z'],
        #     #     '--yaw', config['yaw'], '--pitch', '0', '--roll', '0',
        #     #     '--frame-id', 'rig', '--child-frame-id', f'{name}_cam_link',
        #     # ],
        #     extra_arguments=[{'use_intra_process_comms' : True}],
        # )

        tf_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'tf_{name}',
            arguments=[
                '--x', config['x'], 
                '--y', config['y'], 
                '--z', config['z'],
                '--yaw', config['yaw'], 
                '--pitch', '0', 
                '--roll', '0',
                '--frame-id', 'base_link', 
                '--child-frame-id', f'{name}_cam_link'
            ],
            parameters=[{'use_sim_time': True}],
        )
        tf_nodes.append(tf_node)

        realsense_node = ComposableNode(
            package='realsense2_camera',
            plugin='realsense2_camera::RealSenseNodeFactory',
            name='camera',
            namespace=name,
            parameters=[camera_config, {
                'serial_no': config['serial'],
                'camera_name': f'{name}_cam',
                'qos_image_topic': 'SENSOR_DATA'
            }],
           extra_arguments=[{'use_intra_process_comms' : True}],
        )

        # rectify_node = ComposableNode(
        #     package='isaac_ros_image_proc',
        #     plugin='nvidia::isaac_ros::image_proc::RectifyNode',
        #     name='rectify',
        #     namespace='',
        #     parameters=[{
        #         # 'output_width': camera_width,
        #         # 'output_height': camera_height,
        #     }],
        #     remappings=[
        #         ('image_raw', f'/{name}/camera/color/image_raw'),
        #         ('camera_info', f'/{name}/camera/color/camera_info'),
        #         ('image_rect', f'/{name}/camera/color/image_rect'),
        #         ('camera_info_rect', f'/{name}/camera/color/camera_info_rect')
        #     ],

        # )


        isaac_apriltag_node = ComposableNode(
            package='isaac_ros_apriltag',
            plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
            name=f'apriltag',
            namespace=name,
            parameters=[{'size': 0.145,
                     'max_tags': 3,
                     'tile_size': 4,
                     'tag_family': 'tag36h11',
                     'use_sim_time': True,
                     'backends': 'CUDA'}],
            # Remap the inputs to match the RealSense infra1 stream, 
            # and remap the output to match what TagSLAM expects.
            remappings=[
                ('image', f'/{name}/camera/color/image_raw'),
                ('camera_info', f'/{name}/camera/color/camera_info'),
                ('tag_detections', f'/{name}/detector/tags_nvidia')
            ]
        )

        composable_nodes.extend([
                                # realsense_node,
                                isaac_apriltag_node
                            ])


    container = ComposableNodeContainer(
        name='sentry_stack',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=composable_nodes,
        output='both',
        # arguments=['--ros-args', '--log-level', 'DEBUG']
    )

    translator_node = Node(
        package="localization",
        executable="tag_translator_node",
        output="screen",
        name="tag_translator",
        parameters=[{'use_sim_time': True}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return launch.LaunchDescription([container,
                                     uart_bridge,
                                     # arducam,
                                     detector,
                                     path_planning,
                                    #  ekf_local_node,
                                     ekf_node,
                                     cov_filter,
                                     translator_node,
                                     rviz_node,
                                     *tf_nodes])
