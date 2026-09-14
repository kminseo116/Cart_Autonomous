"""MID-360 + FAST-LIO + Nav2 + D435 + ZLAC hardware stack using body as base."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue


def include(package, relative_path, arguments=None):
    source = os.path.join(get_package_share_directory(package), relative_path)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(source),
        launch_arguments=(arguments or {}).items())


def generate_launch_description():
    patient_share = get_package_share_directory('patient_approach')
    workspace = '/home/reboot/cpr_auto_ws'
    model_path = LaunchConfiguration('model_path')
    class_id = LaunchConfiguration('class_id')
    target_distance = LaunchConfiguration('target_distance')
    pcd_map = LaunchConfiguration('pcd_map')
    map_yaml = LaunchConfiguration('map_yaml')
    show_preview = LaunchConfiguration('show_preview')
    set_initial_pose = LaunchConfiguration('set_initial_pose')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_z = LaunchConfiguration('initial_pose_z')
    initial_pose_qz = LaunchConfiguration('initial_pose_qz')
    initial_pose_qw = LaunchConfiguration('initial_pose_qw')

    nav_arguments = {
        'global_frame_id': 'map',
        'odom_frame_id': 'camera_init',
        'base_frame_id': 'body',
        'odom_topic': '/Odometry',
        'cloud_topic': '/cloud_registered_body',
        'pointcloud_topic': '/cloud_registered_body',
        'lidar_frame_id': 'body',
        'publish_lidar_tf': 'false',
        'publish_imu_tf': 'false',
        'pcd_map_path': pcd_map,
        'map_yaml': map_yaml,
        'set_initial_pose': set_initial_pose,
        'initial_pose_x': initial_pose_x,
        'initial_pose_y': initial_pose_y,
        'initial_pose_z': initial_pose_z,
        'initial_pose_qz': initial_pose_qz,
        'initial_pose_qw': initial_pose_qw,
        'launch_nav2': 'true',
        'autostart': 'true',
    }
    patient_arguments = {
        'input_mode': 'realsense',
        'model_path': model_path,
        'class_id': class_id,
        'confidence_threshold': '0.5',
        'show_preview': show_preview,
        'target_distance': target_distance,
        'base_frame': 'body',
        'odom_topic': '/Odometry',
        'nav_cmd_topic': '/nav2/cmd_vel',
        'output_cmd_topic': '/patient_approach/raw_cmd_vel',
    }

    return LaunchDescription([
        DeclareLaunchArgument('model_path', default_value='/home/reboot/yolo11n.pt'),
        DeclareLaunchArgument('class_id', default_value='41'),
        DeclareLaunchArgument('target_distance', default_value='0.5'),
        DeclareLaunchArgument('pcd_map', default_value=workspace + '/map/fastlio_map.pcd'),
        DeclareLaunchArgument(
            'map_yaml', default_value=workspace + '/map/fastlio_map_2d_clean_p2/fastlio_map_2d_clean_p2.yaml'),
        DeclareLaunchArgument('show_preview', default_value='true'),
        DeclareLaunchArgument('set_initial_pose', default_value='false'),
        DeclareLaunchArgument('initial_pose_x', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_y', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_z', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_qz', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_qw', default_value='1.0'),

        include('livox_ros_driver2', 'launch_ROS2/msg_MID360_launch.py'),
        include('fast_lio', 'launch/mapping.launch.py', {'config_file': 'mid360.yaml', 'rviz': 'false'}),
        include('zlac8015d_driver', 'launch/hardware_drive.launch.py',
                {'enable_wheel_odometry': 'false', 'enable_imu_degrees': 'false'}),

        # All Nav2 velocity output enters the patient manager instead of reaching the motor directly.
        GroupAction([
            SetRemap(src='/cmd_vel', dst='/nav2/cmd_vel'),
            include('lidar_localization_ros2', 'launch/nav2_navigation.launch.py', nav_arguments),
        ]),
        include('patient_approach', 'launch/batch_approach.launch.py', patient_arguments),

        Node(package='patient_approach', executable='lidar_clearance.py', output='screen'),
        Node(package='nav2_collision_monitor', executable='collision_monitor',
             name='collision_monitor', output='screen',
             parameters=[os.path.join(patient_share, 'config', 'body_collision_monitor.yaml')]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='collision_monitor_lifecycle_manager', output='screen',
             parameters=[os.path.join(patient_share, 'config', 'body_collision_monitor.yaml')]),

        # Existing measured camera pose, now attached to the actual FAST-LIO base frame.
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['-0.5', '0.0', '0.6', '0.0', '-0.698132', '0.0', 'body', 'camera_link']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0', '0', '0', '-1.570796', '0', '-1.570796',
                        'camera_link', 'camera_color_optical_frame']),
    ])
