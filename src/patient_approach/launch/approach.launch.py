from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = {'depth_topic': '/camera/aligned_depth/image_raw',
            'camera_info_topic': '/camera/aligned_depth/camera_info',
            'roi_topic': '/patient/roi', 'odom_topic': '/odom',
            'nav_cmd_topic': '/nav2/cmd_vel',
            'output_cmd_topic': '/patient_approach/cmd_vel',
            'target_distance': '1.0', 'base_frame': 'base_link',
            'use_sim_time': 'false'}
    sim = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value=v) for k, v in args.items()],
        Node(package='patient_approach', executable='depth_patient_point.py',
             output='screen', parameters=[{'use_sim_time': sim}], remappings=[
                 ('/camera/aligned_depth/image_raw', LaunchConfiguration('depth_topic')),
                 ('/camera/aligned_depth/camera_info', LaunchConfiguration('camera_info_topic')),
                 ('/patient/roi', LaunchConfiguration('roi_topic'))]),
        Node(package='patient_approach', executable='approach_manager.py', output='screen',
             parameters=[{'use_sim_time': sim,
                          'base_frame': LaunchConfiguration('base_frame'),
                          'target_distance': ParameterValue(LaunchConfiguration('target_distance'), value_type=float)}],
             remappings=[('/odom', LaunchConfiguration('odom_topic')),
                         ('/nav2/cmd_vel', LaunchConfiguration('nav_cmd_topic')),
                         ('/cmd_vel', LaunchConfiguration('output_cmd_topic'))]),
    ])
