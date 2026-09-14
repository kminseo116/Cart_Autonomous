from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = dict(input_mode='realsense', model_path='yolo11n.pt', class_id='0',
                    camera_frame='camera_color_optical_frame', base_frame='body',
                    measure_duration='10.0', target_distance='1.0',
                    show_preview='true', confidence_threshold='0.5',
                    sensor_timeout='1.0',
                    odom_topic='/Odometry', nav_cmd_topic='/nav2/cmd_vel',
                    output_cmd_topic='/patient_approach/raw_cmd_vel', use_sim_time='false')
    value = lambda name, kind: ParameterValue(LaunchConfiguration(name), value_type=kind)
    shared = {'base_frame': LaunchConfiguration('base_frame'),
              'measure_duration': value('measure_duration', float),
              'target_distance': value('target_distance', float),
              'sensor_timeout': value('sensor_timeout', float),
              'use_sim_time': value('use_sim_time', bool)}
    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()],
        Node(package='patient_approach', executable='patient_depth_batch.py', output='screen',
             parameters=[shared, {'input_mode': LaunchConfiguration('input_mode'),
                                 'model_path': LaunchConfiguration('model_path'),
                                 'class_id': value('class_id', int),
                                 'show_preview': value('show_preview', bool),
                                 'confidence_threshold': value('confidence_threshold', float),
                                 'camera_frame': LaunchConfiguration('camera_frame')}]),
        Node(package='patient_approach', executable='batch_approach_manager.py', output='screen',
             parameters=[shared], remappings=[('/odom', LaunchConfiguration('odom_topic')),
                         ('/nav2/cmd_vel', LaunchConfiguration('nav_cmd_topic')),
                         ('/cmd_vel', LaunchConfiguration('output_cmd_topic'))]),
    ])
