"""실제 ZLAC8015D 하드웨어 전용 구동 launch; Gazebo bridge와 함께 실행하지 않는다."""

import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    enable_wheel_odometry = LaunchConfiguration("enable_wheel_odometry")
    enable_imu_degrees = LaunchConfiguration("enable_imu_degrees")
    driver_config = get_package_share_directory("zlac8015d_driver") + "/config/zlac8015d.yaml"
    params_path = Path.cwd() / "src" / "params_setting.json"
    if not params_path.exists():
        params_path = Path(get_package_share_directory("rear_ackermann_controller")) / "config" / "params_setting.json"
    cart_params = json.loads(params_path.read_text(encoding="utf-8"))
    # 차체 기하와 IMU 입력은 params_setting.json을 단일 기준으로 사용한다.
    odometry_parameters = {
        "wheel_radius_m": cart_params["wheel_radius_m"],
        "wheel_track_m": cart_params["rear_track_m"],
        "odom_frame": cart_params["odom_frame"],
        "base_frame": cart_params["base_frame"],
        "use_imu_yaw": cart_params["use_imu_yaw"],
        "imu_topic": cart_params["imu_topic"],
        "imu_timeout_sec": cart_params["imu_timeout_sec"],
        "imu_roll_rad": cart_params["base_to_imu_roll_rad"],
        "imu_pitch_rad": cart_params["base_to_imu_pitch_rad"],
    }
    return LaunchDescription([
        DeclareLaunchArgument("enable_wheel_odometry", default_value="false"),
        DeclareLaunchArgument("enable_imu_degrees", default_value="false"),
        # Twist 우선순위와 wheel rad/s 변환은 기존 노드를 그대로 재사용한다.
        Node(package="rear_ackermann_controller", executable="rear_ackermann_node", output="screen"),
        # 이 노드는 wheel rad/s만 받아 RS485 Modbus RPM 명령으로 변환한다.
        Node(package="zlac8015d_driver", executable="zlac8015d_driver_node",
             parameters=[driver_config], output="screen"),
        # 바퀴 encoder 이동량과 MID-360 gyro yaw로 실제 차체의 /odom을 만든다.
        Node(package="zlac8015d_driver", executable="wheel_odometry_node",
             parameters=[odometry_parameters], condition=IfCondition(enable_wheel_odometry),
             output="screen"),
        # MID-360 gyro z 적분 yaw를 사람이 확인하기 쉬운 degree로 변환한다.
        Node(package="zlac8015d_driver", executable="imu_degrees_node",
             parameters=[{"imu_topic": cart_params["imu_topic"],
                          "max_integration_dt_sec": cart_params["imu_timeout_sec"]}],
             condition=IfCondition(enable_imu_degrees),
             output="screen"),
    ])
