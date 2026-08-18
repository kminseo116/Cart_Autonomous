"""실제 ZLAC8015D 하드웨어 전용 구동 launch; Gazebo bridge와 함께 실행하지 않는다."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    driver_config = get_package_share_directory("zlac8015d_driver") + "/config/zlac8015d.yaml"
    return LaunchDescription([
        # Twist 우선순위와 wheel rad/s 변환은 기존 노드를 그대로 재사용한다.
        Node(package="rear_ackermann_controller", executable="rear_ackermann_node", output="screen"),
        # 이 노드는 wheel rad/s만 받아 RS485 Modbus RPM 명령으로 변환한다.
        Node(package="zlac8015d_driver", executable="zlac8015d_driver_node",
             parameters=[driver_config], output="screen"),
    ])
