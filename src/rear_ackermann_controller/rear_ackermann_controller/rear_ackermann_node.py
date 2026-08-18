"""Convert a Twist command to fixed-wheel differential-drive commands."""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

from .config import load_vehicle_params
from .rear_steer_kinematics import compute_rear_steer_command


class RearAckermannNode(Node):
    """Subscribe to ``/alignment_cmd`` and publish two wheel angular speeds."""

    def __init__(self) -> None:
        super().__init__("rear_ackermann_node")
        self.params = load_vehicle_params()
        self.left_speed_publisher = self.create_publisher(Float64, "/rear_left_wheel_speed_cmd", 10)
        self.right_speed_publisher = self.create_publisher(Float64, "/rear_right_wheel_speed_cmd", 10)
        self.alignment_subscription = self.create_subscription(Twist, "/alignment_cmd", self.on_alignment_twist, 10)
        self.manual_subscription = self.create_subscription(Twist, "/cmd_vel", self.on_manual_twist, 10)
        self.manual_until_s = 0.0
        self.get_logger().info("fixed-wheel differential-drive node started; /cmd_vel has manual priority")

    def on_alignment_twist(self, message: Twist) -> None:
        """Use alignment only when no recent manual ``/cmd_vel`` is present."""

        if time.monotonic() < self.manual_until_s:
            return
        self.apply_twist(message)

    def on_manual_twist(self, message: Twist) -> None:
        """Give the standard ROS manual command a short, renewable priority."""

        self.manual_until_s = time.monotonic() + 0.5
        self.apply_twist(message)

    def apply_twist(self, message: Twist) -> None:
        """Apply differential-drive kinematics and publish rad/s joint speeds."""

        command = compute_rear_steer_command(
            message.linear.x, message.angular.z, self.params
        )
        self.left_speed_publisher.publish(Float64(data=command.left.wheel_speed_mps / self.params.wheel_radius_m))
        self.right_speed_publisher.publish(Float64(data=command.right.wheel_speed_mps / self.params.wheel_radius_m))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RearAckermannNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
