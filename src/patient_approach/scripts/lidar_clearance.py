#!/usr/bin/env python3
"""Publish a fail-safe near-field clearance signal from a body-frame point cloud."""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool


class LidarClearance(Node):
    def __init__(self):
        super().__init__('lidar_clearance')
        self.declare_parameter('cloud_topic', '/cloud_registered_body')
        self.declare_parameter('expected_frame', 'body')
        self.declare_parameter('x_min', 0.74)
        self.declare_parameter('x_max', 0.92)
        self.declare_parameter('half_width', 0.36)
        self.declare_parameter('z_min', -0.25)
        self.declare_parameter('z_max', 1.50)
        self.declare_parameter('blocked_point_count', 4)
        self.declare_parameter('cloud_timeout_sec', 0.5)

        self.expected_frame = str(self.get_parameter('expected_frame').value)
        self.x_min = float(self.get_parameter('x_min').value)
        self.x_max = float(self.get_parameter('x_max').value)
        self.half_width = float(self.get_parameter('half_width').value)
        self.z_min = float(self.get_parameter('z_min').value)
        self.z_max = float(self.get_parameter('z_max').value)
        self.blocked_point_count = int(self.get_parameter('blocked_point_count').value)
        self.timeout_ns = int(float(self.get_parameter('cloud_timeout_sec').value) * 1e9)
        self.last_cloud_ns = None
        self.last_clear = False

        topic = str(self.get_parameter('cloud_topic').value)
        self.publisher = self.create_publisher(Bool, '/patient_approach/path_clear', 10)
        self.create_subscription(PointCloud2, topic, self.on_cloud, 10)
        self.create_timer(0.1, self.publish_state)
        self.get_logger().info(
            f'checking {topic} in {self.expected_frame}: '
            f'x=[{self.x_min:.2f},{self.x_max:.2f}], |y|<{self.half_width:.2f}')

    def on_cloud(self, msg: PointCloud2):
        self.last_cloud_ns = self.get_clock().now().nanoseconds
        if msg.header.frame_id.lstrip('/') != self.expected_frame.lstrip('/'):
            self.last_clear = False
            self.get_logger().error(
                f'cloud frame is {msg.header.frame_id!r}, expected {self.expected_frame!r}',
                throttle_duration_sec=2.0)
            return

        blocked = 0
        try:
            for x, y, z in point_cloud2.read_points(
                    msg, field_names=('x', 'y', 'z'), skip_nans=True):
                if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                    continue
                if (self.x_min <= x <= self.x_max and abs(y) <= self.half_width
                        and self.z_min <= z <= self.z_max):
                    blocked += 1
                    if blocked >= self.blocked_point_count:
                        break
            self.last_clear = blocked < self.blocked_point_count
        except (AssertionError, KeyError, ValueError) as exc:
            self.last_clear = False
            self.get_logger().error(f'cannot read point cloud: {exc}', throttle_duration_sec=2.0)

    def publish_state(self):
        now_ns = self.get_clock().now().nanoseconds
        fresh = self.last_cloud_ns is not None and now_ns - self.last_cloud_ns <= self.timeout_ns
        self.publisher.publish(Bool(data=bool(fresh and self.last_clear)))


def main(args=None):
    rclpy.init(args=args)
    node = LidarClearance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
