#!/usr/bin/env python3
"""Rectified aligned depth + timestamped torso ROI -> optical-frame point.

PolygonStamped carries a rectangular ROI (min/max pixel coordinates). It is
an adapter contract, not a human detector. Input ROI must belong to one tracked
patient and exclude background as much as possible.
"""
import math
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from cv_bridge import CvBridge
from geometry_msgs.msg import PolygonStamped, PointStamped
from sensor_msgs.msg import CameraInfo, Image


class DepthPatientPoint(Node):
    def __init__(self):
        super().__init__('depth_patient_point')
        self.bridge = CvBridge()
        self.info = None
        self.depths, self.rois = deque(maxlen=10), deque(maxlen=10)
        self.last_stamp = -1
        self.pub = self.create_publisher(PointStamped, '/patient/point', 10)
        self.create_subscription(CameraInfo, '/camera/aligned_depth/camera_info',
                                 self.camera_info, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/aligned_depth/image_raw',
                                 self.depth, qos_profile_sensor_data)
        self.create_subscription(PolygonStamped, '/patient/roi', self.roi,
                                 qos_profile_sensor_data)

    def camera_info(self, msg):
        self.info = msg

    def depth(self, msg):
        self.depths.append(msg)
        self.process()

    def roi(self, msg):
        self.rois.append(msg)
        self.process()

    def process(self):
        if self.info is None or not self.depths or not self.rois:
            return
        roi = self.rois[-1]
        roi_ns = Time.from_msg(roi.header.stamp).nanoseconds
        depth = min(self.depths, key=lambda d: abs(Time.from_msg(d.header.stamp).nanoseconds - roi_ns))
        ns = Time.from_msg(depth.header.stamp).nanoseconds
        if ns <= self.last_stamp or abs(ns - roi_ns) > 50_000_000:
            return
        age = (self.get_clock().now().nanoseconds - ns) / 1e9
        if not 0 <= age <= 0.3:
            return
        if not (depth.header.frame_id == roi.header.frame_id == self.info.header.frame_id):
            return
        if (depth.width, depth.height) != (self.info.width, self.info.height):
            return
        if len(roi.polygon.points) < 2:
            return
        xy = np.array([(p.x, p.y) for p in roi.polygon.points])
        if not np.isfinite(xy).all():
            return
        x0, y0 = np.floor(xy.min(axis=0)).astype(int)
        x1, y1 = np.ceil(xy.max(axis=0)).astype(int)
        x0, x1 = max(0, x0), min(depth.width, x1)
        y0, y1 = max(0, y0), min(depth.height, y1)
        if x1 <= x0 or y1 <= y0 or depth.encoding not in ('16UC1', '32FC1'):
            return
        fx, fy, cx, cy = self.info.p[0], self.info.p[5], self.info.p[2], self.info.p[6]
        if not all(math.isfinite(v) for v in (fx, fy, cx, cy)) or fx <= 0 or fy <= 0:
            return
        try:
            z = self.bridge.imgmsg_to_cv2(depth, desired_encoding='passthrough')[y0:y1, x0:x1].astype(float)
        except Exception as exc:
            self.get_logger().warning(f'Depth decode failed: {exc}')
            return
        if depth.encoding == '16UC1':
            z *= 0.001  # ROS depth convention: uint16 millimetres, float32 metres
        valid = np.isfinite(z) & (z > 0.1) & (z < 10.0)
        if np.count_nonzero(valid) < max(10, z.size * 0.5):
            return
        median = np.median(z[valid])
        valid &= np.abs(z - median) < 0.2
        if np.count_nonzero(valid) < max(10, z.size * 0.5):
            return
        v, u = np.nonzero(valid)
        zs = z[valid]
        msg = PointStamped()
        msg.header = depth.header
        msg.point.x = float(np.median((u + x0 - cx) * zs / fx))
        msg.point.y = float(np.median((v + y0 - cy) * zs / fy))
        msg.point.z = float(np.median(zs))
        self.last_stamp = ns
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = DepthPatientPoint()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
