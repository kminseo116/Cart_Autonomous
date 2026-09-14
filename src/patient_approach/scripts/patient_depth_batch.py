#!/usr/bin/env python3
"""User RealSense/YOLO ROI median logic, bounded ten-second measurement."""
import math
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool, Float32, String
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_point


class PatientDepthBatch(Node):
    def __init__(self):
        super().__init__('patient_depth_batch')
        self.mode = self.declare_parameter('input_mode', 'realsense').value
        self.duration = float(self.declare_parameter('measure_duration', 10.0).value)
        self.target = float(self.declare_parameter('target_distance', 1.0).value)
        self.frame = self.declare_parameter('camera_frame', 'camera_color_optical_frame').value
        self.base = self.declare_parameter('base_frame', 'base_link').value
        model_path = self.declare_parameter('model_path', 'yolo11n.pt').value
        self.class_id = int(self.declare_parameter('class_id', 0).value)
        self.show_preview = bool(self.declare_parameter('show_preview', True).value)
        self.confidence = float(self.declare_parameter('confidence_threshold', 0.5).value)
        if not all(math.isfinite(v) and v > 0 for v in (self.duration, self.target)):
            raise ValueError('duration and distance must be positive')
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.pub = self.create_publisher(PointStamped, '/patient/measurement', 10)
        self.depth_pub = self.create_publisher(Float32, '/patient_distance', 10)
        self.move_pub = self.create_publisher(Float32, '/patient_move_distance', 10)
        self.status = self.create_publisher(String, '/patient_measurement/state', 10)
        self.create_subscription(Bool, '/goal_complete', self.trigger, 10)
        self.create_subscription(PointStamped, '/patient/point', self.sample, qos_profile_sensor_data)
        self.active = self.high = False
        self.samples = []
        self.pipeline = None
        self.last_ns = -1
        self.previous_box = None
        self.last_issue = 'waiting for /goal_complete'
        if self.mode == 'realsense':
            if not Path(model_path).is_file():
                raise FileNotFoundError('Provide model_path pointing to an existing YOLO .pt file')
            import pyrealsense2 as rs
            import cv2
            from ultralytics import YOLO
            self.rs = rs
            self.cv2 = cv2
            self.model = YOLO(model_path)
            if self.class_id not in self.model.names:
                raise ValueError(
                    f'class_id {self.class_id} is not present in model; '
                    f'available IDs: {sorted(self.model.names)}')
            self.target_name = self.model.names[self.class_id]
            self.get_logger().info(
                f'YOLO target: class_id={self.class_id}, name={self.target_name}, '
                f'confidence>={self.confidence:.2f}')
            self.pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            self.align = rs.align(rs.stream.color)
            self.pipeline.start(config)
        elif self.mode != 'points':
            raise ValueError('input_mode must be realsense or points')
        self.create_timer(0.05, self.tick)

    def trigger(self, msg):
        rising = msg.data and not self.high
        self.high = msg.data
        if not msg.data:
            self.active = False
        elif rising and not self.active:
            self.active = True
            self.started = time.monotonic()
            self.started_ns = self.get_clock().now().nanoseconds
            self.samples, self.last_ns, self.previous_box = [], -1, None
            self.last_issue = 'detecting patient'
            self.status.publish(String(data='MEASURING'))
            self.get_logger().info('goal_complete: measuring for %.1f seconds' % self.duration)

    def sample(self, msg):
        if self.mode == 'points':
            self.accept(msg)

    def accept(self, msg):
        if not self.active or time.monotonic() - self.started >= self.duration:
            return
        ns = Time.from_msg(msg.header.stamp).nanoseconds
        if ns <= max(self.last_ns, self.started_ns) or not 0 <= (self.get_clock().now().nanoseconds-ns)/1e9 <= .3:
            return
        try:
            point = msg if msg.header.frame_id == self.base else do_transform_point(
                msg, self.tf.lookup_transform(self.base, msg.header.frame_id, Time.from_msg(msg.header.stamp)))
        except TransformException as exc:
            self.last_issue = f'TF unavailable: {self.base} <- {msg.header.frame_id}'
            return
        xyz = (point.point.x, point.point.y, point.point.z)
        if not all(math.isfinite(v) for v in xyz) or xyz[0] <= 0:
            self.last_issue = 'patient point is not in front of base_link'
            return
        self.last_ns = ns
        self.samples.append((xyz, msg.header.stamp, time.monotonic()))
        self.last_issue = f'valid samples: {len(self.samples)}'

    def show(self, image, message, color=(0, 255, 255)):
        if not self.show_preview:
            return
        try:
            self.cv2.putText(image, message, (15, 30), self.cv2.FONT_HERSHEY_SIMPLEX,
                             0.7, color, 2)
            self.cv2.imshow('Patient Depth Camera', image)
            self.cv2.waitKey(1)
        except Exception as exc:
            self.show_preview = False
            self.get_logger().warning(f'camera preview disabled: {exc}')

    def camera(self):
        frames = self.pipeline.poll_for_frames()
        if not frames:
            return
        frames = self.align.process(frames)
        depth, color = frames.get_depth_frame(), frames.get_color_frame()
        if not depth or not color:
            return
        stamp = self.get_clock().now().to_msg()
        image = np.asanyarray(color.get_data()).copy()
        if not self.active:
            self.show(image, 'Camera ON - waiting for goal_complete', (255, 255, 0))
            return
        # Keep low-confidence detections for the preview. The configured
        # threshold below still decides which target samples are accepted.
        boxes = self.model(image, verbose=False, conf=0.05)[0].boxes
        candidates = [b for b in boxes if int(b.cls[0]) == self.class_id
                      and float(b.conf[0]) >= self.confidence]
        detected = []
        for detection in boxes:
            cls = int(detection.cls[0])
            score = float(detection.conf[0])
            name = self.model.names.get(cls, str(cls))
            detected.append(f'{name}:{score:.2f}')
            bx = detection.xyxy[0].cpu().numpy().astype(int)
            accepted = cls == self.class_id and score >= self.confidence
            color = (0, 255, 0) if accepted else (160, 160, 160)
            thickness = 2 if accepted else 1
            self.cv2.rectangle(image, (bx[0], bx[1]), (bx[2], bx[3]), color, thickness)
            self.cv2.putText(image, f'{name} {score:.2f}',
                             (bx[0], max(20, bx[1] - 8)),
                             self.cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, thickness)
        # Ambiguous frames are skipped rather than switching between multiple people.
        if len(candidates) != 1:
            seen = ', '.join(detected[:4]) if detected else 'none'
            self.last_issue = (
                f'need 1 {self.target_name}>={self.confidence:.2f}; '
                f'targets={len(candidates)}; seen={seen}')
            self.show(image, self.last_issue, (0, 0, 255))
            return
        box = candidates[0].xyxy[0].cpu().numpy()
        if self.previous_box is not None:
            a = self.previous_box
            inter = max(0., min(a[2], box[2])-max(a[0], box[0])) * max(0., min(a[3], box[3])-max(a[1], box[1]))
            union = (a[2]-a[0])*(a[3]-a[1]) + (box[2]-box[0])*(box[3]-box[1]) - inter
            if union <= 0 or inter/union < .5:
                self.last_issue = 'patient tracking changed; frame skipped'
                self.show(image, self.last_issue, (0, 0, 255))
                return
        self.previous_box = box
        u, v = int((box[0]+box[2])/2), int((box[1]+box[3])/2)
        values = [depth.get_distance(x, y)
                  for y in range(max(0, v-10), min(depth.get_height(), v+10))
                  for x in range(max(0, u-10), min(depth.get_width(), u+10))]
        valid = [z for z in values if .2 < z < 5.0]
        if len(valid) < max(10, len(values)*.5):
            self.last_issue = f'not enough valid depth pixels: {len(valid)}/{len(values)}'
            self.cv2.rectangle(image, (max(0, u-10), max(0, v-10)),
                               (min(depth.get_width()-1, u+10), min(depth.get_height()-1, v+10)),
                               (0, 0, 255), 2)
            self.show(image, self.last_issue, (0, 0, 255))
            return
        z = float(np.median(valid))
        self.cv2.rectangle(image, (max(0, u-10), max(0, v-10)),
                           (min(depth.get_width()-1, u+10), min(depth.get_height()-1, v+10)),
                           (255, 0, 0), 2)
        self.cv2.circle(image, (u, v), 4, (0, 255, 255), -1)
        intrinsics = depth.profile.as_video_stream_profile().intrinsics
        xyz = self.rs.rs2_deproject_pixel_to_point(intrinsics, [u, v], z)
        msg = PointStamped()
        msg.header.stamp, msg.header.frame_id = stamp, self.frame
        msg.point.x, msg.point.y, msg.point.z = map(float, xyz)
        self.accept(msg)
        remaining = max(0.0, self.duration - (time.monotonic() - self.started))
        self.show(image, f'Depth {z:.2f}m | samples {len(self.samples)} | {remaining:.1f}s',
                  (0, 255, 0))

    def tick(self):
        if self.pipeline:
            self.camera()
        if not self.active:
            return
        if time.monotonic() - self.started >= self.duration:
            self.active = False
            if len(self.samples) < 10 or time.monotonic()-self.samples[-1][2] > .3:
                self.status.publish(String(data='FAILED'))
                self.get_logger().error(
                    f'measurement failed: samples={len(self.samples)}, reason={self.last_issue}')
                return
            xyz = np.array([s[0] for s in self.samples])
            median = np.median(xyz, axis=0)
            if np.percentile(np.linalg.norm(xyz-median, axis=1), 90) > .15:
                self.status.publish(String(data='FAILED'))
                self.get_logger().error('measurement failed: patient position was not stable')
                return
            msg = PointStamped()
            msg.header.frame_id = self.base
            msg.header.stamp = self.samples[-1][1]
            msg.point.x, msg.point.y, msg.point.z = map(float, median)
            distance = math.hypot(median[0], median[1])
            self.depth_pub.publish(Float32(data=distance))
            self.move_pub.publish(Float32(data=max(0., distance-self.target)))
            self.pub.publish(msg)
            self.status.publish(String(data='DONE'))
            self.get_logger().info(f'measured={distance:.3f}m move={max(0., distance-self.target):.3f}m')


def main():
    rclpy.init()
    node = PatientDepthBatch()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.pipeline:
            node.pipeline.stop()
        if node.mode == 'realsense' and node.show_preview:
            node.cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
