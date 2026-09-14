#!/usr/bin/env python3
"""ROS integration simulation; mock Nav2, synthetic depth, real wheel converter.

Run only in an isolated ROS domain. No serial driver or physical sensor starts.
"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

import numpy as np
import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import PolygonStamped, Point32, PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Float32, Float64, String
from std_srvs.srv import Trigger
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class World(Node):
    def __init__(self, batch=False):
        super().__init__('patient_approach_test_world')
        self.batch = batch
        self.measure_at = None
        self.measure_x = None
        self.measure_drift = 0.0
        self.measured_values = []
        self.create_subscription(Float32, '/patient_move_distance', lambda m: self.measured_values.append(m.data), 10)
        self.repeat_trigger = self.create_publisher(Bool, '/goal_complete', 10)
        self.scenario = 'idle'
        self.x = self.y = self.yaw = 0.0
        self.left = self.right = 0.0
        self.state = 'IDLE'
        self.states = []
        self.approach_at = None
        self.fault_at = None
        self.rows = []
        self.start = time.monotonic()
        self.active = False
        self.bridge = CvBridge()
        config = json.loads((Path(get_package_share_directory('rear_ackermann_controller')) / 'config/params_setting.json').read_text())
        self.radius, self.track = config['wheel_radius_m'], config['rear_track_m']
        self.target = (1.6, 0.20)
        self.depth = self.create_publisher(Image, '/camera/aligned_depth/image_raw', 10)
        self.info = self.create_publisher(CameraInfo, '/camera/aligned_depth/camera_info', 10)
        self.roi = self.create_publisher(PolygonStamped, '/patient/roi', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.clear_pub = self.create_publisher(Bool, '/patient_approach/path_clear', 10)
        self.nav_pub = self.create_publisher(Twist, '/nav2/cmd_vel', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/mission_goal', 10)
        self.create_subscription(Float64, '/sim/left_wheel', self.on_left, 10)
        self.create_subscription(Float64, '/sim/right_wheel', self.on_right, 10)
        self.create_subscription(String, '/patient_approach/state', self.on_state, 10)
        self.stop_client = self.create_client(Trigger, '/patient_approach/stop')
        self.server = ActionServer(self, NavigateToPose, '/navigate_to_pose',
                                   execute_callback=self.execute,
                                   goal_callback=lambda _: GoalResponse.REJECT if self.scenario == 'nav_rejected' else GoalResponse.ACCEPT,
                                   cancel_callback=lambda _: CancelResponse.ACCEPT,
                                   callback_group=ReentrantCallbackGroup())
        self.tf = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = 'base_link', 'sim_camera_optical_frame'
        t.transform.rotation.x, t.transform.rotation.y = -0.5, 0.5
        t.transform.rotation.z, t.transform.rotation.w = -0.5, 0.5
        self.tf.sendTransform(t)
        self.last_tick = time.monotonic()
        self.create_timer(0.05, self.tick)

    def on_left(self, msg):
        self.left = msg.data

    def on_right(self, msg):
        self.right = msg.data

    def on_state(self, msg):
        self.state = msg.data
        if self.active and (not self.states or self.states[-1] != msg.data):
            self.states.append(msg.data)
        if self.active and msg.data == ('MOVING' if self.batch else 'APPROACHING') and self.approach_at is None:
            self.approach_at = time.monotonic()

        if self.active and msg.data == 'MEASURING' and self.measure_at is None:
            self.measure_at, self.measure_x = time.monotonic(), self.x

    def execute(self, handle):
        deadline = time.monotonic() + 10
        while rclpy.ok() and time.monotonic() < deadline:
            if handle.is_cancel_requested:
                self.nav_pub.publish(Twist())
                handle.canceled()
                return NavigateToPose.Result()
            if self.x >= handle.request.pose.pose.position.x:
                self.nav_pub.publish(Twist())
                handle.succeed()
                return NavigateToPose.Result()
            cmd = Twist()
            cmd.linear.x = 0.2
            self.nav_pub.publish(cmd)
            time.sleep(0.05)
        handle.abort()
        return NavigateToPose.Result()

    def tick(self):
        now = time.monotonic()
        dt, self.last_tick = min(now - self.last_tick, 0.1), now
        v = (self.left + self.right) * self.radius / 2
        w = (self.right - self.left) * self.radius / self.track
        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt
        self.yaw += w * dt
        fault = self.approach_at is not None and now - self.approach_at >= 0.7
        if fault and self.fault_at is None:
            self.fault_at = now
        stamp = self.get_clock().now().to_msg()
        if not (fault and self.scenario == 'odom_loss'):
            odom = Odometry()
            odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, 'odom', 'base_link'
            odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
            odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(self.yaw/2), math.cos(self.yaw/2)
            odom.twist.twist.linear.x, odom.twist.twist.angular.z = v, w
            self.odom_pub.publish(odom)
        if not (fault and self.scenario == 'clear_timeout'):
            self.clear_pub.publish(Bool(data=not (fault and self.scenario == 'obstacle')))
        # Deliberately continue Nav2 zero messages after success: mux must ignore them.
        if self.state != 'NAVIGATING':
            self.nav_pub.publish(Twist())
        if self.batch and self.state == 'MEASURING':
            self.repeat_trigger.publish(Bool(data=True))
            if self.measure_x is not None:
                self.measure_drift = max(self.measure_drift, abs(self.x-self.measure_x))
        if self.scenario != 'no_detection' and not (fault and self.scenario in ('depth_loss', 'nominal_depth_off')):
            dx, dy = self.target[0] - self.x, self.target[1] - self.y
            forward = math.cos(self.yaw)*dx + math.sin(self.yaw)*dy
            lateral = -math.sin(self.yaw)*dx + math.cos(self.yaw)*dy
            if fault and self.scenario == 'too_close':
                forward, lateral = 0.8, 0.0
            if forward > 0:
                # 160x120 rectified camera, optical z forward, optical x right.
                u = int(round(80 - 120*lateral/forward))
                image = np.full((120, 160), np.nan, dtype=np.float32)
                x0, x1 = max(0, u-5), min(160, u+6)
                if x1 > x0:
                    image[55:66, x0:x1] = forward
                encoding = '32FC1'
                if self.scenario == 'nominal_mm':
                    image = np.nan_to_num(image * 1000).astype(np.uint16)
                    encoding = '16UC1'
                if fault and self.scenario == 'invalid_depth':
                    image[:] = np.nan
                depth = self.bridge.cv2_to_imgmsg(image, encoding=encoding)
                depth.header.stamp, depth.header.frame_id = stamp, 'sim_camera_optical_frame'
                info = CameraInfo()
                info.header, info.width, info.height = depth.header, 160, 120
                info.p = [120., 0., 80., 0., 0., 120., 60., 0., 0., 0., 1., 0.]
                roi = PolygonStamped()
                roi.header = depth.header
                roi.polygon.points = [Point32(x=float(x0), y=55.), Point32(x=float(x1), y=66.)]
                self.info.publish(info)
                self.roi.publish(roi)
                self.depth.publish(depth)
        if self.active:
            self.rows.append((self.scenario, now-self.start, self.state, self.x, self.y,
                              self.yaw, math.hypot(self.target[0]-self.x, self.target[1]-self.y), v, w))

    def run_case(self, scenario):
        self.active = False
        time.sleep(0.5)
        self.x = self.y = self.yaw = 0.0
        self.scenario = scenario
        self.states = []
        self.approach_at = self.fault_at = self.measure_at = self.measure_x = None
        self.measure_drift = 0.0
        self.measured_values = []
        self.start = time.monotonic()
        self.active = True
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = 0.25
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        expected = 'DONE' if scenario.startswith('nominal') else 'FAILED'
        deadline = time.monotonic() + 35
        while time.monotonic() < deadline:
            time.sleep(0.05)
            if (('NAVIGATING' in self.states or
                 (scenario == 'nav_rejected' and time.monotonic() - self.start > 0.5))
                    and self.state in ('DONE', 'FAILED')):
                break
        time.sleep(0.4)
        distance = math.hypot(self.target[0]-self.x, self.target[1]-self.y)
        stopped = abs(self.left) < 1e-6 and abs(self.right) < 1e-6
        passed = self.state == expected and stopped
        if expected == 'DONE':
            passed &= 0.95 <= distance <= 1.055
            passed &= all(s in self.states for s in (('SETTLING', 'MEASURING', 'MOVING') if self.batch else ('SETTLING', 'WAIT_PATIENT', 'APPROACHING')))
        elif scenario not in ('nav_rejected', 'no_detection'):
            passed &= self.approach_at is not None
        latency = None
        if self.fault_at is not None and not scenario.startswith('nominal'):
            rows = [r for r in self.rows if r[0] == scenario and r[1] >= self.fault_at-self.start and abs(r[7]) < 1e-6 and abs(r[8]) < 1e-6]
            if rows:
                latency = rows[0][1] - (self.fault_at-self.start)
            passed &= latency is not None and latency <= 0.55
        measurement_seconds = None
        if self.batch and expected == 'DONE':
            measurement_seconds = self.approach_at-self.measure_at
            passed &= measurement_seconds >= 9.8 and len(self.measured_values) == 1 and self.measure_drift < .001
        if scenario == 'no_detection':
            passed &= self.approach_at is None and not self.measured_values
        self.active = False
        return dict(scenario=scenario, passed=bool(passed), states=self.states,
                    final_distance_m=round(distance, 4), wheels_stopped=stopped,
                    measurement_seconds=measurement_seconds, move_distance_messages=self.measured_values,
                    fault_stop_latency_s=None if latency is None else round(latency, 4))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--batch', action='store_true')
    parser.add_argument('--output-dir', default='/tmp/patient_approach_sim')
    parser.add_argument('--scenarios', nargs='+')
    args = parser.parse_args()
    if args.scenarios is None:
        args.scenarios = (['nominal', 'nominal_mm', 'nominal_depth_off', 'obstacle', 'odom_loss', 'no_detection', 'nav_rejected'] if args.batch else
                          ['nominal', 'nominal_mm', 'depth_loss', 'invalid_depth', 'obstacle', 'clear_timeout', 'odom_loss', 'too_close', 'nav_rejected'])
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or int(os.environ.get('ROS_DOMAIN_ID', '0')) == 0:
        raise SystemExit('Set ROS_LOCALHOST_ONLY=1 and a dedicated nonzero ROS_DOMAIN_ID')
    out = Path(args.output_dir)
    out.mkdir(parents=True, exist_ok=True)
    children, logs = [], []
    rclpy.init()
    world = World(batch=args.batch)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(world)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        node_specs = [
            ('patient_approach', 'batch_approach_manager.py' if args.batch else 'approach_manager.py', ['-r', '/cmd_vel:=/sim/cmd_vel']),
            ('patient_approach', 'depth_patient_point.py', []),
            ('rear_ackermann_controller', 'rear_ackermann_node', [
                '-r', '/cmd_vel:=/sim/cmd_vel', '-r', '/alignment_cmd:=/sim/unused_alignment',
                '-r', '/rear_left_wheel_speed_cmd:=/sim/left_wheel',
                '-r', '/rear_right_wheel_speed_cmd:=/sim/right_wheel'])]
        if args.batch:
            node_specs.append(('patient_approach', 'patient_depth_batch.py', ['-p', 'input_mode:=points']))
        for package, executable, extra in node_specs:
            path = Path(get_package_prefix(package)) / 'lib' / package / executable
            log = (out / f'{executable}.log').open('w')
            logs.append(log)
            command = (['/usr/bin/python3', str(path)] if executable.endswith('.py') else [str(path)])
            children.append(subprocess.Popen(command + ['--ros-args'] + extra,
                                             stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
        time.sleep(2.0)
        if any(p.poll() is not None for p in children):
            raise RuntimeError(f'Node startup failed; inspect {out}')
        results = []
        for scenario in args.scenarios:
            result = world.run_case(scenario)
            results.append(result)
            print(json.dumps(result), flush=True)
        (out / 'results.json').write_text(json.dumps(results, indent=2) + '\n')
        with (out / 'trajectory.csv').open('w') as stream:
            writer = csv.writer(stream)
            writer.writerow(['scenario', 'time_s', 'state', 'x_m', 'y_m', 'yaw_rad', 'distance_m', 'v_mps', 'w_radps'])
            writer.writerows(world.rows)
        if not all(r['passed'] for r in results):
            raise SystemExit(1)
    finally:
        for process in children:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in children:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        for log in logs:
            log.close()
        executor.shutdown()
        thread.join(timeout=2)
        world.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
