#!/usr/bin/env python3
"""Single velocity owner: Nav2 -> measured stop -> fresh depth approach."""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_point


class ApproachManager(Node):
    def __init__(self):
        super().__init__('patient_approach_manager')
        defaults = dict(target_distance=1.0, max_speed=0.10,
                        max_turn_rate=0.10, sensor_timeout=0.3,
                        phase_timeout=90.0, base_frame='base_link')
        self.p = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        for key in defaults:
            if key != 'base_frame' and (not math.isfinite(self.p[key]) or self.p[key] <= 0):
                raise ValueError(f'{key} must be finite and positive')
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.output = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status = self.create_publisher(String, '/patient_approach/state', 10)
        self.create_subscription(PoseStamped, '/mission_goal', self.goal, 10)
        self.create_subscription(Twist, '/nav2/cmd_vel', self.nav_velocity, 10)
        self.create_subscription(PointStamped, '/patient/point', self.point, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.odom, qos_profile_sensor_data)
        self.create_subscription(Bool, '/patient_approach/path_clear', self.clear, 10)
        self.create_service(Trigger, '/patient_approach/stop', self.stop_service)
        self.state = 'IDLE'
        self.since = time.monotonic()
        self.goal_handle = None
        self.pending = False
        self.nav_cmd = Twist()
        self.nav_at = -math.inf
        self.odom_at = -math.inf
        self.stationary_since = None
        self.clear_at = -math.inf
        self.is_clear = False
        self.sample = None
        self.last_stamp = -1
        self.samples = 0
        self.stable_since = None
        self.create_timer(0.05, self.tick)

    def transition(self, state):
        self.state, self.since = state, time.monotonic()
        self.get_logger().info(f'state -> {state}')

    def halt(self, reason):
        self.output.publish(Twist())
        self.get_logger().warning(reason)
        self.transition('FAILED')
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()

    def stop_service(self, request, response):
        self.halt('Stop requested')
        response.success, response.message = True, 'Stopped; active goal cancellation requested'
        return response

    def goal(self, msg):
        if self.state not in ('IDLE', 'DONE', 'FAILED') or self.pending or self.goal_handle:
            self.get_logger().warning('Mission busy, goal ignored')
            return
        if not self.nav.server_is_ready():
            self.get_logger().error('NavigateToPose unavailable')
            return
        self.sample, self.samples, self.last_stamp = None, 0, -1
        self.stable_since, self.stationary_since = None, None
        self.nav_at = -math.inf
        self.transition('NAVIGATING')
        self.pending = True
        goal = NavigateToPose.Goal()
        goal.pose = msg
        self.nav.send_goal_async(goal).add_done_callback(self.accepted)

    def accepted(self, future):
        self.pending = False
        try:
            handle = future.result()
            if handle is None or not handle.accepted:
                self.halt('Nav2 rejected goal')
                return
            self.goal_handle = handle
            handle.get_result_async().add_done_callback(self.finished)
            if self.state != 'NAVIGATING':
                handle.cancel_goal_async()
        except Exception as exc:
            self.halt(f'Nav2 acceptance error: {exc}')

    def finished(self, future):
        self.goal_handle = None
        if self.state != 'NAVIGATING':
            return
        try:
            if future.result().status != GoalStatus.STATUS_SUCCEEDED:
                self.halt('Nav2 did not succeed')
                return
        except Exception as exc:
            self.halt(f'Nav2 result error: {exc}')
            return
        self.output.publish(Twist())
        self.stationary_since = None
        self.transition('SETTLING')

    def nav_velocity(self, msg):
        if all(math.isfinite(v) for v in (msg.linear.x, msg.angular.z)):
            self.nav_cmd, self.nav_at = msg, time.monotonic()

    def odom(self, msg):
        age = (self.get_clock().now() - Time.from_msg(msg.header.stamp)).nanoseconds / 1e9
        if not 0 <= age <= self.p['sensor_timeout']:
            return
        self.odom_at = time.monotonic()
        values = (msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z)
        if all(math.isfinite(v) and abs(v) < 0.01 for v in values):
            if self.stationary_since is None:
                self.stationary_since = self.odom_at
        else:
            self.stationary_since = None

    def clear(self, msg):
        self.is_clear, self.clear_at = msg.data, time.monotonic()

    def point(self, msg):
        if self.state not in ('WAIT_PATIENT', 'APPROACHING'):
            return
        stamp = Time.from_msg(msg.header.stamp)
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        if not 0 <= age <= self.p['sensor_timeout'] or stamp.nanoseconds <= self.last_stamp:
            return
        try:
            if msg.header.frame_id == self.p['base_frame']:
                p = msg.point
            else:
                tf = self.tf.lookup_transform(self.p['base_frame'], msg.header.frame_id, stamp)
                p = do_transform_point(msg, tf).point
        except TransformException:
            return
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z)) or p.x <= 0:
            return
        if (stamp.nanoseconds - self.last_stamp) / 1e9 > self.p['sensor_timeout']:
            self.samples, self.stable_since = 0, None
        self.samples += 1
        self.last_stamp = stamp.nanoseconds
        self.sample = (p.x, p.y, stamp, time.monotonic())
        in_range = (abs(math.hypot(p.x, p.y) - self.p['target_distance']) <= 0.05
                    and abs(math.atan2(p.y, p.x)) <= 0.10)
        if in_range:
            if self.stable_since is None:
                self.stable_since = stamp.nanoseconds
        else:
            self.stable_since = None

    def tick(self):
        now = time.monotonic()
        if now - self.odom_at > self.p['sensor_timeout']:
            self.stationary_since = None
        self.status.publish(String(data=self.state))
        cmd = Twist()
        if self.state not in ('IDLE', 'DONE', 'FAILED') and now - self.since > self.p['phase_timeout']:
            self.halt('Phase timed out')
        if self.state == 'NAVIGATING':
            if now - self.nav_at <= self.p['sensor_timeout']:
                cmd = self.nav_cmd
        elif self.state == 'SETTLING':
            if (now - self.odom_at <= self.p['sensor_timeout']
                    and self.stationary_since is not None
                    and now - self.stationary_since >= 0.5):
                self.sample, self.samples, self.last_stamp = None, 0, -1
                self.transition('WAIT_PATIENT')
        elif self.state in ('WAIT_PATIENT', 'APPROACHING'):
            clear = self.is_clear and now - self.clear_at <= self.p['sensor_timeout']
            fresh = self.sample is not None and now - self.sample[3] <= self.p['sensor_timeout']
            if fresh:
                age = (self.get_clock().now() - self.sample[2]).nanoseconds / 1e9
                fresh = 0 <= age <= self.p['sensor_timeout']
            if self.state == 'WAIT_PATIENT':
                if clear and fresh and self.samples >= 5:
                    self.transition('APPROACHING')
            elif not clear or not fresh or now - self.odom_at > self.p['sensor_timeout']:
                # Latched failure: no automatic restart when data returns.
                self.halt('Approach stopped: stale sensor/odometry or blocked path')
            else:
                x, y, stamp, _ = self.sample
                error = math.hypot(x, y) - self.p['target_distance']
                heading = math.atan2(y, x)
                if error < -0.05:
                    self.halt('Patient inside target distance; no automatic reverse')
                elif (self.stable_since is not None
                      and (stamp.nanoseconds - self.stable_since) / 1e9 >= 0.5
                      and self.stationary_since is not None
                      and now - self.stationary_since >= 0.3):
                    self.transition('DONE')
                else:
                    if abs(heading) > 0.10:
                        cmd.angular.z = max(-self.p['max_turn_rate'],
                                            min(self.p['max_turn_rate'], 0.8 * heading))
                    if abs(heading) <= 0.15 and error > 0.05:
                        cmd.linear.x = min(self.p['max_speed'], 0.5 * error)
        self.output.publish(cmd)


def main():
    rclpy.init()
    node = ApproachManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.output.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
