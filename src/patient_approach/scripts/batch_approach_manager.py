#!/usr/bin/env python3
"""Nav2 handoff + one measured target + odometry closed-loop approach."""
import math
import time
import rclpy
from rclpy.time import Time
from geometry_msgs.msg import PointStamped, Twist
from std_msgs.msg import Bool, String
from approach_manager import ApproachManager


class BatchApproachManager(ApproachManager):
    def __init__(self):
        super().__init__()
        self.measure_duration = float(self.declare_parameter('measure_duration', 10.0).value)
        self.trigger_pub = self.create_publisher(Bool, '/goal_complete', 10)
        self.create_subscription(Bool, '/goal_complete', self.external_trigger, 10)
        self.create_subscription(PointStamped, '/patient/measurement', self.measurement, 10)
        self.create_subscription(String, '/patient_measurement/state', self.measure_state, 10)
        self.pose = None
        self.odom_frame = None
        self.destination = None
        self.arrived_at = None

    def external_trigger(self, msg):
        """Allow an existing Nav2 stack to hand off with /goal_complete."""
        if not msg.data or self.state not in ('IDLE', 'DONE', 'FAILED'):
            return
        if self.pose is None or self.stationary_since is None:
            self.get_logger().warning('goal_complete ignored: waiting for stationary /Odometry')
            return
        if time.monotonic() - self.stationary_since < 0.3:
            self.get_logger().warning('goal_complete ignored: robot is not stationary yet')
            return
        self.destination = None
        self.arrived_at = None
        self.measure_ns = self.get_clock().now().nanoseconds
        self.measure_pose = self.pose
        self.transition('MEASURING')

    def goal(self, msg):
        if self.state in ('IDLE', 'DONE', 'FAILED') and not self.pending and not self.goal_handle:
            self.trigger_pub.publish(Bool(data=False))
        super().goal(msg)

    def odom(self, msg):
        age = (self.get_clock().now()-Time.from_msg(msg.header.stamp)).nanoseconds/1e9
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        values = (p.x, p.y, q.x, q.y, q.z, q.w, msg.twist.twist.linear.x,
                  msg.twist.twist.linear.y, msg.twist.twist.angular.z)
        if not 0 <= age <= self.p['sensor_timeout']:
            self.get_logger().warning(
                f'/Odometry rejected: stamp age={age:.3f}s, limit={self.p["sensor_timeout"]:.3f}s',
                throttle_duration_sec=2.0)
            return
        if not all(math.isfinite(v) for v in values):
            self.get_logger().warning('/Odometry rejected: non-finite pose or velocity',
                                      throttle_duration_sec=2.0)
            return
        norm = math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
        if abs(norm-1.) > .01:
            self.get_logger().warning(f'/Odometry rejected: quaternion norm={norm:.4f}',
                                      throttle_duration_sec=2.0)
            return
        if msg.child_frame_id != self.p['base_frame']:
            self.get_logger().warning(
                f'/Odometry rejected: child_frame_id={msg.child_frame_id!r}, '
                f'expected {self.p["base_frame"]!r}', throttle_duration_sec=2.0)
            return
        pose = (p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
        if self.state in ('MEASURING', 'MOVING') and self.pose is not None:
            if msg.header.frame_id != self.odom_frame or math.hypot(p.x-self.pose[0], p.y-self.pose[1]) > .2:
                self.halt('Odometry discontinuity')
                return
        self.pose, self.odom_frame = pose, msg.header.frame_id
        super().odom(msg)

    def measure_state(self, msg):
        if self.state == 'MEASURING' and msg.data == 'FAILED':
            self.halt('No fresh stable target in measurement window')

    def measurement(self, msg):
        if self.state != 'MEASURING' or self.pose is None:
            return
        ns = Time.from_msg(msg.header.stamp).nanoseconds
        age = (self.get_clock().now().nanoseconds-ns)/1e9
        if (time.monotonic()-self.since < self.measure_duration or ns < self.measure_ns
                or not 0 <= age <= .5 or msg.header.frame_id != self.p['base_frame']):
            return
        x, y = msg.point.x, msg.point.y
        if not all(math.isfinite(v) for v in (x, y)) or x <= 0:
            return
        distance = math.hypot(x, y)
        if distance < self.p['target_distance']-.05:
            self.halt('Target too close')
            return
        travel = max(0., distance-self.p['target_distance'])
        if travel > 3.0:
            self.halt('Requested approach exceeds 3m limit')
            return
        angle = self.pose[2]+math.atan2(y, x)
        self.destination = (self.pose[0]+travel*math.cos(angle), self.pose[1]+travel*math.sin(angle))
        self.arrived_at = None
        self.transition('MOVING')

    def tick(self):
        if self.state not in ('MEASURING', 'MOVING'):
            super().tick()
            if self.state == 'WAIT_PATIENT' and self.pose is not None:
                self.transition('MEASURING')
                self.measure_ns = self.get_clock().now().nanoseconds
                self.measure_pose = self.pose
                self.trigger_pub.publish(Bool(data=True))
            return
        now = time.monotonic()
        self.status.publish(String(data=self.state))
        cmd = Twist()
        if now-self.odom_at > self.p['sensor_timeout']:
            self.halt('Odometry timeout')
        elif self.state == 'MEASURING':
            yaw_delta = math.atan2(math.sin(self.pose[2]-self.measure_pose[2]), math.cos(self.pose[2]-self.measure_pose[2]))
            if math.hypot(self.pose[0]-self.measure_pose[0], self.pose[1]-self.measure_pose[1]) > .02 or abs(yaw_delta) > .02:
                self.halt('Robot moved during measurement')
            elif now-self.since > self.measure_duration+2.0:
                self.halt('Measurement timeout')
        elif not self.is_clear or now-self.clear_at > self.p['sensor_timeout']:
            self.halt('Blocked path or stale clearance')
        elif now-self.since > self.p['phase_timeout']:
            self.halt('Move timeout')
        else:
            dx, dy = self.destination[0]-self.pose[0], self.destination[1]-self.pose[1]
            remaining = math.hypot(dx, dy)
            heading = math.atan2(math.sin(math.atan2(dy, dx)-self.pose[2]), math.cos(math.atan2(dy, dx)-self.pose[2]))
            if remaining <= .02:
                if self.stationary_since is not None and now-self.stationary_since >= .3:
                    self.transition('DONE')
            else:
                if abs(heading) > .04:
                    cmd.angular.z = max(-self.p['max_turn_rate'], min(self.p['max_turn_rate'], .8*heading))
                if abs(heading) < .15:
                    cmd.linear.x = min(self.p['max_speed'], .6*remaining)
        self.output.publish(cmd)


def main():
    rclpy.init()
    node = BatchApproachManager()
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
