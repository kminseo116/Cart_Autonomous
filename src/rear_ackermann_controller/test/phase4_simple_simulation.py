"""A minimal 2D simulation for rear-wheel steering behavior."""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rear_ackermann_controller.rear_ackermann_math import compute_rear_steering_angles


def simulate_vehicle() -> list[tuple[float, float, float]]:
    """Simulate a short trajectory using simple kinematics."""

    vehicle_speed_mps = 0.5
    angular_velocity_rad_per_s = 0.4
    dt_s = 0.1
    steps = 10
    pose_history: list[tuple[float, float, float]] = []

    x_m = 0.0
    y_m = 0.0
    yaw_rad = 0.0

    for _ in range(steps):
        left_angle, right_angle = compute_rear_steering_angles(
            vehicle_speed_mps=vehicle_speed_mps,
            angular_velocity_rad_per_s=angular_velocity_rad_per_s,
        )

        yaw_rad += angular_velocity_rad_per_s * dt_s
        x_m += vehicle_speed_mps * math.cos(yaw_rad) * dt_s
        y_m += vehicle_speed_mps * math.sin(yaw_rad) * dt_s
        pose_history.append((x_m, y_m, yaw_rad))

    return pose_history


if __name__ == "__main__":
    pose_history = simulate_vehicle()
    print("pose_history:")
    for pose in pose_history:
        print(pose)
