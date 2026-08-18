"""Compatibility wrappers for the former rear-Ackermann API."""

from __future__ import annotations

import math
from .rear_steer_kinematics import compute_rear_steer_command
from .vehicle_params import VehicleParams


def compute_rear_steering_angles(
    vehicle_speed_mps: float,
    angular_velocity_rad_per_s: float,
    params: VehicleParams | None = None,
) -> tuple[float, float]:
    """Return steering angles only; prefer ``compute_rear_steer_command``."""

    command = compute_rear_steer_command(
        vehicle_speed_mps, angular_velocity_rad_per_s, params or VehicleParams()
    )
    return command.left.steering_angle_rad, command.right.steering_angle_rad


def compute_turn_radius_m(vehicle_speed_mps: float, angular_velocity_rad_per_s: float) -> float:
    """Compute the turning radius from linear and angular velocity."""

    if abs(angular_velocity_rad_per_s) < 1e-9:
        return float("inf")

    return abs(vehicle_speed_mps / angular_velocity_rad_per_s)
