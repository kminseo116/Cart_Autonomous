"""Tests for fixed-wheel differential drive."""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rear_ackermann_controller.rear_ackermann_math import compute_turn_radius_m
from rear_ackermann_controller.rear_steer_kinematics import compute_rear_steer_command
from rear_ackermann_controller.vehicle_params import VehicleParams


def test_straight_motion_has_zero_steering_and_equal_drive() -> None:
    command = compute_rear_steer_command(0.02, 0.0, VehicleParams())

    assert command.left.steering_angle_rad == pytest.approx(0.0)
    assert command.right.steering_angle_rad == pytest.approx(0.0)
    assert command.left.wheel_speed_mps == pytest.approx(0.02)
    assert command.right.wheel_speed_mps == pytest.approx(0.02)


def test_left_turn_has_different_fixed_wheel_speeds() -> None:
    command = compute_rear_steer_command(0.02, 0.1, VehicleParams())

    assert command.left.steering_angle_rad == command.right.steering_angle_rad == 0.0
    assert command.left.wheel_speed_mps < command.right.wheel_speed_mps


def test_in_place_rotation_has_opposing_wheel_speeds() -> None:
    command = compute_rear_steer_command(0.0, 0.1, VehicleParams())

    assert command.left.wheel_speed_mps == pytest.approx(-command.right.wheel_speed_mps)
    assert command.left.steering_angle_rad == command.right.steering_angle_rad == 0.0


def test_wheel_speed_is_limited() -> None:
    params = VehicleParams(max_wheel_speed_mps=0.2, max_yaw_rate_rad_s=0.5)
    command = compute_rear_steer_command(0.4, 0.5, params)

    assert abs(command.left.wheel_speed_mps) <= 0.2
    assert abs(command.right.wheel_speed_mps) <= 0.2


def test_wheel_speed_limit_preserves_curvature() -> None:
    params = VehicleParams(rear_track_m=0.5, max_wheel_speed_mps=0.2, max_yaw_rate_rad_s=0.5)
    command = compute_rear_steer_command(0.2, 0.2, params)

    assert command.right.wheel_speed_mps == pytest.approx(0.2)
    assert command.left.wheel_speed_mps / command.right.wheel_speed_mps == pytest.approx(0.6)


def test_zero_command_returns_safe_zero_commands() -> None:
    command = compute_rear_steer_command(0.0, 0.0, VehicleParams())

    assert command.left.wheel_speed_mps == command.right.wheel_speed_mps == 0.0


def test_turn_radius_is_infinite_for_straight_motion() -> None:
    assert compute_turn_radius_m(1.0, 0.0) == float("inf")
