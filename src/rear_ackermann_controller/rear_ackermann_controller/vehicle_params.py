"""Vehicle parameters for rear-wheel steering control."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class VehicleParams:
    """Cart dimensions and limits, all expressed in the ``base_link`` frame."""

    body_length_m: float = 1.2
    body_width_m: float = 0.6
    rear_axle_x_m: float = -0.5
    rear_track_m: float = 0.5
    front_caster_x_m: float = 0.5
    wheel_radius_m: float = 0.1
    max_steering_angle_rad: float = 1.5707963267948966
    max_wheel_speed_mps: float = 0.4
    max_yaw_rate_rad_s: float = 0.5

    def __post_init__(self) -> None:
        """Reject non-physical dimensions at construction time."""

        if self.body_length_m <= 0.0 or self.body_width_m <= 0.0:
            raise ValueError("body dimensions must be positive")
        if self.rear_track_m <= 0.0 or self.wheel_radius_m <= 0.0:
            raise ValueError("rear_track_m and wheel_radius_m must be positive")
        if self.max_steering_angle_rad <= 0.0 or self.max_wheel_speed_mps <= 0.0:
            raise ValueError("steering and wheel-speed limits must be positive")


DEFAULT_VEHICLE_PARAMS = VehicleParams()
