"""Safety limits applied before sending cart commands."""

from __future__ import annotations

from .vehicle_params import VehicleParams


def limit_speed_mps(speed_mps: float, params: VehicleParams) -> float:
    """Clamp a requested linear speed to the configured symmetric limit."""

    return max(-params.max_speed_mps, min(params.max_speed_mps, speed_mps))
