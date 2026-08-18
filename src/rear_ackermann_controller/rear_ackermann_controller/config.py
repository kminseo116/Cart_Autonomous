"""Configuration helpers for simple runtime parameter loading."""

from __future__ import annotations

import json
from pathlib import Path

from .vehicle_params import VehicleParams


def _resolve_config_path(config_path: str | None = None) -> Path:
    """Resolve a config path robustly from the workspace root or a provided path."""

    if config_path is None:
        workspace_path = Path.cwd() / "src" / "params_setting.json"
        if workspace_path.exists():
            return workspace_path
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("rear_ackermann_controller")) / "config" / "params_setting.json"

    config_file = Path(config_path).expanduser()
    if not config_file.is_absolute():
        config_file = Path.cwd() / config_file

    return config_file


def load_vehicle_params(config_path: str | None = None) -> VehicleParams:
    """Load the single cart-parameter source from ``params_setting.json``."""

    config_file = _resolve_config_path(config_path)

    if not config_file.exists():
        return VehicleParams()

    with config_file.open("r", encoding="utf-8") as handle:
        raw_config = json.load(handle)

    vehicle_fields = VehicleParams.__dataclass_fields__
    return VehicleParams(**{
        name: float(raw_config[name]) for name in vehicle_fields if name in raw_config
    })


def save_vehicle_params(params: dict[str, float], config_path: str) -> None:
    """Save vehicle parameters to a JSON file."""

    config_file = _resolve_config_path(config_path)
    config_file.parent.mkdir(parents=True, exist_ok=True)

    with config_file.open("w", encoding="utf-8") as handle:
        json.dump(params, handle, indent=2)
