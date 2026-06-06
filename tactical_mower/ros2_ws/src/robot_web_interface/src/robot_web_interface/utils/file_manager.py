"""File I/O utilities for YAML files"""
import yaml
from pathlib import Path
from typing import Dict, Any

# Paths
# Prefer the same base directory used by the control stack (/routen) to keep
# routes/settings consistent with the route manager and scheduler. Fallback to
# /data to remain compatible with existing deployments.
_BASE_DIR_CANDIDATES = [Path("/routen"), Path("/data")]
DATA_DIR = next((p for p in _BASE_DIR_CANDIDATES if p.exists()), Path("/data"))

ROUTES_DIR = DATA_DIR / "routes"
SETTINGS_DIR = DATA_DIR / "settings"

SCHEDULES_FILE = SETTINGS_DIR / "schedules.yaml"
SETTINGS_FILE = SETTINGS_DIR / "settings.yaml"
ARUCO_DOCK_FILE = SETTINGS_DIR / "aruco_dock.yaml"

# Ensure directories exist
DATA_DIR.mkdir(parents=True, exist_ok=True)
ROUTES_DIR.mkdir(parents=True, exist_ok=True)
SETTINGS_DIR.mkdir(parents=True, exist_ok=True)


def load_settings() -> Dict[str, Any]:
    """Load system settings from file. Preserves all fields from YAML, not just known ones."""
    defaults = {
        "battery_threshold": 20,
        "home_tolerance": 0.5,
        "auto_charge_return": True,
        "speed_factor": 1.0,
        "waypoint_tolerance": 0.5,
        "max_route_distance": 3.0,
        "enable_obstacle_avoidance": True,
        "charge_point": {
            "latitude": None,
            "longitude": None,
            "yaw": None
        },
        "home_point": {
            "latitude": None,
            "longitude": None
        },
    }
    
    if not SETTINGS_FILE.exists():
        return defaults
    
    with open(SETTINGS_FILE) as f:
        data = yaml.safe_load(f) or {}
        # Merge defaults with loaded data, preserving all fields from YAML
        result = defaults.copy()
        result.update(data)
        # Ensure nested dicts are properly merged
        if "charge_point" in data:
            result["charge_point"] = {**defaults["charge_point"], **data["charge_point"]}
        if "home_point" in data:
            result["home_point"] = {**defaults["home_point"], **data["home_point"]}
        return result


def save_settings(settings: Dict[str, Any]) -> None:
    """Save system settings to file."""
    with open(SETTINGS_FILE, 'w') as f:
        yaml.dump(settings, f)


def load_aruco_dock() -> Dict[str, Any]:
    """Load the ArUco dock config (empty dict if absent)."""
    if not ARUCO_DOCK_FILE.exists():
        return {}
    with open(ARUCO_DOCK_FILE) as f:
        return yaml.safe_load(f) or {}


def save_aruco_dock(cfg: Dict[str, Any]) -> None:
    """Save the ArUco dock config (preserves all keys passed in)."""
    with open(ARUCO_DOCK_FILE, 'w') as f:
        yaml.dump(cfg, f, default_flow_style=False, sort_keys=True)


def load_schedules() -> Dict[str, Any]:
    """Load schedules from file."""
    if not SCHEDULES_FILE.exists():
        return {"schedules": []}
    with open(SCHEDULES_FILE) as f:
        data = yaml.safe_load(f) or {}
        return data


def save_schedules(schedules_data: Dict[str, Any]) -> None:
    """Save schedules to file."""
    with open(SCHEDULES_FILE, 'w') as f:
        yaml.dump(schedules_data, f)

