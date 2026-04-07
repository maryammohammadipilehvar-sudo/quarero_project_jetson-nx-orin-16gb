"""Settings service for loading and saving settings"""
import yaml
from typing import Dict, Any, Optional
from ..utils.file_manager import load_settings, save_settings, SCHEDULES_FILE, ROUTES_DIR
from .geo_calculator import calculate_home_point_from_charge


def get_settings() -> Dict[str, Any]:
    """Get all settings"""
    return load_settings()


def save_settings_data(settings: Dict[str, Any]) -> None:
    """Save settings"""
    save_settings(settings)


def get_home_position() -> Dict[str, Any]:
    """Get both charge_point and home_point"""
    settings = load_settings()
    charge = settings.get("charge_point", {}) or {}
    home = settings.get("home_point", {}) or {}
    
    return {
        "charge_point": charge,
        "home_point": home
    }


def save_home_position(position_data: Dict, current_position: Dict, fusion_state: Dict) -> tuple[bool, str, Optional[Dict]]:
    """
    Save charge point and calculate home point automatically.
    Home Point = 2m backwards (opposite to Yaw direction) from Charge Point.
    
    Returns: (success, message, result_data)
    """
    try:
        charge_lat = position_data.get("latitude")
        charge_lon = position_data.get("longitude")

        if charge_lat is None or charge_lon is None:
            return False, "Latitude and longitude required", None
        
        fusion_status = fusion_state.get("fusion_status")

        if fusion_status not in (2, 3, 4):
            return False, f"Fusion-Status nicht stabil (Status={fusion_status}). Position kann nur bei grünem Fusion-Status gespeichert werden.", None
        
        yaw = current_position.get("yaw")

        if yaw is None:
            return False, "Kein Yaw-Wert verfügbar", None

        # Home Point berechnen (2m rückwärts vom Charge Point)
        home_lat, home_lon = calculate_home_point_from_charge(
            float(charge_lat),
            float(charge_lon),
            float(yaw),
            distance_m=2.0
        )

        settings = load_settings()
        
        # Charge Point speichern
        settings["charge_point"] = {
            "latitude": float(charge_lat),
            "longitude": float(charge_lon),
            "yaw": float(yaw)
        }
        
        # Home Point speichern (berechneter Punkt)
        settings["home_point"] = {
            "latitude": home_lat,
            "longitude": home_lon
        }
        
        save_settings(settings)

        return True, "success", {
            "charge_point": settings["charge_point"],
            "home_point": settings["home_point"]
        }
    except Exception as e:
        return False, str(e), None


def delete_all_schedules() -> int:
    """Delete all schedules. Returns count of deleted schedules."""
    deleted_count = 0
    if SCHEDULES_FILE.exists():
        with open(SCHEDULES_FILE) as f:
            schedules_data = yaml.safe_load(f) or {}
            deleted_count = len(schedules_data.get("schedules", []))
        with open(SCHEDULES_FILE, 'w') as f:
            yaml.dump({"schedules": []}, f)
    return deleted_count


def delete_all_routes() -> int:
    """Delete all route files. Returns count of deleted routes."""
    route_files = list(ROUTES_DIR.glob("*.yaml"))
    deleted_count = len(route_files)
    for route_file in route_files:
        route_file.unlink()
    return deleted_count


def reset_home_and_charge_point(settings: Dict[str, Any]) -> Dict[str, Any]:
    """Reset home and charge point to None values."""
    settings["charge_point"] = {"latitude": None, "longitude": None, "yaw": None}
    settings["home_point"] = {"latitude": None, "longitude": None}
    return settings

