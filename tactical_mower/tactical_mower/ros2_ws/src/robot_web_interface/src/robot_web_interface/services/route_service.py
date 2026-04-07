"""Route service for CRUD operations.

This service handles route file I/O for the web interface.
For read operations in ROS2 nodes, use the RouteManagerService (/route_manager/*).
The web interface needs direct file access for creating/editing/deleting routes.
Both systems use the same file format and location (/routen/routes/*.yaml).
"""
import yaml
from pathlib import Path
from typing import Dict, List, Optional
from ..utils.file_manager import ROUTES_DIR, load_settings


def get_routes() -> List[str]:
    """Get list of all route names"""
    try:
        routes = [f.stem for f in ROUTES_DIR.glob("*.yaml")]
        return sorted(routes)
    except Exception as e:
        return []


def get_route(route_name: str) -> Optional[Dict]:
    """Get route data by name"""
    try:
        route_file = ROUTES_DIR / f"{route_name}.yaml"
        if not route_file.exists():
            return None
        with open(route_file) as f:
            data = yaml.safe_load(f)
            return {
                "name": route_name,
                "loop_mode": data.get("loop_mode", False),
                "waypoints": data.get("waypoints", [])
            }
    except Exception as e:
        return None


def save_route(route_data: Dict) -> tuple[bool, str, Optional[str]]:
    """
    Save route - Home Point (not Charge Point!) as first waypoint
    
    Returns: (success, message, route_name)
    """
    try:
        name = route_data.get("name", "").strip()
        if not name:
            return False, "Name required", None

        settings = load_settings()
        home_point = settings.get("home_point", {}) or {}

        if not home_point.get("latitude") or not home_point.get("longitude"):
            return False, "Home Point muss erst in den Einstellungen gesetzt werden!", None

        safe_name = "".join(c for c in name if c.isalnum() or c in (' ', '-', '_')).strip()
        safe_name = safe_name.replace(' ', '_')

        route_file = ROUTES_DIR / f"{safe_name}.yaml"

        waypoints = route_data.get("waypoints", [])
        
        home_waypoint = {
            "latitude": home_point.get("latitude"),
            "longitude": home_point.get("longitude"),
            "altitude": 0.0
        }
        
        if not waypoints or waypoints[0] != home_waypoint:
            waypoints.insert(0, home_waypoint)

        data = {
            "name": name,
            "loop_mode": route_data.get("loop_mode", False),
            "waypoints": waypoints
        }

        with open(route_file, 'w') as f:
            yaml.dump(data, f)

        return True, "success", safe_name
    except Exception as e:
        return False, str(e), None


def delete_route(route_name: str, schedules_file: Path) -> tuple[bool, str]:
    """
    Delete route if not used in any schedule
    
    Returns: (success, message)
    """
    try:
        route_file = ROUTES_DIR / f"{route_name}.yaml"
        if not route_file.exists():
            return False, "Route nicht gefunden"
        
        if schedules_file.exists():
            with open(schedules_file) as f:
                schedules_data = yaml.safe_load(f) or {}
                schedules = schedules_data.get("schedules", [])
                
                for schedule in schedules:
                    routes = schedule.get("routes", [])
                    for route in routes:
                        if route.get("route_name") == route_name:
                            return False, "Route wird in Schedule genutzt. Schedule löschen und erneut versuchen"
        
        route_file.unlink()
        return True, "success"
    except Exception as e:
        return False, str(e)

