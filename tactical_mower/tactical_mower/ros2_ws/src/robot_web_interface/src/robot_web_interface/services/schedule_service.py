"""Schedule service for schedule operations and ROS2 sync"""
import yaml
from datetime import datetime
from typing import Dict, List, Optional, Tuple
from pathlib import Path

from ..utils.file_manager import SCHEDULES_FILE, ROUTES_DIR


def _normalize_route_name(name: str) -> str:
    """Strip common file extensions and whitespace from a route name."""
    if not isinstance(name, str):
        return name
    cleaned = name.strip()
    if cleaned.lower().endswith('.yaml'):
        cleaned = cleaned[:-5]
    elif cleaned.lower().endswith('.yml'):
        cleaned = cleaned[:-4]
    return cleaned


def get_schedules() -> List[Dict]:
    """Get all schedules"""
    try:
        if not SCHEDULES_FILE.exists():
            return []
        with open(SCHEDULES_FILE) as f:
            data = yaml.safe_load(f) or {}
            schedules = data.get("schedules", [])
            # Normalize field names: convert 'loop_mode' to 'loop' for consistency
            for schedule in schedules:
                if 'loop_mode' in schedule and 'loop' not in schedule:
                    schedule['loop'] = schedule.pop('loop_mode')
            return schedules
    except Exception as e:
        return []


def get_loop_mode_for_route(route_name: str) -> bool:
    """Get loop_mode from first route"""
    try:
        if route_name:
            first_route_file = ROUTES_DIR / f"{route_name}.yaml"
            if first_route_file.exists():
                with open(first_route_file) as rf:
                    route_data = yaml.safe_load(rf)
                    return route_data.get("loop_mode", False)
    except Exception:
        pass
    return False


def create_robot_config(schedule_data: Dict, schedule_id: str) -> Dict:
    """Create robot config from schedule data"""
    routes_list = schedule_data.get("routes", [])
    route_names = [_normalize_route_name(r.get('route_name', '')) for r in routes_list]
    route_repetitions = [r['repeat_count'] for r in routes_list]
    
    # Use loop from schedule_data (set via UI), default to False
    loop_mode = schedule_data.get("loop", False)
    
    return {
        "schedule_id": schedule_id,
        "route_names": route_names,
        "route_repetitions": route_repetitions,
        "route_mode": schedule_data.get("route_mode", "sequential"),
        "loop_mode": loop_mode,
        "weekdays": schedule_data.get("weekdays", []),
        "start_time": schedule_data.get("start_time"),
        "end_time": schedule_data.get("end_time"),
        "active": schedule_data.get("active", False),
        "require_home_return": schedule_data.get("require_home_return", True)
    }


def prepare_schedule_for_save(schedule_data: Dict) -> Tuple[bool, str, Optional[str], Optional[Dict]]:
    """
    Prepare schedule data for saving (generate ID, validate) without saving to disk.
    
    Returns: (success, message, schedule_id, schedule_dict)
    """
    try:
        if SCHEDULES_FILE.exists():
            with open(SCHEDULES_FILE) as f:
                data = yaml.safe_load(f) or {}
        else:
            data = {}

        if "schedules" not in data:
            data["schedules"] = []

        schedule_id = f"schedule_{len(data['schedules'])}_{int(datetime.now().timestamp())}"

        # Get loop value, defaulting to False if not present
        loop_value = schedule_data.get("loop", False)
        # Ensure it's a boolean
        if isinstance(loop_value, str):
            loop_value = loop_value.lower() in ('true', '1', 'yes')
        loop_value = bool(loop_value)

        # Normalize route names (strip .yaml/.yml)
        normalized_routes = []
        for r in schedule_data.get("routes", []):
            normalized_routes.append({
                **r,
                "route_name": _normalize_route_name(r.get("route_name", ""))
            })

        new_schedule = {
            "schedule_id": schedule_id,
            "routes": normalized_routes,
            "route_mode": schedule_data.get("route_mode", "sequential"),
            "weekdays": schedule_data.get("weekdays", []),
            "start_time": schedule_data.get("start_time"),
            "end_time": schedule_data.get("end_time"),
            "loop": loop_value,
            "require_home_return": schedule_data.get("require_home_return", True),
            "active": schedule_data.get("active", False),
        }

        return True, "success", schedule_id, new_schedule
    except Exception as e:
        return False, str(e), None, None


def save_schedule_to_file(schedule_dict: Dict) -> bool:
    """
    Save a prepared schedule dictionary to file.
    
    Args:
        schedule_dict: Schedule dictionary to save
        
    Returns:
        True if saved successfully, False otherwise
    """
    try:
        if SCHEDULES_FILE.exists():
            with open(SCHEDULES_FILE) as f:
                data = yaml.safe_load(f) or {}
        else:
            data = {}

        if "schedules" not in data:
            data["schedules"] = []

        data["schedules"].append(schedule_dict)
        with open(SCHEDULES_FILE, 'w') as f:
            yaml.dump(data, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
            f.flush()  # Ensure data is written to disk immediately

        return True
    except Exception as e:
        return False


def prepare_schedule_for_edit(index: int, schedule_data: Dict) -> Tuple[bool, str, Optional[Dict]]:
    """
    Prepare schedule edit data without saving to disk.
    
    Returns: (success, message, edit_info with old_schedule_id and new_schedule)
    """
    try:
        if not SCHEDULES_FILE.exists():
            return False, "No schedules found", None
        
        with open(SCHEDULES_FILE) as f:
            data = yaml.safe_load(f) or {}
        
        schedules = data.get("schedules", [])
        if not (0 <= index < len(schedules)):
            return False, "Invalid index", None
        
        old_schedule = schedules[index]
        old_schedule_id = old_schedule.get("schedule_id", f"schedule_{index}")
        
        new_schedule_id = f"schedule_{index}_{int(datetime.now().timestamp())}"
        
        # Get loop value, defaulting to False if not present
        loop_value = schedule_data.get("loop", False)
        # Ensure it's a boolean
        if isinstance(loop_value, str):
            loop_value = loop_value.lower() in ('true', '1', 'yes')
        loop_value = bool(loop_value)
        
        # Normalize route names (strip .yaml/.yml)
        normalized_routes = []
        for r in schedule_data.get("routes", []):
            normalized_routes.append({
                **r,
                "route_name": _normalize_route_name(r.get("route_name", ""))
            })

        new_schedule = {
            "schedule_id": new_schedule_id,
            "routes": normalized_routes,
            "route_mode": schedule_data.get("route_mode", "sequential"),
            "weekdays": schedule_data.get("weekdays", []),
            "start_time": schedule_data.get("start_time"),
            "end_time": schedule_data.get("end_time"),
            "loop": loop_value,
            "require_home_return": schedule_data.get("require_home_return", True),
            "active": schedule_data.get("active", False)
        }
        
        return True, "success", {"old_schedule_id": old_schedule_id, "new_schedule": new_schedule, "index": index}
    except Exception as e:
        return False, str(e), None


def save_schedule_edit_to_file(index: int, new_schedule: Dict) -> bool:
    """
    Save edited schedule to file.
    
    Args:
        index: Index of schedule to update
        new_schedule: New schedule dictionary
        
    Returns:
        True if saved successfully, False otherwise
    """
    try:
        if not SCHEDULES_FILE.exists():
            return False
        
        with open(SCHEDULES_FILE) as f:
            data = yaml.safe_load(f) or {}
        
        schedules = data.get("schedules", [])
        if not (0 <= index < len(schedules)):
            return False
        
        schedules[index] = new_schedule
        data["schedules"] = schedules
        
        with open(SCHEDULES_FILE, 'w') as f:
            yaml.dump(data, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
            f.flush()  # Ensure data is written to disk immediately
        
        return True
    except Exception as e:
        return False


def prepare_schedule_for_delete(index: int) -> Tuple[bool, str, Optional[str]]:
    """
    Prepare schedule deletion (get schedule_id) without deleting from disk.
    
    Returns: (success, message, schedule_id)
    """
    try:
        if not SCHEDULES_FILE.exists():
            return False, "No schedules found", None
        
        with open(SCHEDULES_FILE) as f:
            data = yaml.safe_load(f) or {}
        
        schedules = data.get("schedules", [])
        if not (0 <= index < len(schedules)):
            return False, "Invalid index", None
        
        schedule_to_delete = schedules[index]
        schedule_id = schedule_to_delete.get("schedule_id", f"schedule_{index}")
        
        return True, "success", schedule_id
    except Exception as e:
        return False, str(e), None


def save_schedule_delete_to_file(index: int) -> bool:
    """
    Delete schedule from file.
    
    Args:
        index: Index of schedule to delete
        
    Returns:
        True if deleted successfully, False otherwise
    """
    try:
        if not SCHEDULES_FILE.exists():
            print(f"[ERROR] save_schedule_delete_to_file: Schedules file does not exist")
            return False
        
        with open(SCHEDULES_FILE) as f:
            data = yaml.safe_load(f) or {}
        
        schedules = data.get("schedules", [])
        schedules_count_before = len(schedules)
        print(f"[DEBUG] save_schedule_delete_to_file: Found {schedules_count_before} schedules before deletion, deleting index {index}")
        
        if not (0 <= index < len(schedules)):
            print(f"[ERROR] save_schedule_delete_to_file: Invalid index {index} for {len(schedules)} schedules")
            return False
        
        deleted_schedule = schedules.pop(index)
        print(f"[DEBUG] save_schedule_delete_to_file: Deleted schedule: {deleted_schedule.get('schedule_id', 'unknown')}")
        data["schedules"] = schedules
        
        schedules_count_after = len(schedules)
        print(f"[DEBUG] save_schedule_delete_to_file: {schedules_count_after} schedules remaining after deletion")
        
        with open(SCHEDULES_FILE, 'w') as f:
            yaml.dump(data, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
            f.flush()  # Ensure data is written to disk immediately
        
        print(f"[DEBUG] save_schedule_delete_to_file: Successfully saved file with {schedules_count_after} schedules")
        return True
    except Exception as e:
        print(f"[ERROR] save_schedule_delete_to_file: Exception occurred: {type(e).__name__}: {str(e)}")
        import traceback
        traceback.print_exc()
        return False


def prepare_schedule_for_toggle(index: int) -> Tuple[bool, str, Optional[Dict]]:
    """
    Prepare schedule toggle (get current state, prepare new state) without saving to disk.
    
    Returns: (success, message, toggle_info with old_schedule_id, new_schedule, active state, and index)
    """
    try:
        if not SCHEDULES_FILE.exists():
            return False, "No schedules found", None
        
        with open(SCHEDULES_FILE) as f:
            data = yaml.safe_load(f) or {}
        
        schedules = data.get("schedules", [])
        if not (0 <= index < len(schedules)):
            return False, "Invalid index", None
        
        schedule = schedules[index]
        old_schedule_id = schedule.get("schedule_id", f"schedule_{index}")
        old_active_state = schedule.get("active", False)
        
        new_active_state = not old_active_state
        new_schedule_id = f"schedule_{index}_{int(datetime.now().timestamp())}"
        
        new_schedule = {
            **schedule,
            "schedule_id": new_schedule_id,
            "active": new_active_state
        }
        
        return True, "success", {
            "old_schedule_id": old_schedule_id,
            "new_schedule": new_schedule,
            "active": new_active_state,
            "index": index
        }
    except Exception as e:
        return False, str(e), None


def save_schedule_toggle_to_file(index: int, new_schedule: Dict) -> bool:
    """
    Save toggled schedule to file.
    
    Args:
        index: Index of schedule to update
        new_schedule: New schedule dictionary with toggled active state
        
    Returns:
        True if saved successfully, False otherwise
    """
    try:
        if not SCHEDULES_FILE.exists():
            return False
        
        with open(SCHEDULES_FILE) as f:
            data = yaml.safe_load(f) or {}
        
        schedules = data.get("schedules", [])
        if not (0 <= index < len(schedules)):
            return False
        
        schedules[index] = new_schedule
        data["schedules"] = schedules
        
        with open(SCHEDULES_FILE, 'w') as f:
            yaml.dump(data, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
            f.flush()  # Ensure data is written to disk immediately
        
        return True
    except Exception as e:
        return False

