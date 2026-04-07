"""Settings API endpoints"""
from fastapi import APIRouter
from ...services.settings_service import (
    get_settings, save_settings_data, get_home_position, save_home_position,
    delete_all_schedules, delete_all_routes, reset_home_and_charge_point
)
from ...ros_interface.robot_node import RobotNode
from ...services.route_service import get_routes

router = APIRouter(prefix="/api/settings", tags=["settings"])

# Will be injected by main.py
ros_node: RobotNode = None


def init_settings_router(node: RobotNode):
    """Initialize router with dependencies"""
    global ros_node
    ros_node = node


@router.get("/home-position")
async def get_home_position_endpoint():
    """Get both charge_point and home_point"""
    try:
        return get_home_position()
    except Exception as e:
        return {
            "charge_point": {},
            "home_point": {},
            "error": str(e)
        }


@router.post("/home-position")
async def save_home_position_endpoint(position_data: dict):
    """Save charge point and calculate home point.
    
    If delete_schedules_and_routes is True, all schedules and routes will be deleted first.
    """
    try:
        if ros_node is None:
            return {"status": "error", "message": "ROS node not initialized"}
        
        current_pos = ros_node.get_position()
        fusion_state = ros_node.get_fusion_state()
        
        deleted_schedules = 0
        deleted_routes = 0
        
        # Wenn delete_schedules_and_routes gesetzt ist, zuerst Schedules und Routen löschen
        if position_data.get("delete_schedules_and_routes", False):
            try:
                deleted_schedules = delete_all_schedules()
            except Exception as e:
                return {"status": "error", "message": f"Fehler beim Löschen der Schedules: {str(e)}"}
            
            try:
                deleted_routes = delete_all_routes()
            except Exception as e:
                return {"status": "error", "message": f"Fehler beim Löschen der Routen: {str(e)}"}
        
        success, message, result_data = save_home_position(
            position_data, current_pos, fusion_state
        )
        
        if success:
            return {
                "status": "success",
                "charge_point": result_data["charge_point"],
                "home_point": result_data["home_point"],
                "deleted_schedules": deleted_schedules,
                "deleted_routes": deleted_routes
            }
        else:
            return {"status": "error", "message": message}
    except Exception as e:
        return {"status": "error", "message": str(e)}


@router.get("/battery")
async def get_battery_settings():
    """Get battery threshold"""
    try:
        settings = get_settings()
        return {"threshold": settings.get("battery_threshold", 20)}
    except Exception as e:
        return {"threshold": 20, "error": str(e)}


@router.post("/battery")
async def save_battery_settings(battery_data: dict):
    """Save battery threshold"""
    try:
        threshold = battery_data.get("threshold")
        if threshold is None or threshold < 5 or threshold > 50:
            return {"status": "error", "message": "Threshold must be between 5 and 50"}
        settings = get_settings()
        settings["battery_threshold"] = int(threshold)
        save_settings_data(settings)
        return {"status": "success"}
    except Exception as e:
        return {"status": "error", "message": str(e)}


@router.get("/advanced")
async def get_advanced_settings():
    """Get advanced settings"""
    try:
        settings = get_settings()
        return {
            "speed_factor": settings.get("speed_factor", 1.0),
            "auto_charge_return": settings.get("auto_charge_return", True),
            "max_route_distance": settings.get("max_route_distance", 3.0),
            "waypoint_tolerance": settings.get("waypoint_tolerance", 0.5),
            "enable_obstacle_avoidance": settings.get("enable_obstacle_avoidance", True)
        }
    except Exception as e:
        return {
            "speed_factor": 1.0,
            "auto_charge_return": True,
            "max_route_distance": 3.0,
            "waypoint_tolerance": 0.5,
            "enable_obstacle_avoidance": True,
            "error": str(e)
        }


@router.post("/advanced")
async def save_advanced_settings(advanced_data: dict):
    """Save advanced settings"""
    try:
        speed_factor = advanced_data.get("speed_factor")
        auto_charge = advanced_data.get("auto_charge_return")
        max_route_distance = advanced_data.get("max_route_distance")
        enable_obstacle_avoidance = advanced_data.get("enable_obstacle_avoidance")
        settings = get_settings()

        if speed_factor is not None:
            speed_factor = float(speed_factor)
            if speed_factor < 0.5 or speed_factor > 5.0:
                return {
                    "status": "error",
                    "message": "speed_factor must be between 0.5 and 5.0"
                }

            settings["speed_factor"] = speed_factor

            if ros_node is not None:
                ros_node.publish_speed_factor(speed_factor)

        if auto_charge is not None:
            settings["auto_charge_return"] = bool(auto_charge)

        if max_route_distance is not None:
            max_route_distance = float(max_route_distance)
            if max_route_distance < 1.0 or max_route_distance > 100.0:
                return {
                    "status": "error",
                    "message": "max_route_distance must be between 1.0 and 100.0"
                }
            settings["max_route_distance"] = max_route_distance

        if enable_obstacle_avoidance is not None:
            settings["enable_obstacle_avoidance"] = bool(enable_obstacle_avoidance)
            
            if ros_node is not None:
                ros_node.publish_obstacle_avoidance_enabled(bool(enable_obstacle_avoidance))

        waypoint_tolerance = advanced_data.get("waypoint_tolerance")
        if waypoint_tolerance is not None:
            waypoint_tolerance = float(waypoint_tolerance)
            if waypoint_tolerance < 0.1 or waypoint_tolerance > 5.0:
                return {
                    "status": "error",
                    "message": "waypoint_tolerance must be between 0.1 and 5.0"
                }
            settings["waypoint_tolerance"] = waypoint_tolerance

        save_settings_data(settings)
        return {"status": "success"}
    except Exception as e:
        return {"status": "error", "message": str(e)}


@router.post("/restore")
async def restore_settings_endpoint(restore_data: dict):
    """Restore/delete selected settings categories"""
    try:
        settings = get_settings()
        errors = []
        
        # Delete Home/Charge Position
        if restore_data.get("home_position", False):
            settings = reset_home_and_charge_point(settings)
        
        # Delete Schedules
        if restore_data.get("schedules", False):
            try:
                delete_all_schedules()
            except Exception as e:
                errors.append(f"Fehler beim Löschen der Schedules: {str(e)}")
        
        # Delete Routes
        if restore_data.get("routes", False):
            try:
                delete_all_routes()
            except Exception as e:
                errors.append(f"Fehler beim Löschen der Routen: {str(e)}")
        
        # Delete Advanced Settings (reset to defaults)
        if restore_data.get("advanced", False):
            settings["speed_factor"] = 1.0
            settings["auto_charge_return"] = True
            settings["max_route_distance"] = 3.0
            settings["waypoint_tolerance"] = 0.5
            settings["enable_obstacle_avoidance"] = True
            
            # Publish default values to ROS if node is available
            if ros_node is not None:
                ros_node.publish_speed_factor(1.0)
                ros_node.publish_obstacle_avoidance_enabled(True)
        
        # Delete Email Settings
        if restore_data.get("email", False):
            settings["security_email"] = {
                "enabled": False,
                "smtp_host": "",
                "smtp_port": 587,
                "use_tls": True,
                "username": "",
                "password": "",
                "sender": "",
                "recipients": []
            }
        
        # Delete Alarm Settings
        if restore_data.get("alarm", False):
            settings["security_notification_windows"] = []
            settings["security_event_defaults"] = {
                "pre_event_seconds": 10.0,
                "post_event_seconds": 10.0
            }
            settings["security_always_enable_event_types"] = []
        
        # Save updated settings
        save_settings_data(settings)
        
        if errors:
            return {"status": "partial", "message": "Einige Einstellungen konnten nicht gelöscht werden", "errors": errors}
        else:
            return {"status": "success", "message": "Einstellungen erfolgreich gelöscht"}
    except Exception as e:
        return {"status": "error", "message": str(e)}

