"""Control API endpoints"""
import uuid
import yaml
from datetime import datetime
from fastapi import APIRouter
from ...services.route_service import get_route
from ...utils.file_manager import ROUTES_DIR
from ...utils.connection_manager import ConnectionManager
from ...ros_interface.robot_node import RobotNode

router = APIRouter(prefix="/api/control", tags=["control"])

# These will be injected by main.py
ros_node: RobotNode = None
connection_manager: ConnectionManager = None

# Track autonomous operation state (default False)
autonomous_operation_enabled = False


def init_control_router(node: RobotNode, manager: ConnectionManager):
    """Initialize router with dependencies"""
    global ros_node, connection_manager
    ros_node = node
    connection_manager = manager


async def broadcast_log(event: dict) -> None:
    """Broadcast log event"""
    if connection_manager:
        await connection_manager.broadcast({"type": "event", "data": event})


@router.post("/clear_route")
async def clear_route():
    """Publish empty GeoPath to clear/stop current route"""
    try:
        command_id = str(uuid.uuid4())
        # Publish empty waypoints (will set mode to STOP)
        result = await ros_node.publish_waypoints([], False, command_id, '')
        
        if result.get("success", False):
            await broadcast_log({
                "timestamp": datetime.now().strftime("%H:%M:%S"),
                "message": "🛑 Route gelöscht / gestoppt",
                "level": "info"
            })
            return {"status": "success", "message": "Route cleared"}
        else:
            return {"status": "error", "message": result.get("message", "Failed to clear route")}
    except Exception as e:
        return {"status": "error", "message": str(e)}


@router.post("/stop")
async def stop_autonomous():
    """Toggle autonomous operation (stop/resume)"""
    global autonomous_operation_enabled
    try:
        # Toggle the state
        autonomous_operation_enabled = not autonomous_operation_enabled
        ros_node.status_manager.autonomous_enabled = autonomous_operation_enabled
        
        command_id = str(uuid.uuid4())
        
        # Publish the new state via service (this will stop/resume movement without clearing waypoints)
        result = await ros_node.set_autonomous_operation(autonomous_operation_enabled, command_id)
        
        if not result.get("success", False):
            await broadcast_log({
                "timestamp": datetime.now().strftime("%H:%M:%S"),
                "message": f"❌ Fehler: Autonome Operation - {result.get('message', 'Keine Antwort')}",
                "level": "error"
            })
            return {"status": "error", "message": result.get("message", "Service-Aufruf fehlgeschlagen")}
        
        if autonomous_operation_enabled:
            message = "✅ Autonome Operation aktiviert"
        else:
            message = "⏹️ Autonome Operation gestoppt"
        
        await broadcast_log({
            "timestamp": datetime.now().strftime("%H:%M:%S"),
            "message": message,
            "level": "info"
        })
        
        return {"status": "success", "autonomous_enabled": autonomous_operation_enabled}
    except Exception as e:
        return {"status": "error", "message": str(e)}


@router.get("/autonomous")
async def get_autonomous_status():
    """Get autonomous operation status"""
    global autonomous_operation_enabled
    # Synchronize global state with status_manager
    autonomous_operation_enabled = ros_node.status_manager.autonomous_enabled
    return {"autonomous_enabled": autonomous_operation_enabled}


@router.get("/light")
async def get_light_status():
    """Get light status"""
    return {"light_status": ros_node.light_status}


@router.post("/light")
async def control_light(data: dict):
    """Control light"""
    try:
        state = data.get("state", False)
        command_id = str(uuid.uuid4())
        
        # Call service directly - response includes success status
        result = await ros_node.set_light(state, command_id)
        
        if not result.get("success", False):
            await broadcast_log({
                "timestamp": datetime.now().strftime("%H:%M:%S"),
                "message": f"❌ Fehler: Licht schalten - {result.get('message', 'Keine Antwort')}",
                "level": "error"
            })
            return {"status": "error", "message": result.get("message", "Service-Aufruf fehlgeschlagen")}
        
        # Update status manager
        ros_node._publisher_methods.status_manager.light_status = state
        
        # Log success message
        message = "💡 Licht Ein" if state else "💡 Licht Aus"
        await broadcast_log({
            "timestamp": datetime.now().strftime("%H:%M:%S"),
            "message": message,
            "level": "info"
        })
        
        await connection_manager.broadcast({"type": "light_status", "data": state})

        return {"status": "success", "light_status": state}
    except Exception as e:
        return {"status": "error", "message": str(e)}


@router.get("/alarm")
async def get_alarm_status():
    """Get alarm status"""
    return {"alarm_status": ros_node.alarm_status}


@router.post("/alarm")
async def control_alarm(data: dict):
    """Control alarm"""
    try:
        state = data.get("state", False)
        command_id = str(uuid.uuid4())
        
        # Call service directly
        result = await ros_node.set_alarm(state, command_id)
        
        if not result.get("success", False):
            await broadcast_log({
                "timestamp": datetime.now().strftime("%H:%M:%S"),
                "message": f"❌ Fehler: Alarm schalten - {result.get('message', 'Keine Antwort')}",
                "level": "error"
            })
            return {"status": "error", "message": result.get("message", "Service-Aufruf fehlgeschlagen")}
        
        # Update status manager
        ros_node._publisher_methods.status_manager.alarm_status = state
        
        return {"status": "success", "alarm_status": state}
    except Exception as e:
        return {"status": "error", "message": str(e)}


@router.get("/siren")
async def get_siren_status():
    """Get siren status"""
    return {"siren_status": ros_node.siren_status}


@router.post("/siren")
async def control_siren(data: dict):
    """Control siren"""
    try:
        state = data.get("state", False)
        command_id = str(uuid.uuid4())
        
        # Call service directly
        result = await ros_node.set_siren(state, command_id)
        
        if not result.get("success", False):
            await broadcast_log({
                "timestamp": datetime.now().strftime("%H:%M:%S"),
                "message": f"❌ Fehler: Sirene schalten - {result.get('message', 'Keine Antwort')}",
                "level": "error"
            })
            return {"status": "error", "message": result.get("message", "Service-Aufruf fehlgeschlagen")}
        
        # Update status manager
        ros_node._publisher_methods.status_manager.siren_status = state
        
        return {"status": "success", "siren_status": state}
    except Exception as e:
        return {"status": "error", "message": str(e)}


@router.post("/move")
async def control_move(data: dict):
    """Control robot movement"""
    try:
        x = data.get("x", 0.0)
        y = data.get("y", 0.0)
        x = max(-1.0, min(1.0, float(x)))
        y = max(-1.0, min(1.0, float(y)))
        ros_node.send_move_command(x, y)
        left_stick_forward = max(-100, min(100, int(round(x * 100.0))))
        right_stick_left = max(-100, min(100, int(round(y * 100.0))))
        ros_node.publish_joy_command(left_stick_forward, right_stick_left)
        return {
            "status": "success",
            "x": x,
            "y": y,
            "joy": {
                "left_stick_forward": left_stick_forward,
                "right_stick_left": right_stick_left
            }
        }
    except Exception as e:
        return {"status": "error", "message": str(e)}


@router.post("/emergency-stop")
async def emergency_stop(data: dict):
    """Emergency stop control"""
    try:
        desired_state = bool(data.get("state", True))
        command_id = str(uuid.uuid4())
        
        if desired_state:
            # Call emergency stop service
            result = await ros_node.trigger_emergency_stop_service(True, command_id)
            
            if not result.get("success", False):
                await broadcast_log({
                    "timestamp": datetime.now().strftime("%H:%M:%S"),
                    "message": f"❌ Fehler: NOT-STOPP - {result.get('message', 'Keine Antwort')}",
                    "level": "error"
                })
                return {"status": "error", "message": result.get("message", "Service-Aufruf fehlgeschlagen")}
            
            # Also publish empty waypoints to stop movement via service
            waypoint_command_id = str(uuid.uuid4())
            await ros_node.publish_waypoints([], False, waypoint_command_id, '')
            
            return {"status": "success", "emergency_active": True}
        else:
            # Call emergency stop service to clear
            result = await ros_node.trigger_emergency_stop_service(False, command_id)
            
            if not result.get("success", False):
                await broadcast_log({
                    "timestamp": datetime.now().strftime("%H:%M:%S"),
                    "message": f"❌ Fehler: NOT-STOPP deaktivieren - {result.get('message', 'Keine Antwort')}",
                    "level": "error"
                })
                return {"status": "error", "message": result.get("message", "Service-Aufruf fehlgeschlagen")}
            
            return {"status": "success", "emergency_active": False}
    except Exception as e:
        return {"status": "error", "message": str(e)}


@router.post("/charge/go_to_charge")
async def go_to_charge():
    """Go to charge position and charge"""
    try:
        command_id = str(uuid.uuid4())
        
        # Call service directly
        result = await ros_node.publish_go_to_charge(True, command_id)
        
        if not result.get("success", False):
            await broadcast_log({
                "timestamp": datetime.now().strftime("%H:%M:%S"),
                "message": f"❌ Fehler: Rückkehr zur Ladestation - {result.get('message', 'Keine Antwort')}",
                "level": "error"
            })
            return {"status": "error", "message": result.get("message", "Service-Aufruf fehlgeschlagen")}
        
        return {"status": "success"}
    except Exception as e:
        return {"status": "error", "message": str(e)}


@router.get("/charge/status")
async def get_charging_status():
    """Get charging status"""
    return {"charging_enabled": ros_node._publisher_methods.status_manager.charging_status}


@router.post("/charge/manual")
async def charge_manual(data: dict):
    """Enable/disable charging relay manually"""
    try:
        state = data.get("state", False)
        command_id = str(uuid.uuid4())
        
        # Call service directly
        result = await ros_node.publish_charge_manual(state, command_id)
        
        if not result.get("success", False):
            await broadcast_log({
                "timestamp": datetime.now().strftime("%H:%M:%S"),
                "message": f"❌ Fehler: Ladevorgang Relay - {result.get('message', 'Keine Antwort')}",
                "level": "error"
            })
            return {"status": "error", "message": result.get("message", "Service-Aufruf fehlgeschlagen")}
        
        # Update status manager
        ros_node._publisher_methods.status_manager.charging_status = state
        
        # Broadcast status change to all connected clients
        await connection_manager.broadcast({"type": "charging_status", "data": state})
        
        return {"status": "success", "charging_enabled": state}
    except Exception as e:
        return {"status": "error", "message": str(e)}


@router.post("/save_waypoint")
async def save_waypoint():
    """Save current GPS position as a waypoint (tablet button fallback for PS5 Circle)."""
    pos = ros_node._publisher_methods.status_manager.current_position
    lat = pos.get("latitude", 0.0)
    lon = pos.get("longitude", 0.0)
    if lat == 0.0 and lon == 0.0:
        return {"status": "error", "message": "Kein GPS-Signal verfügbar"}
    wp = {"latitude": lat, "longitude": lon}
    await connection_manager.broadcast({"type": "waypoint_saved", "data": wp})
    return {"status": "success", "waypoint": wp}

