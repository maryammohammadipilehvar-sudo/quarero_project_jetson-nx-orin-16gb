"""Scheduler API endpoints"""
import uuid
from datetime import datetime
from fastapi import APIRouter
from ...services.schedule_service import (
    get_schedules, create_robot_config,
    prepare_schedule_for_save, prepare_schedule_for_edit,
    prepare_schedule_for_toggle, prepare_schedule_for_delete
)
from ...ros_interface.robot_node import RobotNode
from ...utils.connection_manager import ConnectionManager

router = APIRouter(prefix="/api/schedule", tags=["schedule"])

# Will be injected by main.py
ros_node: RobotNode = None
connection_manager: ConnectionManager = None


def init_scheduler_router(node: RobotNode, manager: ConnectionManager):
    """Initialize router with dependencies"""
    global ros_node, connection_manager
    ros_node = node
    connection_manager = manager


async def broadcast_log(event: dict) -> None:
    """Broadcast log event"""
    if connection_manager:
        await connection_manager.broadcast({"type": "event", "data": event})


@router.get("/schedules")
async def get_schedules_endpoint():
    """Get all schedules"""
    try:
        schedules = get_schedules()
        return {"schedules": schedules}
    except Exception as e:
        return {"schedules": [], "error": str(e)}


@router.post("/save")
async def save_schedule_endpoint(schedule_data: dict):
    """Save schedule"""
    try:
        # Prepare schedule data but don't write to file yet
        success, message, schedule_id, schedule_dict = prepare_schedule_for_save(schedule_data)
        
        if not success:
            return {"status": "error", "message": message}
        
        robot_config = create_robot_config(schedule_data, schedule_id)
        command_id = str(uuid.uuid4())
        
        # Call robot service - it will write to the schedules.yaml file directly
        result = await ros_node.publish_schedule_add(robot_config, command_id)
        
        if not result.get("success", False):
            await broadcast_log({
                "timestamp": datetime.now().strftime("%H:%M:%S"),
                "message": f"❌ FEHLER: Zeitplan nicht vom Roboter bestätigt! - {result.get('message', 'Keine Antwort')}",
                "level": "error"
            })
            return {"status": "error", "message": result.get("message", "Keine Bestätigung vom Roboter erhalten")}
        
        return {"status": "success", "schedule_id": schedule_id}
    except Exception as e:
        return {"status": "error", "message": str(e)}


@router.put("/edit/{index}")
async def edit_schedule_endpoint(index: int, schedule_data: dict):
    """Edit schedule at index"""
    try:
        # Prepare edit data but don't write to file
        success, message, edit_info = prepare_schedule_for_edit(index, schedule_data)
        
        if not success:
            return {"status": "error", "message": message}
        
        old_schedule_id = edit_info["old_schedule_id"]
        new_schedule = edit_info["new_schedule"]
        
        # Remove old schedule via robot service (writes to file)
        command_id1 = str(uuid.uuid4())
        result_remove = await ros_node.publish_schedule_remove(old_schedule_id, command_id1)
        
        if not result_remove.get("success", False):
            await broadcast_log({
                "timestamp": datetime.now().strftime("%H:%M:%S"),
                "message": f"❌ FEHLER: Alten Zeitplan konnte der Roboter nicht entfernen! - {result_remove.get('message', 'Keine Antwort')}",
                "level": "error"
            })
            return {"status": "error", "message": result_remove.get("message", "Keine Bestätigung vom Roboter für Entfernung")}
        
        # Add new schedule via robot service (writes to file)
        robot_config = create_robot_config(schedule_data, new_schedule["schedule_id"])
        command_id2 = str(uuid.uuid4())
        result_add = await ros_node.publish_schedule_add(robot_config, command_id2)
        
        if not result_add.get("success", False):
            await broadcast_log({
                "timestamp": datetime.now().strftime("%H:%M:%S"),
                "message": f"❌ FEHLER: Neuer Zeitplan nicht vom Roboter bestätigt! - {result_add.get('message', 'Keine Antwort')}",
                "level": "error"
            })
            return {"status": "error", "message": result_add.get("message", "Keine Bestätigung vom Roboter für neuen Zeitplan")}
        
        return {"status": "success"}
    except Exception as e:
        return {"status": "error", "message": str(e)}


@router.delete("/delete/{index}")
async def delete_schedule_endpoint(index: int):
    """Delete schedule at index"""
    try:
        # Prepare deletion (get schedule_id) without deleting from disk
        success, message, schedule_id = prepare_schedule_for_delete(index)
        
        if not success:
            return {"status": "error", "message": message}
        
        # Call robot service - the robot service will update the schedules.yaml file directly
        command_id = str(uuid.uuid4())
        result = await ros_node.publish_schedule_remove(schedule_id, command_id)
        
        if not result.get("success", False):
            await broadcast_log({
                "timestamp": datetime.now().strftime("%H:%M:%S"),
                "message": f"❌ WARNUNG: Zeitplan-Löschung vom Roboter nicht bestätigt! (Datei unverändert) - {result.get('message', 'Keine Antwort')}",
                "level": "warn"
            })
            return {"status": "error", "message": result.get("message", "Lokal nicht gelöscht – keine Bestätigung vom Roboter")}
        
        return {"status": "success"}
    except Exception as e:
        return {"status": "error", "message": str(e)}


@router.post("/toggle/{index}")
async def toggle_schedule_endpoint(index: int):
    """Toggle schedule active state"""
    try:
        # Prepare toggle data without saving to disk
        success, message, toggle_info = prepare_schedule_for_toggle(index)
        
        if not success:
            return {"status": "error", "message": message}
        
        old_schedule_id = toggle_info["old_schedule_id"]
        new_schedule = toggle_info["new_schedule"]
        new_active_state = toggle_info["active"]
        
        # Use update action instead of remove + add for cleaner toggle handling
        robot_config = create_robot_config(new_schedule, new_schedule["schedule_id"])
        command_id = str(uuid.uuid4())
        result_update = await ros_node.publish_schedule_update(robot_config, old_schedule_id, command_id)
        
        if not result_update.get("success", False):
            await broadcast_log({
                "timestamp": datetime.now().strftime("%H:%M:%S"),
                "message": f"❌ FEHLER: Zeitplan-Toggle nicht vom Roboter bestätigt! - {result_update.get('message', 'Keine Antwort')}",
                "level": "error"
            })
            return {"status": "error", "message": result_update.get("message", "Keine Bestätigung vom Roboter")}
        
        # Robot service succeeded and has already updated the file - do NOT update it again
        
        return {"status": "success", "active": new_active_state}
    except Exception as e:
        return {"status": "error", "message": str(e)}

