"""Robot state WebSocket handler"""
import asyncio
from fastapi import WebSocket, WebSocketDisconnect
from ...ros_interface.robot_node import RobotNode


async def websocket_robot_state(websocket: WebSocket, ros_node: RobotNode):
    """Robot state WebSocket stream"""
    await websocket.accept()
    try:
        while True:
            state = ros_node.get_robot_state()
            # Include button states from status manager (synced from robot)
            state["light_state"] = ros_node.light_status
            state["alarm_state"] = ros_node.alarm_status
            state["siren_state"] = ros_node.siren_status
            state["emergency_stop_active"] = ros_node.emergency_active
            state["autonomous_enabled"] = ros_node.autonomous_enabled
            state["charging_state"] = ros_node._publisher_methods.status_manager.charging_status
            # Include robot connection status (from backend tracking)
            state["robot_connected"] = ros_node.status_manager.robot_connected
            
            await websocket.send_json({
                "type": "robot_state",
                "data": state
            })
            fusion = ros_node.get_fusion_state()
            await websocket.send_json({
                "type": "fusion_status",
                "data": fusion
            })
            await asyncio.sleep(0.5)
    except (WebSocketDisconnect, Exception):
        pass

