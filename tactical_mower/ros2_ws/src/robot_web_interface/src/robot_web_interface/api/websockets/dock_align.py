"""Dock-alignment WebSocket handler (live cross/heading/perp for dock setup)."""
import asyncio
from fastapi import WebSocket, WebSocketDisconnect
from ...ros_interface.robot_node import RobotNode


async def websocket_dock_align(websocket: WebSocket, ros_node: RobotNode):
    """Stream the live fused-marker dock alignment at ~5 Hz."""
    await websocket.accept()
    try:
        while True:
            await websocket.send_json({
                "type": "dock_align",
                "data": ros_node.get_dock_align(),
            })
            await asyncio.sleep(0.2)
    except (WebSocketDisconnect, Exception):
        pass
