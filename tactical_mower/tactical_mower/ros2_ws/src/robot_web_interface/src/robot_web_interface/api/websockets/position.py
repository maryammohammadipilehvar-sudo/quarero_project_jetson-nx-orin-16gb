"""Position WebSocket handler"""
import asyncio
from fastapi import WebSocket, WebSocketDisconnect
from ...ros_interface.robot_node import RobotNode
from ...utils.connection_manager import ConnectionManager


async def websocket_position(websocket: WebSocket, ros_node: RobotNode, connection_manager: ConnectionManager):
    """Position and status WebSocket stream"""
    await connection_manager.connect(websocket)
    try:
        while True:
            try:
                position = ros_node.get_position()
                await websocket.send_json({
                    "type": "position",
                    "data": position
                })
                await websocket.send_json({
                    "type": "light_status",
                    "data": ros_node.light_status
                })
                await websocket.send_json({
                    "type": "autonomous_status",
                    "data": ros_node.autonomous_enabled
                })
                await websocket.send_json({
                    "type": "charging_status",
                    "data": ros_node._publisher_methods.status_manager.charging_status
                })
                fusion = ros_node.get_fusion_state()
                await websocket.send_json({
                    "type": "fusion_status",
                    "data": fusion
                })
            except Exception as e:
                # If send fails, connection is likely dead, break the loop
                break
            await asyncio.sleep(0.5)
    except (WebSocketDisconnect, Exception):
        pass
    finally:
        connection_manager.disconnect(websocket)

