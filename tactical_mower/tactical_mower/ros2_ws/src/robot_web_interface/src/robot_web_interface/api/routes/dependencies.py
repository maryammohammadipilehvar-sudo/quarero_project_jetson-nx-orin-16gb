"""Dependencies for API routes"""
from typing import Optional
from ...ros_interface.robot_node import RobotNode
from ...utils.connection_manager import ConnectionManager

# Global instances (set by main.py)
ros_node: Optional[RobotNode] = None
connection_manager: Optional[ConnectionManager] = None


def get_ros_node() -> RobotNode:
    """Get ROS node instance"""
    if ros_node is None:
        raise RuntimeError("ROS node not initialized")
    return ros_node


def get_connection_manager() -> ConnectionManager:
    """Get connection manager instance"""
    if connection_manager is None:
        raise RuntimeError("Connection manager not initialized")
    return connection_manager

