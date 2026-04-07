"""WebSocket Connection Manager"""
from typing import List, Deque, Dict
from collections import deque
from fastapi import WebSocket
import logging
import threading

logger = logging.getLogger(__name__)

# Maximum number of recent logs to keep in buffer for reconnecting clients
MAX_LOG_BUFFER_SIZE = 100


class ConnectionManager:
    """Manages WebSocket connections for broadcasting messages"""
    
    def __init__(self, max_log_buffer_size: int = MAX_LOG_BUFFER_SIZE):
        self.active_connections: List[WebSocket] = []
        # Buffer to store recent log events for reconnecting clients
        self.log_buffer: Deque[dict] = deque(maxlen=max_log_buffer_size)
        # Persistent log storage (shared across all clients)
        self.persistent_logs: Deque[dict] = deque(maxlen=max_log_buffer_size)
        self._log_lock = threading.Lock()
        # Track active camera stream connections to optimize processing
        self.active_camera_streams: set = set()  # Set of camera types: 'main', 'thermal1', etc.
        self._camera_streams_lock = threading.Lock()
        # Track multiple WebSocket connections per camera type
        self.camera_connections: Dict[str, List[WebSocket]] = {}  # camera_type -> list of websockets
        self._camera_connections_lock = threading.Lock()

    async def connect(self, websocket: WebSocket) -> None:
        """Connect a new WebSocket client and send recent logs"""
        await websocket.accept()
        self.active_connections.append(websocket)
        
        # Send buffered logs to the newly connected client
        if self.log_buffer:
            try:
                for log_event in self.log_buffer:
                    await websocket.send_json({"type": "event", "data": log_event})
            except Exception as e:
                logger.warning(f"Failed to send buffered logs to new client: {e}")
                # If sending fails, the connection is likely dead, remove it
                self.disconnect(websocket)

    def disconnect(self, websocket: WebSocket) -> None:
        """Remove a WebSocket connection"""
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)

    async def broadcast(self, message: dict) -> None:
        """Broadcast a message to all active connections"""
        # If this is a log event, add it to the buffer and persistent storage
        if message.get("type") == "event" and "data" in message:
            log_data = message["data"]
            self.log_buffer.append(log_data)
            
            # Don't store connection status messages in persistent storage (they are temporary)
            # Only store regular log messages
            if log_data.get("connection_status") is None:
                # Add to persistent storage (thread-safe)
                # deque with maxlen automatically limits size
                with self._log_lock:
                    self.persistent_logs.append(log_data)
        
        # Broadcast to all active connections, removing dead ones
        dead_connections = []
        for connection in self.active_connections:
            try:
                await connection.send_json(message)
            except Exception as e:
                # Connection is dead, mark for removal
                logger.debug(f"Failed to send message to WebSocket client: {e}")
                dead_connections.append(connection)
        
        # Remove dead connections
        for dead_conn in dead_connections:
            self.disconnect(dead_conn)
        
        if dead_connections:
            logger.info(f"Removed {len(dead_connections)} dead WebSocket connection(s)")

    def get_logs(self) -> List[dict]:
        """Get all persistent logs (thread-safe)"""
        with self._log_lock:
            return list(self.persistent_logs)

    def clear_logs(self) -> None:
        """Clear all persistent logs (thread-safe)"""
        with self._log_lock:
            self.persistent_logs.clear()
            # Also clear the buffer
            self.log_buffer.clear()

    def get_active_connection_count(self) -> int:
        """Get the number of active connections"""
        return len(self.active_connections)
    
    def register_camera_stream(self, camera_type: str) -> None:
        """Register an active camera stream connection"""
        with self._camera_streams_lock:
            self.active_camera_streams.add(camera_type)
            logger.info(f"Camera stream '{camera_type}' registered. Active streams: {self.active_camera_streams}")
    
    def unregister_camera_stream(self, camera_type: str) -> None:
        """Unregister a camera stream connection"""
        with self._camera_streams_lock:
            self.active_camera_streams.discard(camera_type)
            logger.info(f"Camera stream '{camera_type}' unregistered. Active streams: {self.active_camera_streams}")
    
    def is_camera_stream_active(self, camera_type: str) -> bool:
        """Check if a camera stream has active connections"""
        with self._camera_streams_lock:
            return camera_type in self.active_camera_streams
    
    def has_any_camera_stream_active(self) -> bool:
        """Check if any camera stream is active"""
        with self._camera_streams_lock:
            return len(self.active_camera_streams) > 0
    
    def add_camera_connection(self, camera_type: str, websocket: WebSocket) -> None:
        """Add a WebSocket connection for a camera stream"""
        with self._camera_connections_lock:
            if camera_type not in self.camera_connections:
                self.camera_connections[camera_type] = []
            self.camera_connections[camera_type].append(websocket)
            logger.debug(f"Added camera connection for '{camera_type}'. Total connections: {len(self.camera_connections[camera_type])}")
    
    def remove_camera_connection(self, camera_type: str, websocket: WebSocket) -> None:
        """Remove a WebSocket connection for a camera stream"""
        with self._camera_connections_lock:
            if camera_type in self.camera_connections:
                if websocket in self.camera_connections[camera_type]:
                    self.camera_connections[camera_type].remove(websocket)
                    logger.debug(f"Removed camera connection for '{camera_type}'. Remaining connections: {len(self.camera_connections[camera_type])}")
                # Clean up empty lists
                if len(self.camera_connections[camera_type]) == 0:
                    del self.camera_connections[camera_type]
    
    def get_camera_connections(self, camera_type: str) -> List[WebSocket]:
        """Get all WebSocket connections for a camera stream"""
        with self._camera_connections_lock:
            return self.camera_connections.get(camera_type, []).copy()
    
    def has_camera_connections(self, camera_type: str) -> bool:
        """Check if there are any active connections for a camera stream"""
        with self._camera_connections_lock:
            return camera_type in self.camera_connections and len(self.camera_connections[camera_type]) > 0
    
    async def broadcast_camera_frame(self, camera_type: str, frame_data: str) -> None:
        """Broadcast a camera frame to all connected clients for this camera type"""
        connections = self.get_camera_connections(camera_type)
        if not connections:
            return
        
        dead_connections = []
        message = {
            "type": "camera",
            "data": frame_data
        }
        
        for connection in connections:
            try:
                await connection.send_json(message)
            except Exception as e:
                logger.debug(f"Failed to send camera frame to client: {e}")
                dead_connections.append(connection)
        
        # Remove dead connections
        for dead_conn in dead_connections:
            self.remove_camera_connection(camera_type, dead_conn)
        
        if dead_connections:
            logger.info(f"Removed {len(dead_connections)} dead camera connection(s) for '{camera_type}'")

