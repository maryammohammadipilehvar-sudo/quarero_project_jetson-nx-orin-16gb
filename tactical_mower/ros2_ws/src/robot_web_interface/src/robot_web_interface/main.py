#!/usr/bin/env python3
"""
ROS2 + FastAPI Robot Web Interface - REFACTORED

Organized structure:
- ros_interface/: ROS2 node, publishers, subscribers, status manager
- api/routes/: REST API endpoints
- api/websockets/: WebSocket handlers
- services/: Business logic (routes, schedules, settings)
- utils/: Connection manager, file I/O
"""

import asyncio
import hashlib
import os
import signal
import sys
import threading
from contextlib import asynccontextmanager
from pathlib import Path
from fastapi import FastAPI, WebSocket
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
from starlette.staticfiles import StaticFiles as StarletteStaticFiles
from starlette.responses import Response
import rclpy
from rclpy.executors import MultiThreadedExecutor

# ROS Interface
from .ros_interface.robot_node import RobotNode

# Utils
from .utils.connection_manager import ConnectionManager

# API Routes
from .api.routes import routes, control, settings, scheduler, events, security_arrival
from .api.websockets import camera, position, robot_state

# Global instances
ros_node: RobotNode = None
connection_manager: ConnectionManager = None
event_loop: asyncio.AbstractEventLoop = None
ros_executor: MultiThreadedExecutor = None
ros_thread: threading.Thread = None
_initialized = False
_init_lock = threading.Lock()

# App version - computed from static files hash at startup
_app_version: str = None


def compute_static_files_hash(static_dir: str = "/app/static") -> str:
    """
    Compute a hash of all static files to detect changes.
    This creates a unique version string that changes when any static file changes.
    """
    hasher = hashlib.md5()
    static_path = Path(static_dir)
    
    if not static_path.exists():
        # Fallback for development
        return os.getenv("APP_VERSION", "dev")
    
    # Get all files sorted by path for consistent hashing
    all_files = sorted(static_path.rglob("*"))
    
    for file_path in all_files:
        if file_path.is_file():
            # Include file path and modification time in hash
            hasher.update(str(file_path).encode())
            hasher.update(str(file_path.stat().st_mtime).encode())
    
    return hasher.hexdigest()[:12]  # First 12 chars of MD5


def get_app_version() -> str:
    """Get the current app version (computed once at startup)"""
    global _app_version
    if _app_version is None:
        # Check for explicit version from environment (set during Docker build)
        _app_version = os.getenv("APP_VERSION")
        if not _app_version:
            # Compute from static files
            _app_version = compute_static_files_hash()
    return _app_version


class CacheControlStaticFiles(StarletteStaticFiles):
    """
    Custom StaticFiles that adds Cache-Control headers.
    Forces revalidation while still allowing conditional caching.
    """
    async def get_response(self, path: str, scope) -> Response:
        response = await super().get_response(path, scope)
        # no-cache: Browser must revalidate with server before using cached version
        # This allows ETag/Last-Modified to work while ensuring version checks
        response.headers["Cache-Control"] = "no-cache, must-revalidate"
        return response


def html_response(file_path: str) -> FileResponse:
    """
    Helper function to serve HTML files with proper Cache-Control headers.
    Ensures browser always revalidates HTML pages to get latest version.
    """
    response = FileResponse(file_path)
    response.headers["Cache-Control"] = "no-cache, must-revalidate"
    return response


def cleanup_ros2():
    """Cleanup ROS2 resources - can be called from signal handlers"""
    global ros_node, ros_executor, _initialized
    import time
    
    with _init_lock:
        if not _initialized:
            return
        
        print("Cleaning up ROS2 node (signal handler)")
        
        # Step 1: Stop the executor first
        if ros_executor is not None:
            try:
                print("Shutting down executor")
                ros_executor.shutdown()
                time.sleep(0.3)
            except Exception as e:
                print(f"Error shutting down executor: {e}")
            ros_executor = None
        
        # Step 2: Destroy the node
        if ros_node is not None:
            try:
                node_name = ros_node.get_name()
                print(f"Destroying node {node_name}")
                ros_node.destroy_node()
                time.sleep(0.5)  # Wait for FastDDS to process removal
            except Exception as e:
                print(f"Error destroying node: {e}")
            ros_node = None
        
        # Step 3: Shutdown rclpy
        if rclpy.ok():
            try:
                print("Shutting down rclpy")
                rclpy.shutdown()
                time.sleep(0.3)
            except Exception as e:
                print(f"Error shutting down rclpy: {e}")
        
        _initialized = False
        print("Cleanup complete")


def signal_handler(signum, frame):
    """Handle SIGTERM/SIGINT to ensure proper cleanup"""
    print(f"Received signal {signum}, cleaning up...")
    cleanup_ros2()
    sys.exit(0)


# Register signal handlers for graceful shutdown
signal.signal(signal.SIGTERM, signal_handler)
signal.signal(signal.SIGINT, signal_handler)


def ros_spin():
    """ROS2 executor spin in separate thread"""
    global ros_executor
    if ros_executor is not None:
        ros_executor.spin()


@asynccontextmanager
async def lifespan(app: FastAPI):
    """FastAPI lifespan manager"""
    global ros_node, connection_manager, event_loop, ros_executor, ros_thread, _initialized
    
    # Prevent multiple initializations (thread-safe)
    with _init_lock:
        if _initialized:
            # Already initialized, skip
            print("WARNING: lifespan called multiple times, skipping initialization")
            yield
            return
        
        _initialized = True
        print("Initializing ROS2 node (lifespan start)")
    
    try:
        event_loop = asyncio.get_running_loop()

        # Initialize connection manager
        connection_manager = ConnectionManager()

        # Initialize ROS2 - ensure it's only initialized once
        # Check if rclpy is already initialized (could be from previous run or another process)
        try:
            if not rclpy.ok():
                print("Calling rclpy.init()")
                rclpy.init()
            else:
                print("WARNING: rclpy already initialized! This might cause duplicate nodes.")
        except RuntimeError as e:
            # rclpy might already be initialized
            if "already initialized" not in str(e).lower():
                raise
            print(f"rclpy already initialized: {e}")
        
        # Wait a moment for any stale nodes from previous runs to be cleaned up by FastDDS discovery
        # This helps prevent duplicate node appearances in ros2 node list
        import time
        print("Waiting for FastDDS discovery to clean up stale nodes...")
        time.sleep(2.0)  # Give FastDDS time to remove old node entries
        
        # IMPORTANT: Create executor BEFORE creating node to prevent automatic registration
        # If we create the node first, it might be automatically added to a default executor
        print("Creating MultiThreadedExecutor (before node creation)")
        ros_executor = MultiThreadedExecutor()
        
        # Create ROS2 node - it will be automatically registered in ROS2 graph when created
        # But we explicitly control which executor it's added to
        print("Creating RobotNode")
        ros_node = RobotNode(connection_manager, event_loop)
        node_name = ros_node.get_name()
        print(f"RobotNode created with name: {node_name}")
        
        # IMPORTANT: Immediately add node to our executor to prevent default executor registration
        # This ensures the node is only in one executor
        print("Immediately adding node to our executor (preventing default executor)")
        ros_executor.add_node(ros_node)
        print(f"Node {node_name} added to executor")
        
        # Add node to executor (explicitly, not relying on default)
        # IMPORTANT: Only add to our executor, not to any default executor
        # The node is automatically registered in ROS2 graph when created,
        # adding to executor only manages callbacks, not graph registration
        print("Adding node to executor")
        ros_executor.add_node(ros_node)
        print(f"Node {node_name} added to executor")
        
        # Verify node is only in our executor
        nodes_in_executor = ros_executor.get_nodes()
        print(f"Nodes in executor: {[n.get_name() for n in nodes_in_executor]}")
        
        # Check if node name was auto-renamed (indicates duplicate)
        if node_name != 'robot_web_interface':
            print(f"WARNING: Node name was auto-renamed to {node_name}, indicating a duplicate exists!")
        
        # Initialize API routers with dependencies
        control.init_control_router(ros_node, connection_manager)
        settings.init_settings_router(ros_node)
        scheduler.init_scheduler_router(ros_node, connection_manager)
        routes.init_routes_router(ros_node)
        security_arrival.init_security_arrival_router(ros_node)

        # Start ROS2 in separate thread
        print("Starting ROS2 executor thread")
        ros_thread = threading.Thread(target=ros_spin, daemon=True)
        ros_thread.start()
        print("ROS2 executor thread started")
        
        yield
        
    finally:
        # Cleanup - use the shared cleanup function
        cleanup_ros2()


# Create FastAPI app
app = FastAPI(lifespan=lifespan)

# Include routers
app.include_router(routes.router)
app.include_router(control.router)
app.include_router(settings.router)
app.include_router(scheduler.router)
app.include_router(events.router)
app.include_router(security_arrival.router)

# Static file serving with cache control
app.mount("/static", CacheControlStaticFiles(directory="/app/static"), name="static")


# HTML page routes - all use html_response() for proper cache control
@app.get("/")
async def get_interface():
    """Main interface page"""
    return html_response("/app/static/index.html")


@app.get("/scheduler")
async def get_scheduler():
    """Scheduler page"""
    return html_response("/app/static/scheduler.html")


@app.get("/settings")
async def get_settings():
    """Settings page"""
    return html_response("/app/static/settings.html")


@app.get("/events")
async def get_events_page():
    """Security events review page"""
    return html_response("/app/static/events.html")


@app.get("/control")
async def get_control():
    """Control page"""
    return html_response("/app/static/control.html")


# WebSocket endpoints
@app.websocket("/ws/position")
async def websocket_position_endpoint(websocket: WebSocket):
    """Position WebSocket endpoint"""
    global ros_node, connection_manager
    await position.websocket_position(websocket, ros_node, connection_manager)


@app.websocket("/ws/camera/main")
async def websocket_camera_main_endpoint(websocket: WebSocket):
    """Main camera WebSocket endpoint"""
    global ros_node
    await camera.websocket_camera_main(websocket, ros_node)


@app.websocket("/ws/camera/thermal1")
async def websocket_camera_thermal1_endpoint(websocket: WebSocket):
    """Thermal camera 1 WebSocket endpoint"""
    global ros_node
    await camera.websocket_camera_thermal1(websocket, ros_node)


@app.websocket("/ws/camera/thermal2")
async def websocket_camera_thermal2_endpoint(websocket: WebSocket):
    """Thermal camera 2 WebSocket endpoint"""
    global ros_node
    await camera.websocket_camera_thermal2(websocket, ros_node)


@app.websocket("/ws/camera/rgb2")
async def websocket_camera_rgb2_endpoint(websocket: WebSocket):
    """RGB2 camera WebSocket endpoint (Axis channel 1)"""
    global ros_node
    await camera.websocket_camera_rgb2(websocket, ros_node)


@app.websocket("/ws/camera/lidar_debug")
async def websocket_camera_lidar_debug_endpoint(websocket: WebSocket):
    """LIDAR debug (Livox) camera WebSocket endpoint"""
    global ros_node
    await camera.websocket_camera_lidar_debug(websocket, ros_node)


@app.websocket("/ws/camera/person_detection")
async def websocket_camera_person_detection_endpoint(websocket: WebSocket):
    await camera.websocket_camera_person_detection(websocket, ros_node)


@app.websocket("/ws/camera/depth_debug")
async def websocket_camera_depth_debug_endpoint(websocket: WebSocket):
    """RealSense depth obstacle debug WebSocket endpoint"""
    global ros_node
    await camera.websocket_camera_depth_debug(websocket, ros_node)


@app.websocket("/ws/robot_state")
async def websocket_robot_state_endpoint(websocket: WebSocket):
    """Robot state WebSocket endpoint"""
    global ros_node
    await robot_state.websocket_robot_state(websocket, ros_node)


# Logs API
@app.get("/api/logs")
async def get_logs():
    """Get all persistent logs"""
    global connection_manager
    if connection_manager:
        logs = connection_manager.get_logs()
        return {"logs": logs}
    return {"logs": []}


@app.delete("/api/logs")
async def clear_logs():
    """Clear all persistent logs"""
    global connection_manager
    if connection_manager:
        connection_manager.clear_logs()
        # Broadcast log clear event to all clients
        await connection_manager.broadcast({
            "type": "logs_cleared",
            "data": {"message": "Logs cleared"}
        })
        return {"status": "success", "message": "Logs cleared"}
    return {"status": "error", "message": "Connection manager not available"}


# Debug mode API
@app.get("/api/debug/enabled")
async def get_debug_enabled():
    """Check if debug mode is enabled"""
    debug_mode = os.getenv("DEBUG_MODE", "false").lower() == "true"
    return {"debug_enabled": debug_mode}


# Version API for cache busting
@app.get("/api/version")
async def get_version():
    """
    Get the current app version.
    Frontend uses this to detect when a new version is deployed
    and trigger a cache refresh.
    """
    return {"version": get_app_version()}


def main() -> None:
    """Main entry point"""
    import uvicorn
    # Disable reload to prevent multiple node instances
    # Use single worker to avoid duplicate nodes
    uvicorn.run(app, host="0.0.0.0", port=8020, reload=False, workers=1)


if __name__ == '__main__':
    main()
