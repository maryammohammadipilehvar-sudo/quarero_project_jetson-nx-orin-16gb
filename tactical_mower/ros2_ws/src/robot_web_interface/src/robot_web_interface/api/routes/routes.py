"""Routes API endpoints"""
import uuid
from fastapi import APIRouter
from ...services.route_service import get_routes, get_route, save_route, delete_route
from ...utils.file_manager import SCHEDULES_FILE
from ...ros_interface.robot_node import RobotNode

router = APIRouter(prefix="/api/routes", tags=["routes"])

# Injected by main.py via init_routes_router so start_now/ can call the ROS service.
ros_node: RobotNode = None


def init_routes_router(node: RobotNode):
    """Inject the shared ROS node so endpoints can call services."""
    global ros_node
    ros_node = node


@router.get("")
async def list_routes():
    """Get list of all routes"""
    try:
        routes = get_routes()
        return {"routes": routes}
    except Exception as e:
        return {"routes": [], "error": str(e)}


@router.get("/{route_name}")
async def get_route_by_name(route_name: str):
    """Get route data by name"""
    try:
        route = get_route(route_name)
        if route is None:
            return {"error": "Route not found"}
        return route
    except Exception as e:
        return {"error": str(e)}


@router.post("/save")
async def save_route_endpoint(route_data: dict):
    """Save route - Home Point as first waypoint"""
    try:
        success, message, route_name = save_route(route_data)
        if success:
            return {"status": "success", "route": route_name}
        else:
            return {"status": "error", "message": message}
    except Exception as e:
        return {"status": "error", "message": str(e)}


@router.post("/start_now/{route_name}")
async def start_route_now(route_name: str):
    """Load the named route from disk and dispatch it to the wp_follower.

    Wraps the existing /control/waypoints ROS service. Operator must still
    have autonomous_operation enabled for the robot to actually drive — the
    wp_follower control loop gates on that flag.
    """
    if ros_node is None:
        return {"status": "error", "message": "ROS bridge nicht bereit"}
    route = get_route(route_name)
    if route is None:
        return {"status": "error", "message": f"Route '{route_name}' nicht gefunden"}
    waypoints = route.get("waypoints") or []
    if not waypoints:
        return {"status": "error", "message": "Route hat keine Wegpunkte"}
    try:
        result = await ros_node.publish_waypoints(
            waypoints,
            bool(route.get("loop_mode", False)),
            str(uuid.uuid4()),
            route_name,
        )
        if result.get("success"):
            return {"status": "success", "route": route_name, "waypoints": len(waypoints)}
        return {"status": "error", "message": result.get("message", "Service-Aufruf fehlgeschlagen")}
    except Exception as e:
        return {"status": "error", "message": str(e)}


@router.delete("/delete/{route_name}")
async def delete_route_endpoint(route_name: str):
    """Delete route and cascade-remove it from any schedules that reference it."""
    try:
        success, message, summary = delete_route(route_name, SCHEDULES_FILE)
        if success:
            return {"status": "success", **summary}
        else:
            return {"status": "error", "message": message, **summary}
    except Exception as e:
        return {"status": "error", "message": str(e)}

