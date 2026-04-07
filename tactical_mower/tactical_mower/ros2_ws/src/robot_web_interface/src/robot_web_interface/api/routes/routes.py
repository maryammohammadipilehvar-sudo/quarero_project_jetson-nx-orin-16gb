"""Routes API endpoints"""
from fastapi import APIRouter
from ...services.route_service import get_routes, get_route, save_route, delete_route
from ...utils.file_manager import SCHEDULES_FILE

router = APIRouter(prefix="/api/routes", tags=["routes"])


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


@router.delete("/delete/{route_name}")
async def delete_route_endpoint(route_name: str):
    """Delete route if not used in schedules"""
    try:
        success, message = delete_route(route_name, SCHEDULES_FILE)
        if success:
            return {"status": "success"}
        else:
            return {"status": "error", "message": message}
    except Exception as e:
        return {"status": "error", "message": str(e)}

