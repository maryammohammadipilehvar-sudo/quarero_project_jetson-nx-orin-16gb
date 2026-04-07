"""Home return controller for generating reverse routes."""

from pathlib import Path
from typing import Optional, Tuple
from ..navigation.coordinate_transformer import CoordinateTransformer
from ..routes.route_manager import RouteManager
from interfaces.msg import GeoPath


class HomeReturnController:
    """Manages home return logic and reverse route generation."""
    
    def __init__(
        self,
        coordinate_transformer: CoordinateTransformer,
        route_manager: RouteManager,
        tolerance: float = 0.1
    ):
        """Initialize home return controller.
        
        Args:
            coordinate_transformer: Coordinate transformer instance
            route_manager: Route manager for loading routes
            tolerance: Distance tolerance for home arrival check (meters)
        """
        self.coord_transformer = coordinate_transformer
        self.route_manager = route_manager
        self.tolerance = tolerance
    
    def is_at_home(
        self,
        robot_position: Tuple[float, float, float],
        home_position: Tuple[float, float, float]
    ) -> bool:
        """Check if robot is at home position.
        
        Args:
            robot_position: Current robot position (lat, lon, alt)
            home_position: Home position (lat, lon, alt)
            
        Returns:
            True if at home within tolerance, False otherwise
        """
        if not robot_position or not home_position:
            return False
        
        distance = self.coord_transformer.compute_distance(robot_position, home_position)
        return distance < self.tolerance
    
    def create_reverse_route(
        self, 
        route_name: str, 
        current_position: Optional[Tuple[float, float, float]] = None
    ) -> Optional[GeoPath]:
        """Create optimal home return route from route file.
        
        Uses intelligent routing: for non-looped routes, returns reversed path.
        For looped routes, calculates whether going backwards or continuing forward
        is shorter and returns the optimal path.
        
        Args:
            route_name: Name of route file
            current_position: Current robot position (lat, lon, alt). If provided,
                            enables optimal path calculation for looped routes.
            
        Returns:
            GeoPath message with optimal waypoints for home return, or None if route not found
        """
        if current_position:
            return self.route_manager.find_shortest_way_home(route_name, current_position)
        else:
            # Fallback to simple reverse if position not available
            return self.route_manager.create_reverse_geopath(route_name, mode=GeoPath.ONCE)
    
    def find_closest_route_for_home_return(
        self,
        current_position: Tuple[float, float, float],
        max_distance: float = 50.0,
        use_perpendicular_entry: bool = True
    ) -> Optional[Tuple[str, float, GeoPath]]:
        """Find the closest route to current position and create reverse path home.
        
        This method is useful when the robot needs to return home but doesn't
        have a predefined home return route. It finds the nearest route and
        creates a reverse path along it.
        
        Args:
            current_position: Current robot position (lat, lon, alt)
            max_distance: Maximum distance in meters to search for routes
            use_perpendicular_entry: If True, insert a perpendicular waypoint to take
                                    the shortest path to rejoin the route. If False,
                                    navigate directly to the nearest waypoint.
            
        Returns:
            Tuple of (route_name, distance, geopath) or None if no route found
        """
        if not current_position:
            return None
        
        try:
            # Find closest route to current position
            route_name, distance = self.route_manager.find_closest_route(
                current_position,
                distance_threshold=max_distance
            )
            
            if not route_name:
                return None
            
            # Create optimal home return geopath for this route
            if use_perpendicular_entry:
                geopath = self.route_manager.find_shortest_way_home_with_perpendicular_entry(
                    route_name, current_position
                )
            else:
                geopath = self.create_reverse_route(route_name, current_position)
            
            if not geopath:
                return None
            
            return (route_name, distance, geopath)
            
        except Exception:
            return None

