"""Centralized route management for the control package.

This module provides a unified interface for all route-related operations,
including loading, saving, listing, and converting route data between formats.
"""

import yaml
import math
from pathlib import Path
from typing import Dict, List, Optional, Tuple
from geometry_msgs.msg import Point
from interfaces.msg import GeoPath
from ..navigation.geo_helper import haversine_distance, calculate_route_distance_from_waypoints


class RouteManager:
    """Centralized manager for route operations.
    
    This class encapsulates all route-related logic including:
    - Loading routes from YAML files
    - Listing available routes
    - Converting route data to GeoPath messages
    - Creating reverse routes for home return
    - Validating route data
    """
    
    def __init__(self, routes_dir: Path, logger=None):
        """Initialize the RouteManager.
        
        Args:
            routes_dir: Path to the directory containing route YAML files
            logger: Optional logger for debug messages
        """
        self.routes_dir = Path(routes_dir).expanduser()
        self.routes_dir.mkdir(parents=True, exist_ok=True)
        self.logger = logger
        self._log_debug(f"RouteManager initialized with routes_dir: {self.routes_dir}")
        self._log_debug(f"Routes directory exists: {self.routes_dir.exists()}")
        self._log_debug(f"Routes directory is directory: {self.routes_dir.is_dir()}")
    
    def _log_debug(self, message: str):
        """Log debug message if logger is available."""
        if self.logger and hasattr(self.logger, 'debug'):
            self.logger.debug(f"[RouteManager] {message}")
    
    def _log_info(self, message: str):
        """Log info message if logger is available."""
        if self.logger and hasattr(self.logger, 'info'):
            self.logger.info(f"[RouteManager] {message}")
    
    def _log_warn(self, message: str):
        """Log warning message if logger is available."""
        if self.logger and hasattr(self.logger, 'warn'):
            self.logger.warn(f"[RouteManager] {message}")
    
    def _log_error(self, message: str):
        """Log error message if logger is available."""
        if self.logger and hasattr(self.logger, 'error'):
            self.logger.error(f"[RouteManager] {message}")
    
    def list_routes(self) -> List[str]:
        """Get list of all available route names.
        
        Returns:
            List of route names (without .yaml extension)
        """
        try:
            self._log_debug(f"Listing routes in: {self.routes_dir}")
            # List all files in directory for debugging
            all_files = list(self.routes_dir.glob("*"))
            self._log_debug(f"All files in directory ({len(all_files)}): {[f.name for f in all_files]}")
            
            routes = [f.stem for f in self.routes_dir.glob("*.yaml")]
            self._log_debug(f"Found {len(routes)} .yaml routes: {routes}")
            return sorted(routes)
        except Exception as e:
            self._log_error(f"Error listing routes: {e}")
            return []
    
    def route_exists(self, route_name: str) -> bool:
        """Check if a route file exists.
        
        Args:
            route_name: Name of the route (without .yaml extension)
            
        Returns:
            True if route file exists, False otherwise
        """
        route_file = self.routes_dir / f"{route_name}.yaml"
        exists = route_file.exists()
        self._log_debug(f"Route '{route_name}' exists check: {exists} (path: {route_file})")
        return exists
    
    def load_route(self, route_name: str) -> Optional[Dict]:
        """Load route data from YAML file.
        
        Args:
            route_name: Name of the route (without .yaml extension)
            
        Returns:
            Dictionary containing route data with keys:
            - name: Route name
            - loop_mode: Boolean indicating loop mode
            - waypoints: List of waypoint dictionaries with lat/lon/alt
            Returns None if route not found or invalid
        """
        route_file = self.routes_dir / f"{route_name}.yaml"
        self._log_debug(f"Loading route '{route_name}' from: {route_file}")
        
        if not route_file.exists():
            self._log_warn(f"Route file not found: {route_file}")
            return None
        
        try:
            with open(route_file, 'r') as f:
                data = yaml.safe_load(f)
            
            self._log_debug(f"Loaded YAML data for '{route_name}': {list(data.keys()) if data else 'None'}")
            
            if not data or 'waypoints' not in data:
                self._log_warn(f"Route '{route_name}' missing waypoints or empty data")
                return None
            
            # Ensure all waypoints have required fields
            waypoints = []
            for i, wp in enumerate(data.get('waypoints', [])):
                if 'latitude' in wp and 'longitude' in wp:
                    waypoints.append({
                        'latitude': float(wp['latitude']),
                        'longitude': float(wp['longitude']),
                        'altitude': float(wp.get('altitude', 0.0))
                    })
                else:
                    self._log_warn(f"Waypoint {i} in route '{route_name}' missing lat/lon")
            
            if not waypoints:
                self._log_warn(f"Route '{route_name}' has no valid waypoints")
                return None
            
            self._log_debug(f"Successfully loaded route '{route_name}' with {len(waypoints)} waypoints")
            
            return {
                'name': data.get('name', route_name),
                'loop_mode': bool(data.get('loop_mode', False)),
                'waypoints': waypoints
            }
        except Exception as e:
            self._log_error(f"Error loading route '{route_name}': {e}")
            return None
    
    def get_waypoints(self, route_name: str) -> Optional[List[Dict]]:
        """Get waypoints from a route.
        
        Args:
            route_name: Name of the route (without .yaml extension)
            
        Returns:
            List of waypoint dictionaries with lat/lon/alt, or None if not found
        """
        route_data = self.load_route(route_name)
        if route_data:
            return route_data.get('waypoints', [])
        return None
    
    def create_geopath(
        self,
        route_name: str,
        override_mode: Optional[int] = None
    ) -> Optional[GeoPath]:
        """Create a GeoPath message from a route.
        
        Args:
            route_name: Name of the route (without .yaml extension)
            override_mode: Optional mode to override route's loop_mode
                          (GeoPath.ONCE, GeoPath.LOOP, or GeoPath.PING_PONG)
            
        Returns:
            GeoPath message, or None if route not found
        """
        route_data = self.load_route(route_name)
        if not route_data:
            return None
        
        waypoints = route_data.get('waypoints', [])
        if not waypoints:
            return None
        
        geopath = GeoPath()
        
        # Set mode based on override or route's loop_mode
        if override_mode is not None:
            geopath.mode = override_mode
        else:
            loop_mode = route_data.get('loop_mode', False)
            geopath.mode = GeoPath.LOOP if loop_mode else GeoPath.PING_PONG
        
        # Add waypoints
        for wp in waypoints:
            point = Point()
            point.x = float(wp['latitude'])
            point.y = float(wp['longitude'])
            point.z = float(wp.get('altitude', 0.0))
            geopath.waypoints.append(point)
        
        return geopath
    
    def calculate_route_distance(
        self,
        route_name: str,
        mode: Optional[int] = None
    ) -> Optional[float]:
        """Calculate total distance of a route in meters.
        
        Args:
            route_name: Name of the route (without .yaml extension)
            mode: Optional mode override (GeoPath.ONCE, GeoPath.LOOP, or GeoPath.PING_PONG)
                 If None, uses route's loop_mode
            
        Returns:
            Total distance in meters, or None if route not found
        """
        route_data = self.load_route(route_name)
        if not route_data:
            return None
        
        waypoints = route_data.get('waypoints', [])
        if len(waypoints) < 2:
            return 0.0
        
        # Determine mode string
        if mode is not None:
            # Convert GeoPath mode to string
            if mode == GeoPath.LOOP:
                mode_str = 'loop'
            elif mode == GeoPath.PING_PONG:
                mode_str = 'ping_pong'
            elif mode == GeoPath.ONCE:
                mode_str = 'once'
            else:
                mode_str = 'once'
        else:
            loop_mode = route_data.get('loop_mode', False)
            mode_str = 'loop' if loop_mode else 'ping_pong'
        
        # Use helper function to calculate distance
        return calculate_route_distance_from_waypoints(waypoints, mode_str)
    
    def create_reverse_geopath(
        self,
        route_name: str,
        mode: int = GeoPath.ONCE
    ) -> Optional[GeoPath]:
        """Create a reversed GeoPath for home return.
        
        Args:
            route_name: Name of the route (without .yaml extension)
            mode: GeoPath mode (default: ONCE for single home return)
            
        Returns:
            GeoPath with reversed waypoints, or None if route not found
        """
        route_data = self.load_route(route_name)
        if not route_data:
            return None
        
        waypoints = route_data.get('waypoints', [])
        if not waypoints:
            return None
        
        geopath = GeoPath()
        geopath.mode = mode
        
        # Add waypoints in reverse order
        for wp in reversed(waypoints):
            point = Point()
            point.x = float(wp['latitude'])
            point.y = float(wp['longitude'])
            point.z = float(wp.get('altitude', 0.0))
            geopath.waypoints.append(point)
        
        return geopath
    
    def find_shortest_way_home(
        self,
        route_name: str,
        current_position: Tuple[float, float, float]
    ) -> Optional[GeoPath]:
        """Find the shortest way home for a given route and current position.
        
        For non-looped routes: Always returns reversed path (going back the way it came).
        For looped routes: Compares the distance of going backwards vs continuing forward
        and returns the shorter path.
        
        Args:
            route_name: Name of the route (without .yaml extension)
            current_position: Current robot position as (latitude, longitude, altitude)
            
        Returns:
            GeoPath with optimal waypoints for home return, or None if route not found
        """
        route_data = self.load_route(route_name)
        if not route_data:
            self._log_warn(f"Cannot find shortest way home: route '{route_name}' not found")
            return None
        
        waypoints = route_data.get('waypoints', [])
        if not waypoints:
            self._log_warn(f"Cannot find shortest way home: route '{route_name}' has no waypoints")
            return None
        
        # Find closest waypoint to current position
        min_dist_to_waypoint = float('inf')
        closest_waypoint_idx = 0
        
        for i, wp in enumerate(waypoints):
            wp_pos = (wp['latitude'], wp['longitude'], wp['altitude'])
            dist = haversine_distance(current_position, wp_pos)
            if dist < min_dist_to_waypoint:
                min_dist_to_waypoint = dist
                closest_waypoint_idx = i

        loop_mode = route_data.get('loop_mode', False)
        if loop_mode: 
            # For looped routes, calculate both directions and choose shorter one
            self._log_debug(f"Route '{route_name}' is looped - calculating optimal direction")
            
            self._log_debug(f"Closest waypoint index: {closest_waypoint_idx} (distance: {min_dist_to_waypoint:.2f}m)")
            
            # Calculate distance going backwards (to waypoint 0)
            backward_distance = 0.0
            for i in range(closest_waypoint_idx, 0, -1):
                wp1 = waypoints[i]
                wp2 = waypoints[i - 1]
                backward_distance += haversine_distance(
                    (wp1['latitude'], wp1['longitude'], wp1['altitude']),
                    (wp2['latitude'], wp2['longitude'], wp2['altitude'])
                )
            
            # Calculate distance going forward (to complete the loop back to waypoint 0)
            forward_distance = 0.0
            num_waypoints = len(waypoints)
            
            # From closest waypoint to end of route
            for i in range(closest_waypoint_idx, num_waypoints - 1):
                wp1 = waypoints[i]
                wp2 = waypoints[i + 1]
                forward_distance += haversine_distance(
                    (wp1['latitude'], wp1['longitude'], wp1['altitude']),
                    (wp2['latitude'], wp2['longitude'], wp2['altitude'])
                )
        
            # Loop closure: from last waypoint back to first
            if num_waypoints > 1:
                forward_distance += haversine_distance(
                    (waypoints[-1]['latitude'], waypoints[-1]['longitude'], waypoints[-1]['altitude']),
                    (waypoints[0]['latitude'], waypoints[0]['longitude'], waypoints[0]['altitude'])
                )
        
            self._log_info(f"Route '{route_name}' distances - Backward: {backward_distance:.2f}m, Forward: {forward_distance:.2f}m")


        # Create GeoPath based on shorter direction
        geopath = GeoPath()
        geopath.mode = GeoPath.ONCE
        
        if not loop_mode or backward_distance <= forward_distance:
            # Go backwards - add waypoints from closest to start in reverse
            self._log_info(f"Choosing backward path {'(Path is not looped)' if not loop_mode else f'(shorter by {forward_distance - backward_distance:.2f}m)'}")
            for i in range(closest_waypoint_idx, -1, -1):
                wp = waypoints[i]
                point = Point()
                point.x = float(wp['latitude'])
                point.y = float(wp['longitude'])
                point.z = float(wp.get('altitude', 0.0))
                geopath.waypoints.append(point)
        else:
            # Go forward - add waypoints from closest to end, then back to start
            self._log_info(f"Choosing forward path (shorter by {backward_distance - forward_distance:.2f}m)")
            for i in range(closest_waypoint_idx, num_waypoints):
                wp = waypoints[i]
                point = Point()
                point.x = float(wp['latitude'])
                point.y = float(wp['longitude'])
                point.z = float(wp.get('altitude', 0.0))
                geopath.waypoints.append(point)
            
            # Add first waypoint to close the loop and return home
            wp = waypoints[0]
            point = Point()
            point.x = float(wp['latitude'])
            point.y = float(wp['longitude'])
            point.z = float(wp.get('altitude', 0.0))
            geopath.waypoints.append(point)
        
        return geopath
    
    def validate_route_data(self, route_data: Dict) -> Tuple[bool, List[str]]:
        """Validate route data structure.
        
        Args:
            route_data: Dictionary containing route data
            
        Returns:
            Tuple of (is_valid, error_messages)
        """
        errors = []
        
        if not isinstance(route_data, dict):
            errors.append("Route data must be a dictionary")
            return False, errors
        
        # Check waypoints
        waypoints = route_data.get('waypoints', [])
        if not waypoints:
            errors.append("Route must contain at least one waypoint")
        elif not isinstance(waypoints, list):
            errors.append("Waypoints must be a list")
        else:
            for i, wp in enumerate(waypoints):
                if not isinstance(wp, dict):
                    errors.append(f"Waypoint {i} must be a dictionary")
                    continue
                
                if 'latitude' not in wp:
                    errors.append(f"Waypoint {i} missing 'latitude'")
                if 'longitude' not in wp:
                    errors.append(f"Waypoint {i} missing 'longitude'")
                
                # Validate coordinate ranges
                try:
                    lat = float(wp['latitude'])
                    if not -90 <= lat <= 90:
                        errors.append(f"Waypoint {i} latitude out of range (-90 to 90)")
                except (ValueError, TypeError):
                    errors.append(f"Waypoint {i} latitude must be a number")
                
                try:
                    lon = float(wp['longitude'])
                    if not -180 <= lon <= 180:
                        errors.append(f"Waypoint {i} longitude out of range (-180 to 180)")
                except (ValueError, TypeError):
                    errors.append(f"Waypoint {i} longitude must be a number")
        
        # Check loop_mode (optional)
        if 'loop_mode' in route_data:
            if not isinstance(route_data['loop_mode'], bool):
                errors.append("loop_mode must be a boolean")
        
        return len(errors) == 0, errors
    
    def get_first_waypoint(self, route_name: str) -> Optional[Tuple[float, float, float]]:
        """Get the first waypoint of a route.
        
        Args:
            route_name: Name of the route (without .yaml extension)
            
        Returns:
            Tuple of (latitude, longitude, altitude) or None if not found
        """
        route_data = self.load_route(route_name)
        if not route_data:
            return None
        
        waypoints = route_data.get('waypoints', [])
        if not waypoints:
            return None
        
        wp = waypoints[0]
        return (
            float(wp['latitude']),
            float(wp['longitude']),
            float(wp.get('altitude', 0.0))
        )
    
    def get_last_waypoint(self, route_name: str) -> Optional[Tuple[float, float, float]]:
        """Get the last waypoint of a route.
        
        Args:
            route_name: Name of the route (without .yaml extension)
            
        Returns:
            Tuple of (latitude, longitude, altitude) or None if not found
        """
        route_data = self.load_route(route_name)
        if not route_data:
            return None
        
        waypoints = route_data.get('waypoints', [])
        if not waypoints:
            return None
        
        wp = waypoints[-1]
        return (
            float(wp['latitude']),
            float(wp['longitude']),
            float(wp.get('altitude', 0.0))
        )
    
    def get_waypoint_count(self, route_name: str) -> int:
        """Get the number of waypoints in a route.
        
        Args:
            route_name: Name of the route (without .yaml extension)
            
        Returns:
            Number of waypoints, or 0 if route not found
        """
        route_data = self.load_route(route_name)
        if not route_data:
            return 0
        
        waypoints = route_data.get('waypoints', [])
        return len(waypoints)
    
    def find_closest_route(
        self,
        position: Tuple[float, float, float],
        distance_threshold: float = 1.0
    ) -> Tuple[Optional[str], float]:
        """Find the route closest to a given position.
        
        This method calculates the minimum perpendicular distance from the position
        to each route's path segments (lines between consecutive waypoints).
        Early stopping occurs if a route is found within the distance threshold.
        
        Args:
            position: Current position as (latitude, longitude, altitude)
            distance_threshold: Stop search if route found within this distance (meters)
            
        Returns:
            Tuple of (route_name, distance_in_meters) or (None, float('inf')) if no routes
        """
        self._log_debug(f"Finding closest route to position: {position} (threshold: {distance_threshold}m)")
        routes = self.list_routes()
        if not routes:
            self._log_warn("No routes available for closest route search")
            return None, float('inf')
        
        min_distance = float('inf')
        closest_route = None
        
        for route_name in routes:
            waypoints = self.get_waypoints(route_name)
            if not waypoints or len(waypoints) < 2:
                self._log_debug(f"Skipping route '{route_name}': insufficient waypoints ({len(waypoints) if waypoints else 0})")
                continue
            
            # Calculate minimum distance to any segment in this route
            route_min_distance = float('inf')
            
            for i in range(len(waypoints) - 1):
                wp1 = waypoints[i]
                wp2 = waypoints[i + 1]
                
                # Calculate distance to this segment
                distance = self._distance_to_segment(
                    position,
                    (wp1['latitude'], wp1['longitude'], wp1['altitude']),
                    (wp2['latitude'], wp2['longitude'], wp2['altitude'])
                )
                
                route_min_distance = min(route_min_distance, distance)
            
            self._log_debug(f"Route '{route_name}' min distance: {route_min_distance:.2f}m")
            
            # Update global minimum
            if route_min_distance < min_distance:
                min_distance = route_min_distance
                closest_route = route_name
                self._log_debug(f"New closest route: '{closest_route}' at {min_distance:.2f}m")
                
                # Early stopping if within threshold
                if min_distance <= distance_threshold:
                    self._log_info(f"Early stop: found route '{closest_route}' within threshold ({min_distance:.2f}m <= {distance_threshold}m)")
                    return closest_route, min_distance
        
        if closest_route:
            self._log_info(f"Closest route: '{closest_route}' at {min_distance:.2f}m")
        else:
            self._log_warn("No closest route found")
        
        return closest_route, min_distance
    
    def _distance_to_segment(
        self,
        point: Tuple[float, float, float],
        segment_start: Tuple[float, float, float],
        segment_end: Tuple[float, float, float]
    ) -> float:
        """Calculate minimum distance from point to line segment in meters.
        
        Uses Haversine formula for geodesic distances and vector projection
        to find perpendicular distance to segment.
        
        Args:
            point: Position as (lat, lon, alt)
            segment_start: Segment start as (lat, lon, alt)
            segment_end: Segment end as (lat, lon, alt)
            
        Returns:
            Distance in meters
        """
        # Calculate distances from point to segment endpoints
        dist_to_start = haversine_distance(point, segment_start)
        dist_to_end = haversine_distance(point, segment_end)
        
        # Calculate segment length
        segment_length = haversine_distance(segment_start, segment_end)
        
        # If segment has zero length, return distance to the point
        if segment_length < 1e-6:
            return dist_to_start
        
        # Calculate projection parameter t (where point projects onto segment)
        # t = 0 means projection is at start, t = 1 means at end
        # Using dot product approximation for small distances
        lat_diff_point = point[0] - segment_start[0]
        lon_diff_point = point[1] - segment_start[1]
        lat_diff_segment = segment_end[0] - segment_start[0]
        lon_diff_segment = segment_end[1] - segment_start[1]
        
        # Dot product divided by segment length squared
        t = (lat_diff_point * lat_diff_segment + lon_diff_point * lon_diff_segment) / \
            (lat_diff_segment * lat_diff_segment + lon_diff_segment * lon_diff_segment)
        
        # Clamp t to [0, 1] to stay within segment
        t = max(0.0, min(1.0, t))
        
        # Calculate closest point on segment
        closest_lat = segment_start[0] + t * lat_diff_segment
        closest_lon = segment_start[1] + t * lon_diff_segment
        closest_alt = segment_start[2] + t * (segment_end[2] - segment_start[2])
        
        # Return distance to closest point
        return haversine_distance(point, (closest_lat, closest_lon, closest_alt))
    
    def _find_closest_point_on_segment(
        self,
        point: Tuple[float, float, float],
        segment_start: Tuple[float, float, float],
        segment_end: Tuple[float, float, float]
    ) -> Tuple[float, float, float]:
        """Find the closest point on a line segment to a given point.
        
        Uses vector projection to find the perpendicular point on the segment.
        
        Args:
            point: Position as (lat, lon, alt)
            segment_start: Segment start as (lat, lon, alt)
            segment_end: Segment end as (lat, lon, alt)
            
        Returns:
            Closest point on segment as (lat, lon, alt)
        """
        # Calculate segment length
        segment_length = haversine_distance(segment_start, segment_end)
        
        # If segment has zero length, return start point
        if segment_length < 1e-6:
            return segment_start
        
        # Calculate projection parameter t
        lat_diff_point = point[0] - segment_start[0]
        lon_diff_point = point[1] - segment_start[1]
        lat_diff_segment = segment_end[0] - segment_start[0]
        lon_diff_segment = segment_end[1] - segment_start[1]
        
        # Dot product divided by segment length squared
        t = (lat_diff_point * lat_diff_segment + lon_diff_point * lon_diff_segment) / \
            (lat_diff_segment * lat_diff_segment + lon_diff_segment * lon_diff_segment)
        
        # Clamp t to [0, 1] to stay within segment
        t = max(0.0, min(1.0, t))
        
        # Calculate closest point on segment
        closest_lat = segment_start[0] + t * lat_diff_segment
        closest_lon = segment_start[1] + t * lon_diff_segment
        closest_alt = segment_start[2] + t * (segment_end[2] - segment_start[2])
        
        return (closest_lat, closest_lon, closest_alt)
    
    
    def _find_perpendicular_entry_point(
        self,
        route_name: str,
        position: Tuple[float, float, float]
    ) -> Optional[Tuple[Tuple[float, float, float], int]]:
        """Find the perpendicular entry point to rejoin a route.
        
        This calculates the closest point on any segment of the route,
        which represents the shortest perpendicular path to rejoin the route.
        
        Args:
            route_name: Name of the route
            position: Current position as (lat, lon, alt)
            
        Returns:
            Tuple of (entry_point, segment_index) where entry_point is (lat, lon, alt)
            and segment_index is the index of the closest segment, or None if route not found
        """
        waypoints = self.get_waypoints(route_name)
        if not waypoints or len(waypoints) < 2:
            self._log_warn(f"Cannot find entry point: route '{route_name}' has insufficient waypoints")
            return None
        
        min_distance = float('inf')
        closest_point = None
        closest_segment_idx = 0
        
        # Check each segment to find the closest perpendicular point
        for i in range(len(waypoints) - 1):
            wp1 = waypoints[i]
            wp2 = waypoints[i + 1]
            
            segment_start = (wp1['latitude'], wp1['longitude'], wp1['altitude'])
            segment_end = (wp2['latitude'], wp2['longitude'], wp2['altitude'])
            
            # Find closest point on this segment
            point_on_segment = self._find_closest_point_on_segment(
                position, segment_start, segment_end
            )
            
            # Calculate distance to this point
            distance = haversine_distance(position, point_on_segment)
            
            if distance < min_distance:
                min_distance = distance
                closest_point = point_on_segment
                closest_segment_idx = i
        
        if closest_point:
            self._log_info(
                f"Found perpendicular entry point at segment {closest_segment_idx}, "
                f"distance: {min_distance:.2f}m"
            )
            return (closest_point, closest_segment_idx)
        
        return None
    
    def find_shortest_way_home_with_perpendicular_entry(
        self,
        route_name: str,
        current_position: Tuple[float, float, float]
    ) -> Optional[GeoPath]:
        """Find shortest way home with perpendicular entry to route.
        
        This method creates an optimal home return path by:
        1. Finding the closest perpendicular point on the route
        2. Inserting it as the first waypoint (shortest path to rejoin route)
        3. Continuing with the optimal path logic (forward/backward)
        
        Args:
            route_name: Name of the route (without .yaml extension)
            current_position: Current robot position as (latitude, longitude, altitude)
            
        Returns:
            GeoPath with optimal waypoints including perpendicular entry point,
            or None if route not found
        """
        # Step 1: Find perpendicular entry point
        entry_result = self._find_perpendicular_entry_point(route_name, current_position)
        if not entry_result:
            self._log_warn(f"Could not find perpendicular entry point for route '{route_name}'")
            # Fall back to standard method without perpendicular entry
            return self.find_shortest_way_home(route_name, current_position)
        
        entry_point, segment_idx = entry_result
        
        # Step 2: Get optimal home path using standard logic
        geopath = self.find_shortest_way_home(route_name, current_position)
        if not geopath:
            return None
        
        # Step 3: Insert perpendicular entry point at the beginning
        entry_waypoint = Point()
        entry_waypoint.x = float(entry_point[0])  # latitude
        entry_waypoint.y = float(entry_point[1])  # longitude
        entry_waypoint.z = float(entry_point[2])  # altitude
        
        # Insert at the beginning of the waypoint list
        geopath.waypoints.insert(0, entry_waypoint)
        
        self._log_info(
            f"Created home return path with perpendicular entry for route '{route_name}' "
            f"({len(geopath.waypoints)} waypoints total)"
        )
        
        return geopath
