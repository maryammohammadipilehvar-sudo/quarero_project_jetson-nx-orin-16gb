"""Geographic calculation helpers for route distance and coordinate operations."""

import math
from typing import Tuple, List, Dict, Optional


def haversine_distance(
    pos1: Tuple[float, float, float],
    pos2: Tuple[float, float, float]
) -> float:
    """Calculate distance between two GPS coordinates using Haversine formula.
    
    Args:
        pos1: First position as (lat, lon, alt)
        pos2: Second position as (lat, lon, alt)
        
    Returns:
        Distance in meters
    """
    # Earth radius in meters
    R = 6371000.0
    
    # Convert to radians
    lat1, lon1 = math.radians(pos1[0]), math.radians(pos1[1])
    lat2, lon2 = math.radians(pos2[0]), math.radians(pos2[1])
    
    # Haversine formula
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    
    a = math.sin(dlat / 2) ** 2 + \
        math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    c = 2 * math.asin(math.sqrt(a))
    
    return R * c


def calculate_route_distance_from_waypoints(
    waypoints: List[Dict],
    mode: str
) -> float:
    """Calculate total distance of a route in meters from waypoint list.
    
    Args:
        waypoints: List of waypoint dicts with latitude, longitude, altitude
        mode: Route mode ('once', 'loop', 'ping_pong', 'none')
        
    Returns:
        Total distance in meters
    """
    if len(waypoints) < 2:
        return 0.0
    
    # Calculate forward distance (1->2->3->4)
    total_distance = 0.0
    for i in range(len(waypoints) - 1):
        wp1 = waypoints[i]
        wp2 = waypoints[i + 1]
        pos1 = (float(wp1['latitude']), float(wp1['longitude']), float(wp1.get('altitude', 0.0)))
        pos2 = (float(wp2['latitude']), float(wp2['longitude']), float(wp2.get('altitude', 0.0)))
        total_distance += haversine_distance(pos1, pos2)
    
    # For LOOP mode: add distance from last waypoint back to first (4->1)
    if mode == 'loop':
        last_wp = waypoints[-1]
        first_wp = waypoints[0]
        pos_last = (float(last_wp['latitude']), float(last_wp['longitude']), float(last_wp.get('altitude', 0.0)))
        pos_first = (float(first_wp['latitude']), float(first_wp['longitude']), float(first_wp.get('altitude', 0.0)))
        total_distance += haversine_distance(pos_last, pos_first)
    
    # For PING_PONG mode: add reverse distance (4->3->2->1)
    elif mode == 'ping_pong':
        for i in range(len(waypoints) - 1, 0, -1):
            wp1 = waypoints[i]
            wp2 = waypoints[i - 1]
            pos1 = (float(wp1['latitude']), float(wp1['longitude']), float(wp1.get('altitude', 0.0)))
            pos2 = (float(wp2['latitude']), float(wp2['longitude']), float(wp2.get('altitude', 0.0)))
            total_distance += haversine_distance(pos1, pos2)
    
    # For ONCE mode: only forward distance (already calculated)
    
    return total_distance

