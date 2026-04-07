"""Coordinate transformation utilities for GPS/ENU conversions."""

import math
from typing import Tuple


class CoordinateTransformer:
    """Handles coordinate transformations between GPS (LLH) and ENU frames.
    
    Uses a local origin to convert between GPS coordinates (latitude, longitude, altitude)
    and local ENU (East, North, Up) coordinates.
    """
    
    EARTH_RADIUS = 6378137.0  # Earth radius in meters
    
    def __init__(self, origin_lat: float = 0.0, origin_lon: float = 0.0, origin_alt: float = 0.0):
        """Initialize coordinate transformer with origin.
        
        Args:
            origin_lat: Origin latitude in degrees
            origin_lon: Origin longitude in degrees
            origin_alt: Origin altitude in meters
        """
        self._origin_lat = origin_lat
        self._origin_lon = origin_lon
        self._origin_alt = origin_alt
    
    def set_origin(self, lat: float, lon: float, alt: float = 0.0):
        """Set the local ENU origin.
        
        Args:
            lat: Origin latitude in degrees
            lon: Origin longitude in degrees
            alt: Origin altitude in meters
        """
        self._origin_lat = lat
        self._origin_lon = lon
        self._origin_alt = alt
    
    def get_origin(self) -> Tuple[float, float, float]:
        """Get the current origin.
        
        Returns:
            Tuple of (latitude, longitude, altitude)
        """
        return (self._origin_lat, self._origin_lon, self._origin_alt)
    
    def gps_to_enu(self, lat: float, lon: float, alt: float) -> Tuple[float, float, float]:
        """Convert GPS coordinates to local ENU coordinates.
        
        Uses flat-earth approximation, accurate for small distances (< 10km).
        
        Args:
            lat: Latitude in degrees
            lon: Longitude in degrees
            alt: Altitude in meters
            
        Returns:
            Tuple of (east, north, up) in meters
        """
        # Convert to radians
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        origin_lat_rad = math.radians(self._origin_lat)
        origin_lon_rad = math.radians(self._origin_lon)
        
        # Differences
        dlat = lat_rad - origin_lat_rad
        dlon = lon_rad - origin_lon_rad
        
        # ENU coordinates (flat-earth approximation)
        x = self.EARTH_RADIUS * dlon * math.cos(origin_lat_rad)  # East
        y = self.EARTH_RADIUS * dlat  # North
        z = alt - self._origin_alt  # Up
        
        return x, y, z
    
    def compute_distance(
        self,
        pos1: Tuple[float, float, float],
        pos2: Tuple[float, float, float]
    ) -> float:
        """Compute haversine distance between two GPS coordinates.
        
        Args:
            pos1: First position as (latitude, longitude, altitude)
            pos2: Second position as (latitude, longitude, altitude)
            
        Returns:
            Distance in meters (horizontal distance, altitude ignored)
        """
        lat1, lon1, _ = pos1
        lat2, lon2, _ = pos2
        
        # Convert to radians
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        
        # Haversine formula
        a = (math.sin(dlat / 2) ** 2 +
             math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2) ** 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        
        return self.EARTH_RADIUS * c

