"""Speed estimator for robot navigation using filtered sensor data."""

import time
from typing import Optional


class SpeedEstimator:
    """Estimates robot speed using filtered GPS/fusion data and commanded velocities.
    
    Implements an exponential moving average (EMA) filter to smooth noisy GPS velocity
    readings and provides sanity checking against commanded speeds.
    """
    
    def __init__(self, filter_alpha: float = 0.25, max_deviation: float = 1.5):
        """Initialize speed estimator.
        
        Args:
            filter_alpha: EMA filter coefficient (0-1). Higher = more responsive, lower = smoother
            max_deviation: Maximum allowed deviation (m/s) between GPS and commanded speed
        """
        self._filtered_speed_m_s = 0.0
        self._alpha = filter_alpha
        self._max_deviation = max_deviation
        self._last_update_time: Optional[float] = None
        self._commanded_speed_m_s: Optional[float] = None
        
    def update_from_gps(self, velocity_x: float, velocity_y: float, velocity_z: float = 0.0) -> float:
        """Update speed estimate from GPS/fusion velocity vector.
        
        Args:
            velocity_x: X velocity component in m/s (forward/backward)
            velocity_y: Y velocity component in m/s (left/right)
            velocity_z: Z velocity component in m/s (up/down) - usually ignored for ground robots
            
        Returns:
            Filtered speed estimate in m/s
        """
        # Calculate magnitude of velocity vector (2D planar speed)
        import math
        gps_speed_m_s = math.sqrt(velocity_x**2 + velocity_y**2)
        
        # Apply EMA filter
        if self._last_update_time is None:
            # First update - initialize with GPS value
            self._filtered_speed_m_s = gps_speed_m_s
        else:
            # EMA: filtered = alpha * new + (1 - alpha) * old
            self._filtered_speed_m_s = (
                self._alpha * gps_speed_m_s + 
                (1.0 - self._alpha) * self._filtered_speed_m_s
            )
        
        # Sanity check against commanded speed if available
        if self._commanded_speed_m_s is not None:
            deviation = abs(self._filtered_speed_m_s - self._commanded_speed_m_s)
            if deviation > self._max_deviation:
                # GPS seems way off - blend more heavily with commanded speed
                blend_weight = 0.7  # Favor commanded speed
                self._filtered_speed_m_s = (
                    blend_weight * self._commanded_speed_m_s + 
                    (1.0 - blend_weight) * self._filtered_speed_m_s
                )
        
        self._last_update_time = time.time()
        return self._filtered_speed_m_s
    
    def update_commanded_speed(self, linear_velocity_m_s: float):
        """Update the commanded speed for sanity checking.
        
        Args:
            linear_velocity_m_s: Commanded linear velocity in m/s
        """
        self._commanded_speed_m_s = abs(linear_velocity_m_s)
    
    def get_speed_m_s(self) -> float:
        """Get current speed estimate in m/s.
        
        Returns:
            Filtered speed in m/s
        """
        return self._filtered_speed_m_s
    
    def get_speed_kmh(self) -> float:
        """Get current speed estimate in km/h.
        
        Returns:
            Filtered speed in km/h
        """
        return self._filtered_speed_m_s * 3.6
    
    def reset(self):
        """Reset the estimator to initial state."""
        self._filtered_speed_m_s = 0.0
        self._last_update_time = None
        self._commanded_speed_m_s = None
