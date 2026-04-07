"""Runtime estimation for battery-powered robots."""

from collections import deque
from typing import Optional
import time


class RuntimeEstimator:
    """Estimates remaining runtime based on power consumption history.
    
    Uses moving averages and pessimistic estimation strategies to provide
    conservative runtime estimates when the robot is moving.
    """
    
    def __init__(
        self,
        capacity_ah: float,
        safety_margin: float = 0.10,
        short_term_window: float = 30.0,
        long_term_window: float = 120.0,
        max_history_size: int = 1000
    ):
        """Initialize runtime estimator.
        
        Args:
            capacity_ah: Battery capacity in Ah
            safety_margin: Safety margin factor (0.15 = 15% reduction)
            short_term_window: Short-term average window in seconds
            long_term_window: Long-term average window in seconds
            max_history_size: Maximum number of samples to keep in history
        """
        self.capacity_ah = capacity_ah
        self.safety_margin = safety_margin
        self.short_term_window = short_term_window
        self.long_term_window = long_term_window
        self.max_history_size = max_history_size
        
        # Power consumption history: list of (timestamp, power_watts, current_amps)
        self._power_history: deque = deque(maxlen=max_history_size)
        
        # Track if robot is moving
        self._is_moving = False
        self._last_movement_time: Optional[float] = None
        self._movement_timeout = 5.0  # Consider stopped if no movement for 5 seconds
    
    def update_power_consumption(self, voltage: float, current: float, timestamp: Optional[float] = None):
        """Update power consumption history.
        
        Args:
            voltage: Battery voltage in Volts
            current: Total current draw in Amperes
            timestamp: Timestamp in seconds (uses current time if None)
        """
        if timestamp is None:
            timestamp = time.time()
        
        power = voltage * current  # Watts
        
        self._power_history.append((timestamp, power, current))
    
    def set_moving(self, is_moving: bool, timestamp: Optional[float] = None):
        """Update robot movement state.
        
        Args:
            is_moving: True if robot is currently moving
            timestamp: Timestamp in seconds (uses current time if None)
        """
        if timestamp is None:
            timestamp = time.time()
        
        self._is_moving = is_moving
        if is_moving:
            self._last_movement_time = timestamp
    
    def is_robot_moving(self, current_time: Optional[float] = None) -> bool:
        """Check if robot is considered moving.
        
        Args:
            current_time: Current timestamp (uses current time if None)
            
        Returns:
            True if robot is moving or was moving recently
        """
        if current_time is None:
            current_time = time.time()
        
        if self._is_moving:
            return True
        
        if self._last_movement_time is not None:
            time_since_movement = current_time - self._last_movement_time
            return time_since_movement < self._movement_timeout
        
        return False
    
    def _get_average_current(self, window_seconds: float, current_time: Optional[float] = None) -> Optional[float]:
        """Calculate average current over a time window.
        
        Args:
            window_seconds: Time window in seconds
            current_time: Current timestamp (uses current time if None)
            
        Returns:
            Average current in Amperes, or None if insufficient data
        """
        if current_time is None:
            current_time = time.time()
        
        if len(self._power_history) == 0:
            return None
        
        cutoff_time = current_time - window_seconds
        samples = [item for item in self._power_history if item[0] >= cutoff_time]
        
        if len(samples) == 0:
            return None
        
        total_current = sum(item[2] for item in samples)
        return total_current / len(samples)
    
    def _get_peak_current(self, window_seconds: float, current_time: Optional[float] = None) -> Optional[float]:
        """Get peak current over a time window.
        
        Args:
            window_seconds: Time window in seconds
            current_time: Current timestamp (uses current time if None)
            
        Returns:
            Peak current in Amperes, or None if insufficient data
        """
        if current_time is None:
            current_time = time.time()
        
        if len(self._power_history) == 0:
            return None
        
        cutoff_time = current_time - window_seconds
        samples = [item for item in self._power_history if item[0] >= cutoff_time]
        
        if len(samples) == 0:
            return None
        
        return max(item[2] for item in samples)
    
    def estimate_runtime(
        self,
        soc: float,
        current_time: Optional[float] = None
    ) -> Optional[float]:
        """Estimate remaining runtime in seconds.
        
        Uses pessimistic estimation: takes the maximum of short-term and long-term
        averages, adds safety margin, and uses peak current as a worst-case scenario.
        
        Args:
            soc: State of Charge (0.0 to 1.0)
            current_time: Current timestamp (uses current time if None)
            
        Returns:
            Estimated runtime in seconds, or None if insufficient data
        """
        if current_time is None:
            current_time = time.time()
        
        # Calculate remaining capacity
        remaining_capacity_ah = soc * self.capacity_ah
        
        if remaining_capacity_ah <= 0:
            return 0.0
        
        # Get current estimates
        short_term_avg = self._get_average_current(self.short_term_window, current_time)
        long_term_avg = self._get_average_current(self.long_term_window, current_time)
        peak_current = self._get_peak_current(self.short_term_window, current_time)
        
        # If no data available, return None
        if short_term_avg is None and long_term_avg is None:
            return None
        
        # Use the maximum of available averages (pessimistic)
        current_estimates = [avg for avg in [short_term_avg, long_term_avg] if avg is not None]
        if not current_estimates:
            return None
        
        # Take maximum average (pessimistic approach)
        avg_current = max(current_estimates)
        
        # Also consider peak current as worst-case scenario
        if peak_current is not None:
            # Use weighted average: 70% of max average, 30% of peak
            pessimistic_current = 0.7 * avg_current + 0.3 * peak_current
        else:
            pessimistic_current = avg_current
        
        # Apply safety margin
        pessimistic_current *= (1.0 + self.safety_margin)
        
        # Calculate runtime: remaining capacity / current draw
        if pessimistic_current <= 0:
            return None
        
        runtime_hours = remaining_capacity_ah / pessimistic_current
        runtime_seconds = runtime_hours * 3600.0
        
        return runtime_seconds
    
    def clear_history(self):
        """Clear power consumption history."""
        self._power_history.clear()
    
    def get_recent_average_power(self, window_seconds: float = 30.0, current_time: Optional[float] = None) -> Optional[float]:
        """Get recent average power consumption.
        
        Args:
            window_seconds: Time window in seconds
            current_time: Current timestamp (uses current time if None)
            
        Returns:
            Average power in Watts, or None if insufficient data
        """
        if current_time is None:
            current_time = time.time()
        
        if len(self._power_history) == 0:
            return None
        
        cutoff_time = current_time - window_seconds
        samples = [item for item in self._power_history if item[0] >= cutoff_time]
        
        if len(samples) == 0:
            return None
        
        total_power = sum(item[1] for item in samples)
        return total_power / len(samples)

