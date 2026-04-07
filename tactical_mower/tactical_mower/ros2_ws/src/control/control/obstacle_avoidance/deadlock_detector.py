"""Deadlock detection for obstacle avoidance."""

import time
from typing import Optional


class DeadlockDetector:
    """Detects deadlock situations where robot is stuck."""
    
    def __init__(
        self,
        timeout: float = 10.0,
        min_distance: float = 0.5
    ):
        """Initialize deadlock detector.
        
        Args:
            timeout: Time in seconds before considering deadlock (default: 10.0)
            min_distance: Minimum distance traveled to avoid deadlock (meters)
        """
        self.timeout = timeout
        self.min_distance = min_distance
        self._state_start_time: Optional[float] = None
        self._state_start_position: Optional[tuple] = None
        self._last_position: Optional[tuple] = None
        self._current_state_name: Optional[str] = None
    
    def reset(self):
        """Reset deadlock detection state."""
        self._state_start_time = None
        self._state_start_position = None
        self._last_position = None
        self._current_state_name = None
    
    def update_state(self, state_name: str, current_position: Optional[tuple] = None):
        """Update current state for deadlock detection.
        
        Args:
            state_name: Name of current avoidance state
            current_position: Current robot position (x, y) or None
        """
        now = time.time()
        
        # Check if state actually changed
        state_changed = (self._current_state_name != state_name)
        
        # If state changed, reset tracking (especially when entering EMERGENCY_STOP)
        if state_changed:
            # Reset timer when entering EMERGENCY_STOP state
            if state_name == 'EMERGENCY_STOP':
                self._state_start_time = now
                self._state_start_position = current_position
                self._last_position = current_position
            else:
                # When leaving EMERGENCY_STOP, clear the timer
                self._state_start_time = None
                self._state_start_position = None
        
        # Update current state name
        self._current_state_name = state_name
        
        # Update last position if provided
        if current_position is not None:
            self._last_position = current_position
    
    def check_deadlock(self, state_name: str, current_position: Optional[tuple] = None) -> bool:
        """Check if robot is in deadlock situation.
        
        Deadlock is detected when:
        - Robot is in EMERGENCY_STOP state
        - State has been active for more than timeout
        - Robot has moved less than min_distance
        
        Args:
            state_name: Name of current avoidance state
            current_position: Current robot position (x, y) or None
            
        Returns:
            True if deadlock detected, False otherwise
        """
        if state_name != 'EMERGENCY_STOP':
            return False
        
        if self._state_start_time is None or current_position is None:
            return False
        
        duration = time.time() - self._state_start_time
        if duration < self.timeout:
            return False
        
        # Check distance traveled
        if self._state_start_position is not None:
            dx = current_position[0] - self._state_start_position[0]
            dy = current_position[1] - self._state_start_position[1]
            distance_traveled = (dx**2 + dy**2)**0.5
            
            if distance_traveled < self.min_distance:
                return True
        
        return False

