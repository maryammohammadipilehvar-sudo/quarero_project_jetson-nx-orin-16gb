"""Emergency stop controller for robot safety."""

from typing import Tuple


class EmergencyStopController:
    """Manages emergency stop state and commands.
    
    Provides state machine for normal operation and emergency stop.
    """
    
    def __init__(self):
        """Initialize emergency stop controller."""
        self._active = False
    
    def activate(self):
        """Activate emergency stop (stop all motion)."""
        self._active = True
    
    def deactivate(self):
        """Deactivate emergency stop (resume normal operation)."""
        self._active = False
    
    def is_active(self) -> bool:
        """Check if emergency stop is active.
        
        Returns:
            True if emergency stop is active, False otherwise
        """
        return self._active
    
    def get_stop_command(self) -> Tuple[float, float]:
        """Get stop command (zero velocities).
        
        Returns:
            Tuple of (left_vel, right_vel) = (0.0, 0.0)
        """
        return 0.0, 0.0

