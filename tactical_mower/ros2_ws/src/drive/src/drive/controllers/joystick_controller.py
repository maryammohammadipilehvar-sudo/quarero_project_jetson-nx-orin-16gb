"""Joystick input processing and command generation."""

from typing import Optional, Tuple
from interfaces.msg import Joy
from ..kinematics.differential_drive import DifferentialDriveKinematics
from .speed_controller import SpeedController


class JoystickController:
    """Processes joystick input and generates drive commands.
    
    Handles joystick input scaling, steering inversion, and
    conversion to wheel velocity commands.
    """
    
    def __init__(
        self,
        kinematics: DifferentialDriveKinematics,
        speed_controller: SpeedController,
        flip_steering: bool = False
    ):
        """Initialize joystick controller.
        
        Args:
            kinematics: Differential drive kinematics instance
            speed_controller: Speed controller instance
            flip_steering: If True, invert steering direction
        """
        self.kinematics = kinematics
        self.speed_controller = speed_controller
        self.flip_steering = flip_steering
        self._prev_msg: Optional[Joy] = None
        self._last_steer: float = 0.0
        self._last_speed: float = 0.0
    
    def process_joystick(self, msg: Joy) -> Tuple[float, float]:
        """Process joystick message and generate wheel velocities.
        
        Args:
            msg: Joystick message
            
        Returns:
            Tuple of (left_vel_rad_s, right_vel_rad_s) in radians per second
        """
        # Extract joystick values
        steer_raw = float(msg.right_stick_right)
        speed_raw = float(msg.left_stick_forward)
        
        # Apply steering inversion BEFORE filtering to maintain consistency
        # This ensures the filter operates on values that will be passed to kinematics
        if self.flip_steering:
            steer_input = -steer_raw
        else:
            steer_input = steer_raw
        
        # Filter disabled for debugging - pass through directly
        # Filter can be re-enabled later if needed
        steer_filtered = steer_input
        speed_filtered = speed_raw
        
        # Store values for potential future filter use
        self._last_steer = steer_filtered
        self._last_speed = speed_filtered
        
        # Use filtered values directly (flip already applied if needed)
        steer = steer_filtered
        speed = speed_filtered
        
        # Get max speed from controller
        max_speed = self.speed_controller.get_max_speed()
        
        # Calculate wheel velocities
        left_rad_s, right_rad_s = self.kinematics.calculate_wheel_velocities(
            steer, speed, max_speed
        )
        
        # Store message for edge detection
        self._prev_msg = msg
        
        return left_rad_s, right_rad_s
    
    def is_rising_edge(self, current: bool, previous: Optional[bool]) -> bool:
        """Detect a rising edge of a boolean signal.
        
        Args:
            current: Current state of the input signal
            previous: Previous state of the input signal (None if not available)
            
        Returns:
            True if a rising edge is detected, False otherwise
        """
        if previous is None:
            return False
        return current and not previous
    
    def get_previous_message(self) -> Optional[Joy]:
        """Get previous joystick message.
        
        Returns:
            Previous joystick message or None
        """
        return self._prev_msg
    
    def reset(self):
        """Reset controller state."""
        self._prev_msg = None
        self._last_steer = 0.0
        self._last_speed = 0.0

