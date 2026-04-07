"""Differential drive kinematics calculations."""

from typing import Tuple


class DifferentialDriveKinematics:
    """Kinematics calculations for differential drive robot.
    
    Converts joystick/steering commands to left/right wheel velocities.
    Supports both joystick mode (steer + speed) and pure velocity mode.
    """
    
    def __init__(self, wheel_diameter: float, wheel_base: float):
        """Initialize kinematics with robot parameters.
        
        Args:
            wheel_diameter: Wheel diameter in meters
            wheel_base: Distance between wheel centers in meters
        """
        self.wheel_diameter = wheel_diameter
        self.wheel_radius = wheel_diameter / 2.0
        self.wheel_base = wheel_base
    
    def calculate_wheel_velocities(
        self,
        steer: float,
        speed: float,
        max_speed: float
    ) -> Tuple[float, float]:
        """Calculate left and right wheel speeds from joystick input.
        
        Args:
            steer: Steering input [-100, 100], where negative = left, positive = right
            speed: Forward/backward speed input [-100, 100], where positive = forward
            max_speed: Maximum linear speed in m/s
            
        Returns:
            Tuple of (left_wheel_rad_s, right_wheel_rad_s) in radians per second
        """
        # Clamp inputs
        steer = max(-100.0, min(100.0, steer))
        speed = max(-100.0, min(100.0, speed))
        
        # Apply deadband to prevent oscillation when inputs are near zero
        DEADBAND = 2.0  # Ignore inputs smaller than 2% to prevent jitter
        if abs(steer) < DEADBAND:
            steer = 0.0
        if abs(speed) < DEADBAND:
            speed = 0.0
        
        # Convert input to linear velocity and angular velocity
        v = (speed / 100.0) * max_speed
        omega = (steer / 100.0) * (max_speed / (self.wheel_base / 2.0)) * 0.5
        
        # Differential drive kinematics
        # omega > 0 means turning right (counterclockwise), omega < 0 means turning left (clockwise)
        # When speed=0 and steer != 0: pure rotation in place
        # - omega > 0 (turn right): left wheel forward, right wheel backward
        # - omega < 0 (turn left): left wheel backward, right wheel forward
        v_left = v - omega * self.wheel_base / 2.0
        v_right = v + omega * self.wheel_base / 2.0
        
        # Convert to angular velocities (rad/s)
        left_rad_s = v_left / self.wheel_radius
        right_rad_s = v_right / self.wheel_radius
        
        return left_rad_s, right_rad_s
    
    def calculate_from_linear_angular(
        self,
        linear_velocity: float,
        angular_velocity: float
    ) -> Tuple[float, float]:
        """Calculate wheel velocities from linear and angular velocities.
        
        Args:
            linear_velocity: Linear velocity in m/s (positive = forward)
            angular_velocity: Angular velocity in rad/s (positive = counterclockwise)
            
        Returns:
            Tuple of (left_wheel_rad_s, right_wheel_rad_s) in radians per second
        """
        # Differential drive kinematics
        v_left = linear_velocity - angular_velocity * self.wheel_base / 2.0
        v_right = linear_velocity + angular_velocity * self.wheel_base / 2.0
        
        # Convert to angular velocities (rad/s)
        left_rad_s = v_left / self.wheel_radius
        right_rad_s = v_right / self.wheel_radius
        
        return left_rad_s, right_rad_s

