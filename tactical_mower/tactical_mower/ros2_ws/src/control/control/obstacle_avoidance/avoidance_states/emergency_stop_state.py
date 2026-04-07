"""EMERGENCY_STOP state implementation."""

import math
import time
from typing import Optional, Set, Tuple
import logging

from .base_avoidance_state import BaseAvoidanceState
from ..avoidance_state import AvoidanceState
from ..avoidance_context import (
    AvoidanceContext,
    has_any_blocked_sectors,
    has_blocked_stop_sectors,
    has_blocked_slow_sectors
)


class EmergencyStopState(BaseAvoidanceState):
    """EMERGENCY_STOP state: Critical situation < 0.3m, immediate stop.
    
    In this state, the robot stops immediately for initial_stop_duration seconds,
    then allows rotation (steering) while keeping speed at 0.
    """
    
    def __init__(
        self,
        state: AvoidanceState = AvoidanceState.EMERGENCY_STOP,
        initial_stop_duration: float = 3.0,
        logger: Optional[logging.Logger] = None
    ):
        """Initialize EMERGENCY_STOP state.
        
        Args:
            state: State enum value
            initial_stop_duration: Duration to fully stop (steering=0, speed=0) when entering state (s)
            logger: Optional logger
        """
        super().__init__(state, logger)
        self._initial_stop_duration = initial_stop_duration
        self._state_entry_time: Optional[float] = None
    
    def can_enter(self, context: AvoidanceContext) -> bool:
        """EMERGENCY_STOP can be entered when at least one STOP sector is blocked.
        
        Args:
            context: Current avoidance context
            
        Returns:
            True if at least one STOP sector is blocked
        """
        return has_blocked_stop_sectors(context)
    
    def can_exit_to(self, target_state: AvoidanceState, context: AvoidanceContext) -> bool:
        """Check if transition to target state is allowed.
        
        EMERGENCY_STOP can transition to:
        - FREE_DRIVE: When no sectors are blocked
        - SLOW_APPROACH: When only SLOW sectors are blocked (no STOP sectors)
        
        Args:
            target_state: Target state to transition to
            context: Current avoidance context
            
        Returns:
            True if transition is allowed
        """
        if target_state == AvoidanceState.FREE_DRIVE:
            return not has_any_blocked_sectors(context)
        
        if target_state == AvoidanceState.SLOW_APPROACH:
            return has_blocked_slow_sectors(context) and not has_blocked_stop_sectors(context)
        
        return False
    
    def get_valid_transitions(self, context: AvoidanceContext, enabled: bool = True) -> Set[AvoidanceState]:
        """Get valid transitions from EMERGENCY_STOP.
        
        Args:
            context: Current avoidance context
            enabled: Whether obstacle avoidance is enabled (default: True)
            
        Returns:
            Set of valid target states
        """
        valid = set()
        
        # FREE_DRIVE: no sectors blocked
        if not has_any_blocked_sectors(context):
            valid.add(AvoidanceState.FREE_DRIVE)
        # SLOW_APPROACH: only SLOW sectors blocked (no STOP sectors)
        elif has_blocked_slow_sectors(context) and not has_blocked_stop_sectors(context):
            valid.add(AvoidanceState.SLOW_APPROACH)
        
        return valid
    
    def on_enter(self, previous_state: Optional[AvoidanceState], context: AvoidanceContext):
        """Called when entering EMERGENCY_STOP.
        
        Args:
            previous_state: Previous state
            context: Current avoidance context
        """
        super().on_enter(previous_state, context)
        # Record entry time for initial stop duration
        self._state_entry_time = time.time()
        self._logger.warning(
            f"EMERGENCY_STOP: At least one STOP sector is blocked - stopping for {self._initial_stop_duration}s, then allowing rotation"
        )
    
    def _extract_heading(self, pose) -> float:
        """Extract heading (radians) from pose quaternion."""
        q = pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y**2 + q.z**2)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def _compute_steering_to_waypoint(
        self, 
        robot_pose, 
        target_waypoint, 
        max_steering: float = 100.0
    ) -> float:
        """Compute steering command to reach target waypoint from current pose.
        
        Args:
            robot_pose: Current robot pose
            target_waypoint: Target waypoint pose
            max_steering: Maximum steering command value
            
        Returns:
            Steering command in range [-max_steering, max_steering]
        """
        # Extract robot heading from quaternion
        heading = self._extract_heading(robot_pose)
        
        # Calculate angle to target
        dx = target_waypoint.pose.position.x - robot_pose.pose.position.x
        dy = target_waypoint.pose.position.y - robot_pose.pose.position.y
        angle_to_target = math.atan2(dy, dx)
        
        # Heading error (normalized to [-pi, pi])
        heading_error = self._normalize_angle(angle_to_target - heading)
        
        # Convert to steering command
        steering = max(
            -max_steering,
            min(max_steering, heading_error * 100 / math.pi)
        )
        
        return steering
    
    def on_update(self, context: AvoidanceContext) -> Tuple[Optional[float], Optional[float]]:
        """Stop robot immediately in EMERGENCY_STOP state.
        
        Behavior:
        1. For initial_stop_duration seconds: fully stop (steering=0, speed=0)
        2. After initial_stop_duration: allow rotation (steering from context, speed=0)
        
        Args:
            context: Current avoidance context
            
        Returns:
            (0.0, 0.0) during initial stop period, (steering, 0.0) after initial stop period
        """
        # Initialize entry time if not set (shouldn't happen, but safety check)
        if self._state_entry_time is None:
            self._state_entry_time = time.time()
        
        # Calculate time since entering EMERGENCY_STOP state
        elapsed_time = time.time() - self._state_entry_time
        
        # Phase 1: Full stop for initial_stop_duration seconds
        if elapsed_time < self._initial_stop_duration:
            # Fully stop: no steering, no speed
            return 0.0, 0.0
        
        # Phase 2: Allow rotation (steering allowed, speed=0)
        # Use original steering command to allow rotation towards waypoint
        steering = context.original_steering if context.original_steering is not None else 0.0
        return steering, 0.0

