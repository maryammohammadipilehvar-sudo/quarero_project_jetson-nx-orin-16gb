"""Mission state machine for managing mission execution state."""

from typing import Optional, Tuple


class MissionState:
    """Mission execution states."""
    IDLE = 0
    ACTIVE = 1
    RETURNING_HOME = 2
    CHARGING = 3
    ERROR = 4


class MissionStateMachine:
    """State machine for mission execution."""
    
    def __init__(self):
        """Initialize mission state machine."""
        self._state = MissionState.IDLE
        self._current_schedule_id: Optional[str] = None
        self._current_route_idx = 0
        self._current_route_repetition = 0

    def to_dict(self) -> dict:
        """Serialize mission state to a dict for persistence."""
        return {
            'state': int(self._state),
            'current_schedule_id': self._current_schedule_id,
            'current_route_idx': int(self._current_route_idx),
            'current_route_repetition': int(self._current_route_repetition)
        }

    @classmethod
    def from_dict(cls, data: dict) -> 'MissionStateMachine':
        """Create a MissionStateMachine from a serialized dict."""
        m = cls()
        try:
            m._state = int(data.get('state', MissionState.IDLE))
            m._current_schedule_id = data.get('current_schedule_id')
            m._current_route_idx = int(data.get('current_route_idx', 0))
            m._current_route_repetition = int(data.get('current_route_repetition', 0))
        except Exception:
            # If data is malformed, return a fresh machine
            return cls()
        return m
    
    def get_state(self) -> int:
        """Get current mission state.
        
        Returns:
            Current state (MissionState constant)
        """
        return self._state
    
    def is_active(self) -> bool:
        """Check if mission is active.
        
        Returns:
            True if mission is active, False otherwise
        """
        return self._state == MissionState.ACTIVE
    
    def start_mission(self, schedule_id: str, route_idx: int = 0, repetition: int = 0):
        """Start a new mission.
        
        Args:
            schedule_id: Schedule identifier
            route_idx: Starting route index
            repetition: Starting repetition number
        """
        self._state = MissionState.ACTIVE
        self._current_schedule_id = schedule_id
        self._current_route_idx = route_idx
        self._current_route_repetition = repetition
    
    def start_home_return(self):
        """Start home return sequence."""
        self._state = MissionState.RETURNING_HOME
    
    def start_charging(self):
        """Start charging sequence."""
        self._state = MissionState.CHARGING
    
    def stop_mission(self):
        """Stop current mission."""
        self._state = MissionState.IDLE
        self._current_schedule_id = None
        self._current_route_idx = 0
        self._current_route_repetition = 0
    
    def set_error(self):
        """Set error state."""
        self._state = MissionState.ERROR
    
    def get_current_schedule_id(self) -> Optional[str]:
        """Get current schedule ID.
        
        Returns:
            Current schedule ID or None
        """
        return self._current_schedule_id
    
    def get_current_route_info(self) -> Tuple[int, int]:
        """Get current route information.
        
        Returns:
            Tuple of (route_index, repetition)
        """
        return (self._current_route_idx, self._current_route_repetition)
    
    def set_current_route_info(self, route_idx: int, repetition: int):
        """Set current route information.
        
        Args:
            route_idx: Route index
            repetition: Repetition number
        """
        self._current_route_idx = route_idx
        self._current_route_repetition = repetition

