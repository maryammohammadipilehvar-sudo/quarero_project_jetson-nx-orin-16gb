"""Robot state context for state machine decisions."""

from dataclasses import dataclass


@dataclass
class RobotStateContext:
    """Context information for state machine decisions.
    
    This dataclass contains all the information needed by states to make
    transition decisions and execute their behavior.
    """
    # Position information
    charge_pos_set: bool = False
    is_at_charge_pos: bool = False  # Precise tolerance (5cm) for charging
    is_near_charge_pos: bool = False  # Larger tolerance (40cm) for state recovery after MANUAL
    is_at_home_pos: bool = False  # Normal tolerance from settings (default 50cm)
    is_near_home_pos: bool = False  # Larger tolerance (60cm) for undocking completion
    
    # System status
    need_charge: bool = False
    route_active: bool = False
    schedule_active: bool = False
    rtk_fix: bool = False
    sensor_info_valid: bool = False
    docked: bool = False  # Physical docking state
    autonomous_operation_enabled: bool = False  # Autonomous mode must be enabled via web app
    
    # Charging state
    charging_relay_enabled: bool = False

