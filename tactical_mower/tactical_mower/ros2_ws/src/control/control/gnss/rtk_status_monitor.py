"""RTK status monitoring for GNSS positioning.

This module provides utilities to monitor RTK (Real-Time Kinematic) fix status
from Fixposition fusion system for precise positioning operations like docking.
"""

from enum import IntEnum
from typing import Optional, Tuple, Dict
from collections import deque
from dataclasses import dataclass
import time


class RTKStatus(IntEnum):
    """GNSS status codes from Fixposition system."""
    NO_FIX = 0
    SPP = 1          # Standard Point Positioning (meter-level accuracy)
    DGPS = 2         # Differential GPS
    PPS = 3          # Precise Point Positioning
    RTK_FIXED = 8    # RTK with fixed ambiguities (cm-level accuracy)
    RTK_FLOAT = 5    # RTK with float ambiguities (dm-level accuracy)
    ESTIMATED = 6    # Dead reckoning mode


@dataclass
class FusionState:
    """Container for fusion state data."""
    gnss1_status: Optional[int] = None
    gnss2_status: Optional[int] = None
    fusion_status: Optional[int] = None
    imu_status: Optional[int] = None
    timestamp: float = 0.0


class RTKStatusMonitor:
    """Monitor RTK fix status for precision positioning operations.
    
    This class tracks GNSS status from Fixposition fusion system and provides
    methods to verify RTK fix quality before critical operations like docking.
    
    Status codes:
        0 = No fix
        1 = SPP (Standard positioning, ~meter accuracy)
        5 = RTK Float (~decimeter accuracy)
        8 = RTK Fixed (~centimeter accuracy)
    
    Example:
        monitor = RTKStatusMonitor(require_both_gnss=True)
        monitor.update_fusion_state(fusion_dict)
        
        is_ready, reason = monitor.is_rtk_ready_for_docking()
        if not is_ready:
            print(f"Cannot dock: {reason}")
    """
    
    def __init__(
        self,
        require_both_gnss: bool = True,
        required_status: int = RTKStatus.RTK_FIXED,
        stability_window: int = 5,
        min_stable_readings: int = 3
    ):
        """Initialize RTK status monitor.
        
        Args:
            require_both_gnss: If True, both GNSS1 and GNSS2 must have RTK fix
            required_status: Minimum required GNSS status (default: RTK_FIXED=8)
            stability_window: Number of recent readings to keep for stability check
            min_stable_readings: Minimum consecutive good readings for stability
        """
        self._require_both_gnss = require_both_gnss
        self._required_status = required_status
        self._stability_window = stability_window
        self._min_stable_readings = min_stable_readings
        
        self._fusion_state: Optional[FusionState] = None
        self._status_history: deque = deque(maxlen=stability_window)
        self._last_update_time: float = 0.0
        
    def update_fusion_state(self, fusion_data: Dict) -> None:
        """Update fusion state from ROS message or dict.
        
        Args:
            fusion_data: Dictionary with keys 'gnss1_status', 'gnss2_status',
                        'fusion_status', 'imu_status'
        """
        current_time = time.time()
        
        self._fusion_state = FusionState(
            gnss1_status=fusion_data.get('gnss1_status'),
            gnss2_status=fusion_data.get('gnss2_status'),
            fusion_status=fusion_data.get('fusion_status'),
            imu_status=fusion_data.get('imu_status'),
            timestamp=current_time
        )
        
        self._last_update_time = current_time
        
        # Track status history for stability check
        gnss1_ok = self._fusion_state.gnss1_status == self._required_status
        gnss2_ok = self._fusion_state.gnss2_status == self._required_status
        
        if self._require_both_gnss:
            self._status_history.append(gnss1_ok and gnss2_ok)
        else:
            self._status_history.append(gnss1_ok or gnss2_ok)
    
    def is_rtk_fixed(self, require_both: Optional[bool] = None) -> Tuple[bool, str]:
        """Check if RTK status is at required level (default: Fixed).
        
        Args:
            require_both: Override constructor setting for this check
            
        Returns:
            Tuple of (is_fixed, reason_message)
        """
        if self._fusion_state is None:
            return False, "Kein Fusion-Status verfügbar"
        
        require_both = require_both if require_both is not None else self._require_both_gnss
        
        gnss1 = self._fusion_state.gnss1_status
        gnss2 = self._fusion_state.gnss2_status
        
        # Check data freshness (warn if older than 2 seconds)
        age = time.time() - self._last_update_time
        if age > 2.0:
            return False, f"Fusion-Daten veraltet ({age:.1f}s)"
        
        if require_both:
            if gnss1 != self._required_status:
                status_name = self._get_status_name(gnss1)
                return False, f"GNSS1 nicht RTK Fixed (Status: {status_name})"
            if gnss2 != self._required_status:
                status_name = self._get_status_name(gnss2)
                return False, f"GNSS2 nicht RTK Fixed (Status: {status_name})"
            return True, "RTK Fixed (beide GNSS)"
        else:
            # At least one must be at required status
            if gnss1 == self._required_status or gnss2 == self._required_status:
                which = []
                if gnss1 == self._required_status:
                    which.append("GNSS1")
                if gnss2 == self._required_status:
                    which.append("GNSS2")
                return True, f"RTK Fixed ({', '.join(which)})"
            
            gnss1_name = self._get_status_name(gnss1)
            gnss2_name = self._get_status_name(gnss2)
            return False, f"Kein GNSS mit RTK Fixed (GNSS1: {gnss1_name}, GNSS2: {gnss2_name})"
    
    def is_stable(self) -> Tuple[bool, str]:
        """Check if RTK status has been stable for minimum required readings.
        
        Returns:
            Tuple of (is_stable, reason_message)
        """
        if len(self._status_history) < self._min_stable_readings:
            return False, f"Zu wenig Messwerte ({len(self._status_history)}/{self._min_stable_readings})"
        
        # Count consecutive good readings from the end
        consecutive_good = 0
        for status_ok in reversed(self._status_history):
            if status_ok:
                consecutive_good += 1
            else:
                break
        
        if consecutive_good >= self._min_stable_readings:
            return True, f"Stabil ({consecutive_good}/{self._min_stable_readings} Messwerte)"
        else:
            return False, f"Instabil ({consecutive_good}/{self._min_stable_readings} konsekutive Messwerte)"
    
    def is_rtk_ready_for_docking(self) -> Tuple[bool, str]:
        """Combined check: RTK fixed AND stable.
        
        This is the main method to call before docking operations.
        
        Returns:
            Tuple of (is_ready, reason_message)
        """
        # First check RTK status
        is_fixed, fixed_reason = self.is_rtk_fixed()
        if not is_fixed:
            return False, fixed_reason
        
        # Then check stability
        is_stable, stable_reason = self.is_stable()
        if not is_stable:
            return False, f"RTK Fixed aber {stable_reason.lower()}"
        
        return True, f"RTK bereit für Docking ({fixed_reason}, {stable_reason.lower()})"
    
    def is_fusion_initialized(self) -> Tuple[bool, str]:
        """Check if fusion engine is initialized (green status).
        
        Fusion status should be GLOBAL_INIT (2) for best accuracy.
        
        Returns:
            Tuple of (is_initialized, reason_message)
        """
        if self._fusion_state is None:
            return False, "Kein Fusion-Status verfügbar"
        
        fusion_status = self._fusion_state.fusion_status
        
        # Check data freshness
        age = time.time() - self._last_update_time
        if age > 2.0:
            return False, f"Fusion-Daten veraltet ({age:.1f}s)"
        
        # GLOBAL_INIT = 2 means fusion is fully initialized (green)
        if fusion_status == 2:
            return True, "Fusion global initialisiert"
        elif fusion_status == 1:
            return False, "Fusion nur lokal initialisiert (Status: 1)"
        elif fusion_status == 0:
            return False, "Fusion nicht initialisiert (Status: 0)"
        else:
            return False, f"Fusion Status unbekannt (Status: {fusion_status})"
    
    def get_current_status(self) -> Dict:
        """Get current status as dictionary for logging/debugging.
        
        Returns:
            Dictionary with current GNSS status, stability info, and timestamps
        """
        if self._fusion_state is None:
            return {
                "available": False,
                "reason": "Kein Fusion-Status verfügbar"
            }
        
        is_fixed, fixed_reason = self.is_rtk_fixed()
        is_stable, stable_reason = self.is_stable()
        is_ready, ready_reason = self.is_rtk_ready_for_docking()
        
        age = time.time() - self._last_update_time
        
        return {
            "available": True,
            "gnss1_status": self._fusion_state.gnss1_status,
            "gnss1_status_name": self._get_status_name(self._fusion_state.gnss1_status),
            "gnss2_status": self._fusion_state.gnss2_status,
            "gnss2_status_name": self._get_status_name(self._fusion_state.gnss2_status),
            "fusion_status": self._fusion_state.fusion_status,
            "imu_status": self._fusion_state.imu_status,
            "is_fixed": is_fixed,
            "fixed_reason": fixed_reason,
            "is_stable": is_stable,
            "stable_reason": stable_reason,
            "is_ready_for_docking": is_ready,
            "ready_reason": ready_reason,
            "data_age_seconds": round(age, 2),
            "history_size": len(self._status_history),
            "require_both_gnss": self._require_both_gnss,
            "required_status": self._required_status
        }
    
    def reset_history(self) -> None:
        """Clear status history (e.g., after manual position change)."""
        self._status_history.clear()
    
    def set_required_status(self, status: int) -> None:
        """Change required status level.
        
        Args:
            status: New required status (e.g., RTKStatus.RTK_FLOAT for less strict check)
        """
        self._required_status = status
        self._status_history.clear()  # Reset history when changing requirements
    
    def _get_status_name(self, status: Optional[int]) -> str:
        """Get human-readable name for GNSS status code.
        
        Args:
            status: GNSS status code
            
        Returns:
            Human-readable status name
        """
        if status is None:
            return "Unbekannt"
        
        status_names = {
            0: "No Fix",
            1: "SPP",
            2: "DGPS",
            3: "PPS",
            5: "RTK Float",
            6: "Estimated",
            8: "RTK Fixed"
        }
        
        return status_names.get(status, f"Status {status}")
