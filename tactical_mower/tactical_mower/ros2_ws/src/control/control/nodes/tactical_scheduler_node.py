#!/usr/bin/env python3
"""ROS2 node for tactical schedule management and mission coordination.

This node orchestrates scheduling components to manage route execution,
home return, and charging operations.
"""

import json
import yaml
import time
import math
import random
import rclpy
from rclpy.node import Node
from pathlib import Path
from typing import Optional, Dict, Tuple
from datetime import datetime
import pytz

from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point
from interfaces.msg import Schedule, GeoPath
from interfaces.srv import CommandControl, ScheduleService, GetScheduleAction
from sensor_msgs.msg import NavSatFix
from fixposition_driver_msgs.msg import FusionEpoch

from ..navigation.coordinate_transformer import CoordinateTransformer
from ..scheduling.schedule_manager import ScheduleManager
from ..scheduling.schedule_validator import ScheduleValidator
from ..scheduling.time_window_checker import TimeWindowChecker
from ..home_return.home_return_controller import HomeReturnController
from ..mission.mission_state_machine import MissionState, MissionStateMachine
from ..gnss.rtk_status_monitor import RTKStatusMonitor, RTKStatus
from ..routes.route_manager import RouteManager


class TacticalSchedulerNode(Node):
    """ROS2 node for schedule management and mission coordination."""

    def __init__(self):
        """Initialize the TacticalSchedulerNode."""
        super().__init__('tactical_scheduler')

        # Parameters
        self.declare_parameter('schedules_file', '/routen/settings/schedules.yaml')
        self.declare_parameter('routes_dir', '/routen/routes')
        self.declare_parameter('settings_file', '/routen/settings/settings.yaml')
        self.declare_parameter('check_interval', 2.0)
        self.declare_parameter('default_tolerance_home_check', 1.0)

        schedules_file = Path(self.get_parameter('schedules_file').value).expanduser()
        configured_routes_dir = Path(self.get_parameter('routes_dir').value).expanduser()
        settings_file = Path(self.get_parameter('settings_file').value).expanduser()
        check_interval = self.get_parameter('check_interval').value
        tolerance_home = self.get_parameter('default_tolerance_home_check').value
        
        # Prefer an existing routes directory to avoid mismatches between containers
        candidate_dirs = [
            configured_routes_dir,
            Path("/routen/routes"),
            Path("/data/routes"),
        ]
        self._routes_dir = next((p for p in candidate_dirs if p.exists()), configured_routes_dir)
        self.get_logger().info(f"Using routes directory: {self._routes_dir}")
        
        # Initialize components
        self._coord_transformer = CoordinateTransformer()
        self._route_manager = RouteManager(self._routes_dir, logger=self.get_logger())
        self._schedule_manager = ScheduleManager(schedules_file, self._route_manager)
        self._time_checker = TimeWindowChecker()
        self._home_return = HomeReturnController(
            self._coord_transformer,
            self._route_manager,
            tolerance=tolerance_home
        )
        self._mission_state = MissionStateMachine()
        self._rtk_monitor = RTKStatusMonitor(
            require_both_gnss=True,
            stability_window=5,
            min_stable_readings=3
        )

        # Settings
        self._settings_file = settings_file
        self._settings = {}
        self._default_battery_threshold = 20
        self._default_home_tolerance = 0.0
        self._default_auto_charge_return = True
        self._home_position: Optional[Tuple[float, float, float]] = None
        self._charge_position: Optional[Tuple[float, float, float]] = None
        self._reload_settings()

        # Charging state will be determined from robot state machine via topic
        self._last_robot_state: Optional[str] = None
        self._resume_check_timer = None
        self._resume_check_attempts = 0

        # Schedules dictionary
        self._schedules: Dict[str, Dict] = {}

        # Robot state
        self._battery_level: Optional[int] = None
        self._robot_position: Optional[Tuple[float, float, float]] = None
        self._autonomous_operation_enabled: bool = False  # Track autonomous mode state

        # Publishers
        self._ack_pub = self.create_publisher(Bool, '/tactical/control/schedule/acknowledge', 10)
        self._charging_dock_pub = self.create_publisher(Point, '/tactical/control/charging/dock', 10)
        self._charging_undock_pub = self.create_publisher(Bool, '/tactical/control/charging/undock', 10)
        self._charging_enable_pub = self.create_publisher(Bool, '/control/enable_charging', 10)
        self._charging_requested_pub = self.create_publisher(Bool, '/tactical/control/charging/requested', 10)
        self._log_info_pub = self.create_publisher(String, '/tactical/logging/info', 10)
        self._log_warn_pub = self.create_publisher(String, '/tactical/logging/warn', 10)
        self._log_error_pub = self.create_publisher(String, '/tactical/logging/error', 10)

        # Services
        self.create_service(CommandControl, '/control/go_to_charge_pos_and_charge', self._go_to_charge_service)
        self.create_service(CommandControl, '/control/charge_manual', self._charge_manual_service)
        self.create_service(ScheduleService, '/tactical/control/schedule', self._schedule_service)
        self.create_service(GetScheduleAction, '/control/get_schedule_action', self._get_schedule_action_service)
        
        # Subscribers (keep for backward compatibility and other sources)
        self.create_subscription(String, '/robot/state', self._robot_status_callback, 10)
        self.create_subscription(NavSatFix, '/fixposition/odometry_llh', self._gps_callback, 10)
        self.create_subscription(Bool, '/tactical/robot/route_completed', self._route_completed_callback, 10)
        self.create_subscription(String, '/tactical/robot/charging_status', self._charging_status_callback, 10)
        self.create_subscription(FusionEpoch, '/fixposition/fusion', self._fusion_callback, 10)
        
        # Subscribe to autonomous operation state changes
        self.create_subscription(Bool, '/control/autonomous_operation', self._autonomous_operation_callback, 10)

        # Load schedules
        self._schedules = self._schedule_manager.load_all()
        for schedule_id, schedule in self._schedules.items():
            schedule_display = self._get_schedule_display_name(schedule, schedule_id)
            self._log_info(f"📅 Zeitplan '{schedule_display}' geladen")

        # Timer for schedule checking
        self._schedule_timer = self.create_timer(check_interval, self._check_schedules)

        # Home monitoring timer
        self._home_monitor_timer = None
        self._charging_pending = False
        
        # Schedule overwrite tracking (for battery threshold)
        self._overwritten_schedule_id: Optional[str] = None  # Just track which schedule was interrupted
        self._is_undocking = False  # Track if undocking is in progress
        self._charging_relay_enabled = False  # Track actual relay state (single source of truth)
        self._unified_charging_state = False  # Unified charging state from /robot/state
        
        # Schedule that needs to start after undocking completes (when schedule starts while docked)
        self._pending_schedule_after_undock: Optional[str] = None
        
        # Spam guard: prevents re-publishing charge return request every status tick
        self._charge_return_triggered = False
        
        # Temporary storage for home return route info (for button press case)
        self._home_return_schedule_id: Optional[str] = None
        self._home_return_route_idx: Optional[int] = None
        
        # Battery monitoring during charging
        self._target_charge_soc = 80  # Target SoC before resuming schedule
        self._min_battery_threshold = 20  # Minimum battery threshold for low battery lock
        
        # Low battery lock - prevents new missions until battery reaches target SoC
        self._low_battery_lock = False  # Set when battery drops below threshold during mission
        
        # Undock timer
        self._undock_timer = None
        
        # Track last charging status to avoid duplicate logs
        self._last_charging_status: Optional[str] = None
        
        # Schedule state for service-based architecture
        # The scheduler no longer publishes GeoPath directly - control loop queries for schedule data
        self._route_ready_for_pickup = False  # True when a route is ready to be fetched by control loop

        self._timezone = pytz.timezone('Europe/Berlin')

        self.get_logger().info("TacticalSchedulerNode initialized")

        # Subscribe to robot state machine to determine charging state
        self.create_subscription(String, '/tactical/robot/state', self._robot_state_machine_callback, 10)

    def _reload_settings(self):
        """Reload settings from YAML file."""
        if not self._settings_file.exists():
            self.get_logger().warning(f"Settings file not found: {self._settings_file}")
            self._settings = {}
        else:
            try:
                with open(self._settings_file, 'r') as f:
                    self._settings = yaml.safe_load(f) or {}
            except Exception as e:
                self.get_logger().error(f"Failed to load settings: {e}")
                self._settings = {}
        
        self._default_battery_threshold = self._settings.get('battery_threshold', 20)
        self._default_home_tolerance = self._settings.get('home_tolerance', 0.0)
        self._default_auto_charge_return = self._settings.get('auto_charge_return', True)
        self._max_route_distance = float(self._settings.get('max_route_distance', 3.0))
        
        home_pos = self._settings.get('home_point')
        if home_pos and 'latitude' in home_pos and 'longitude' in home_pos:
            self._home_position = (home_pos['latitude'], home_pos['longitude'], home_pos.get('yaw', 0.0))
        
        charge_pos = self._settings.get('charge_point')
        if charge_pos and 'latitude' in charge_pos and 'longitude' in charge_pos:
            self._charge_position = (charge_pos['latitude'], charge_pos['longitude'], charge_pos.get('yaw', 0.0))
        # detection radius for considering robot at charge position (meters)
        # Increased to 0.5m to account for GPS inaccuracy when docked
        # Home position is 2m behind charge point, so 0.5m won't cause false positives
        charge_radius = float(self._settings.get('charge_detection_radius', 0.5))
        self._charge_detection_radius = min(charge_radius, 1.0)  # Cap at 1.0 m maximum

    def _format_schedule_time_window(self, schedule: Dict) -> str:
        """Format schedule time window as user-friendly string.
        
        Args:
            schedule: Schedule dictionary with weekdays, start_time, end_time
            
        Returns:
            Formatted string like "Mo-Fr 08:00-12:00" or "Mo, Mi, Fr 14:00-18:00"
        """
        weekdays = schedule.get('weekdays', [])
        start_time = schedule.get('start_time', '')
        end_time = schedule.get('end_time', '')
        
        if not weekdays or not start_time or not end_time:
            return "Zeitplan"
        
        # Map weekday numbers to German abbreviations
        weekday_names = ['Mo', 'Di', 'Mi', 'Do', 'Fr', 'Sa', 'So']
        
        # Convert weekday numbers to names
        weekday_strs = [weekday_names[wd] for wd in weekdays if 0 <= wd < 7]
        
        if not weekday_strs:
            return f"{start_time}-{end_time}"
        
        # Format weekdays
        if len(weekday_strs) == 7:
            weekday_display = "täglich"
        elif len(weekday_strs) == 5 and set(weekday_strs) == {'Mo', 'Di', 'Mi', 'Do', 'Fr'}:
            weekday_display = "Mo-Fr"
        elif len(weekday_strs) == 2 and weekday_strs == ['Sa', 'So']:
            weekday_display = "Sa-So"
        else:
            weekday_display = ", ".join(weekday_strs)
        
        return f"{weekday_display} {start_time}-{end_time}"
    
    def _get_schedule_display_name(self, schedule: Optional[Dict], schedule_id: Optional[str] = None) -> str:
        """Get user-friendly display name for a schedule.
        
        Args:
            schedule: Schedule dictionary (optional, will be looked up if not provided)
            schedule_id: Schedule ID (optional, used if schedule is None)
            
        Returns:
            Formatted string like "Mo-Fr 08:00-12:00" or "Zeitplan (ID: ...)" if schedule not found
        """
        if schedule is None and schedule_id:
            schedule = self._schedules.get(schedule_id)
        
        if schedule:
            return self._format_schedule_time_window(schedule)
        elif schedule_id:
            return f"Zeitplan (ID: {schedule_id[:8]}...)"  # Show first 8 chars of ID as fallback
        else:
            return "Zeitplan"
    
    def _find_similar_schedule(self, new_schedule: Dict) -> Optional[Tuple[str, Dict]]:
        """Find a similar schedule that differs only in schedule_id and active status.
        
        This is used to detect when a schedule is being toggled (activated/deactivated)
        rather than being newly created.
        
        Args:
            new_schedule: The new schedule dictionary to compare
            
        Returns:
            Tuple of (schedule_id, schedule_dict) if a similar schedule is found, None otherwise
        """
        # Fields to compare (excluding schedule_id and active)
        compare_fields = ['weekdays', 'start_time', 'end_time', 'route_names', 
                         'route_repetitions', 'route_mode', 'loop_mode', 
                         'require_home_return', 'battery_threshold', 'home_tolerance', 
                         'auto_charge_return']
        
        for schedule_id, existing_schedule in self._schedules.items():
            # Skip if same schedule_id (would be an update, not a toggle)
            if schedule_id == new_schedule.get('schedule_id'):
                continue
            
            # Compare all relevant fields
            is_similar = all(
                existing_schedule.get(field) == new_schedule.get(field)
                for field in compare_fields
            )
            
            if is_similar:
                return (schedule_id, existing_schedule)
        
        return None

    def _log_info(self, text: str):
        """Publish info log."""
        msg = String()
        msg.data = text
        self._log_info_pub.publish(msg)
        self.get_logger().info(f"Sent: {text}")

    def _log_warn(self, text: str):
        """Publish warn log."""
        msg = String()
        msg.data = text
        self._log_warn_pub.publish(msg)
        self.get_logger().warning(f"Sent: {text}")

    def _log_error(self, text: str):
        """Publish error log."""
        msg = String()
        msg.data = text
        self._log_error_pub.publish(msg)
        self.get_logger().error(f"Sent: {text}")

    def _publish_ack(self, success: bool):
        """Publish acknowledgment."""
        msg = Bool()
        msg.data = success
        self._ack_pub.publish(msg)

    def _schedule_service(self, request, response):
        """Handle schedule service (add or remove)."""
        command_id = request.command_id
        action = request.action
        
        self.get_logger().info(f"Schedule service called: action={action} (command_id: {command_id})")
        
        if action == "add":
            # Handle schedule addition
            msg = request.schedule
            self.get_logger().info(f"Adding schedule: {msg.schedule_id}")

            if len(msg.route_names) != len(msg.route_repetitions):
                self._log_error("Fehler: Anzahl der Routen und Wiederholungen stimmt nicht überein.")
                response.success = False
                response.message = "Route names and repetitions count mismatch"
                return response

            schedule_data = {
                'schedule_id': msg.schedule_id,
                'weekdays': list(msg.weekdays),
                'start_time': msg.start_time,
                'end_time': msg.end_time,
                'route_names': list(msg.route_names),
                'route_repetitions': list(msg.route_repetitions),
                'route_mode': msg.route_mode,
                'loop_mode': msg.loop_mode,
                'active': msg.active,
                'require_home_return': msg.require_home_return,
                'battery_threshold': msg.battery_threshold if msg.battery_threshold > 0 else self._default_battery_threshold,
                'home_tolerance': msg.home_tolerance if msg.home_tolerance > 0 else self._default_home_tolerance,
                'auto_charge_return': msg.auto_charge_return,
                'done': False  # Reset done flag when schedule is added/updated
            }

            # Validate using schedule manager's validator
            validator = ScheduleValidator(self._route_manager)
            is_valid, errors = validator.validate(schedule_data)
            if not is_valid:
                self.get_logger().error(f"Invalid schedule: {errors}")
                schedule_display = self._format_schedule_time_window(schedule_data)
                error_msg = ', '.join(errors) if isinstance(errors, list) else str(errors)
                self._log_error(f"❌ Ungültiger Zeitplan '{schedule_display}': {error_msg}")
                response.success = False
                response.message = f"Invalid schedule: {errors}"
                return response

            # Check if this is a toggle (activation/deactivation) of an existing schedule
            # Toggles should now use "update" action, but we keep this for backward compatibility
            similar_schedule = self._find_similar_schedule(schedule_data)
            is_toggle = similar_schedule is not None
            
            # Save
            self._schedules[msg.schedule_id] = schedule_data
            if self._schedule_manager.save(schedule_data):
                schedule_display = self._get_schedule_display_name(schedule_data)
                
                if is_toggle:
                    # This is a toggle - log activation/deactivation instead of save
                    if msg.active:
                        self._log_info(f"✅ Zeitplan '{schedule_display}' aktiviert")
                    else:
                        self._log_info(f"⏸️ Zeitplan '{schedule_display}' deaktiviert")
                else:
                    # This is a new schedule or update
                    self._log_info(f"✅ Zeitplan '{schedule_display}' erfolgreich gespeichert")
                
                response.success = True
                response.message = f"Schedule '{msg.schedule_id}' saved successfully"
            else:
                self._log_error("❌ Fehler beim Speichern des Zeitplans")
                response.success = False
                response.message = "Failed to save schedule"
            return response
            
        elif action == "remove":
            # Handle schedule removal
            schedule_id = request.schedule_id
            
            try:
                # Check if schedule exists
                schedule_exists = schedule_id in self._schedules
                
                if schedule_exists:
                    self.get_logger().info(f"Removing schedule: {schedule_id}")
                else:
                    self.get_logger().info(f"Removing schedule: {schedule_id} (not found in memory, checking disk)")

                # Get schedule object if it exists (for display name)
                schedule_obj = self._schedules.get(schedule_id) if schedule_exists else None
                
                # Stop mission if this schedule is currently active
                if self._mission_state.get_current_schedule_id() == schedule_id:
                    schedule_display = self._get_schedule_display_name(schedule=schedule_obj, schedule_id=schedule_id)
                    self._log_warn(f"⏹️ Aktive Mission '{schedule_display}' wird gestoppt")
                    self._stop_current_mission()

                # Remove from memory
                if schedule_exists:
                    schedule_display = self._get_schedule_display_name(schedule=schedule_obj, schedule_id=schedule_id)
                    del self._schedules[schedule_id]
                else:
                    schedule_display = self._get_schedule_display_name(schedule=None, schedule_id=schedule_id)

                # Try to delete file (may not exist if using different storage format)
                file_deleted = self._schedule_manager.delete(schedule_id)
                
                # Success if schedule was removed from memory OR file was deleted
                # (File might not exist if web app uses different storage format)
                if schedule_exists or file_deleted:
                    # Log deletion (toggles now use "update" action, so this is a real deletion)
                    self._log_info(f"🗑️ Zeitplan '{schedule_display}' erfolgreich gelöscht")
                    response.success = True
                    response.message = f"Schedule '{schedule_id}' deleted successfully"
                else:
                    # Schedule not found in memory and file doesn't exist
                    self.get_logger().info(f"Schedule '{schedule_id}' not found (already deleted or never existed)")
                    response.success = True  # Consider it successful - already deleted or never existed
                    response.message = f"Schedule '{schedule_id}' not found (may already be deleted)"
                return response
            except Exception as e:
                self.get_logger().error(f"Error removing schedule: {e}")
                schedule_display = self._get_schedule_display_name(schedule=None, schedule_id=schedule_id)
                self._log_error(f"❌ Fehler beim Löschen des Zeitplans '{schedule_display}': {str(e)}")
                response.success = False
                response.message = f"Error: {str(e)}"
                return response
                
        elif action == "update":
            # Handle schedule update (typically for activation/deactivation toggle)
            # This is a cleaner approach than remove + add
            msg = request.schedule
            old_schedule_id = request.schedule_id
            self.get_logger().info(f"Updating schedule: {old_schedule_id} -> {msg.schedule_id}")
            
            try:
                # Check if old schedule exists
                old_schedule_exists = old_schedule_id in self._schedules
                old_schedule = self._schedules.get(old_schedule_id) if old_schedule_exists else None
                
                if len(msg.route_names) != len(msg.route_repetitions):
                    self._log_error("Fehler: Anzahl der Routen und Wiederholungen stimmt nicht überein.")
                    response.success = False
                    response.message = "Route names and repetitions count mismatch"
                    return response

                schedule_data = {
                    'schedule_id': msg.schedule_id,
                    'weekdays': list(msg.weekdays),
                    'start_time': msg.start_time,
                    'end_time': msg.end_time,
                    'route_names': list(msg.route_names),
                    'route_repetitions': list(msg.route_repetitions),
                    'route_mode': msg.route_mode,
                    'loop_mode': msg.loop_mode,
                    'active': msg.active,
                    'require_home_return': msg.require_home_return,
                    'battery_threshold': msg.battery_threshold if msg.battery_threshold > 0 else self._default_battery_threshold,
                    'home_tolerance': msg.home_tolerance if msg.home_tolerance > 0 else self._default_home_tolerance,
                    'auto_charge_return': msg.auto_charge_return,
                    'done': False  # Reset done flag when schedule is updated
                }

                # Validate using schedule manager's validator
                validator = ScheduleValidator(self._route_manager)
                is_valid, errors = validator.validate(schedule_data)
                if not is_valid:
                    self.get_logger().error(f"Invalid schedule: {errors}")
                    schedule_display = self._format_schedule_time_window(schedule_data)
                    error_msg = ', '.join(errors) if isinstance(errors, list) else str(errors)
                    self._log_error(f"❌ Ungültiger Zeitplan '{schedule_display}': {error_msg}")
                    response.success = False
                    response.message = f"Invalid schedule: {errors}"
                    return response

                # Check if this is an activation/deactivation toggle
                is_toggle = False
                old_active = None
                if old_schedule_exists and old_schedule:
                    # Compare all fields except schedule_id and active status
                    compare_fields = ['weekdays', 'start_time', 'end_time', 'route_names', 
                                     'route_repetitions', 'route_mode', 'loop_mode', 
                                     'require_home_return', 'battery_threshold', 'home_tolerance', 
                                     'auto_charge_return']
                    is_toggle = all(old_schedule.get(field) == schedule_data.get(field) for field in compare_fields)
                    if is_toggle:
                        old_active = old_schedule.get('active', False)

                # Stop mission if old schedule is currently active and schedule_id changed
                if old_schedule_id != msg.schedule_id and self._mission_state.get_current_schedule_id() == old_schedule_id:
                    schedule_display = self._get_schedule_display_name(schedule=old_schedule, schedule_id=old_schedule_id)
                    self._log_warn(f"⏹️ Aktive Mission '{schedule_display}' wird gestoppt")
                    self._stop_current_mission()

                # Remove old schedule if schedule_id changed
                if old_schedule_exists and old_schedule_id != msg.schedule_id:
                    del self._schedules[old_schedule_id]
                    # Also delete old file
                    self._schedule_manager.delete(old_schedule_id)

                # Save new/updated schedule
                self._schedules[msg.schedule_id] = schedule_data
                if self._schedule_manager.save(schedule_data):
                    schedule_display = self._get_schedule_display_name(schedule_data)
                    
                    if is_toggle and old_active is not None:
                        # This is a toggle - log activation/deactivation
                        if msg.active and not old_active:
                            self._log_info(f"✅ Zeitplan '{schedule_display}' aktiviert")
                        elif not msg.active and old_active:
                            self._log_info(f"⏸️ Zeitplan '{schedule_display}' deaktiviert")
                            # If this schedule was pending to start after undock, cancel it
                            if self._pending_schedule_after_undock == msg.schedule_id:
                                self.get_logger().info(f"Cancelling pending schedule '{msg.schedule_id}' due to deactivation")
                                self._pending_schedule_after_undock = None
                                # Cancel undock timer if active
                                if self._undock_timer:
                                    self._undock_timer.cancel()
                                    self._undock_timer = None
                        else:
                            # Active status didn't change, just update
                            self._log_info(f"✅ Zeitplan '{schedule_display}' erfolgreich aktualisiert")
                    else:
                        # This is a regular update (not a toggle)
                        self._log_info(f"✅ Zeitplan '{schedule_display}' erfolgreich aktualisiert")
                    
                    response.success = True
                    response.message = f"Schedule '{msg.schedule_id}' updated successfully"
                else:
                    self._log_error("❌ Fehler beim Aktualisieren des Zeitplans")
                    response.success = False
                    response.message = "Failed to update schedule"
                return response
            except Exception as e:
                self.get_logger().error(f"Error updating schedule: {e}")
                schedule_display = self._get_schedule_display_name(schedule=None, schedule_id=old_schedule_id)
                self._log_error(f"❌ Fehler beim Aktualisieren des Zeitplans '{schedule_display}': {str(e)}")
                response.success = False
                response.message = f"Error: {str(e)}"
                return response
        else:
            response.success = False
            response.message = f"Unknown action: {action}"
            return response

    def _autonomous_operation_callback(self, msg: Bool):
        """Handle autonomous operation state changes."""
        self._autonomous_operation_enabled = msg.data
        if msg.data:
            self.get_logger().info("Autonomous operation enabled - schedules can be executed")
        else:
            self.get_logger().info("Autonomous operation disabled - schedules will not be executed")
    
    def _robot_status_callback(self, msg: String):
        """Handle robot status updates."""
        try:
            data = json.loads(msg.data)
            self._battery_level = float(data.get("battery_percentage", 0))
            # Update unified charging state from robot_controller_node
            prev_charging_state = self._unified_charging_state
            self._unified_charging_state = bool(data.get("charging_state", False))

            # Detect charging start transition (False → True)
            if self._unified_charging_state and not prev_charging_state:
                if self._charge_return_triggered:
                    self._log_info("🔋 Automatischer Ladevorgang gestartet (Rückkehr wegen niedrigem Batteriestand).")
                else:
                    self._log_info("🔌 Manueller Ladevorgang gestartet.")

            # self.get_logger().info(f"Received battery level: {self._battery_level}%")

            # Check if we're charging and need to monitor SoC for low battery lock
            if self._unified_charging_state:
                # Clear low battery lock when target SoC is reached (but continue charging)
                if self._low_battery_lock and self._battery_level >= self._target_charge_soc:
                    self._low_battery_lock = False
                    self._log_info(f"Batteriestand {self._battery_level}% erreicht - Low-Battery-Lock aufgehoben. Ladevorgang läuft weiter bis 100%.")
                
                # Check if we have a pending schedule waiting for sufficient battery
                if self._pending_schedule_after_undock and self._overwritten_schedule_id:
                    if self._battery_level >= self._target_charge_soc:
                        # Only undock if autonomous operation is enabled
                        if not self._autonomous_operation_enabled:
                            self.get_logger().info(f"Battery sufficient, but autonomous mode is disabled - mission will not resume.")
                        else:
                            self._log_info(f"Batteriestand ausreichend ({self._battery_level}% >= {self._target_charge_soc}%). Starte Undocking für wartende Mission.")
                            # Stop charging and start undocking
                            self._enable_charging_relay(False)
                            self._overwritten_schedule_id = None
                            
                            undock_msg = Bool()
                            undock_msg.data = True
                            self._charging_undock_pub.publish(undock_msg)
                            # Set undocking flag immediately to prevent stale state messages from interfering
                            self._is_undocking = True
                            
                            if self._undock_timer:
                                self._undock_timer.cancel()
                            self._undock_timer = self.create_timer(3.0, self._on_undock_complete_once)
                
                # Charging continues until a new schedule triggers or manual undock is requested

            # Battery check fires for BOTH manual and scheduled missions
            # Uses robot FSM state (NAVIGATING) instead of internal mission_state
            # so it works regardless of how the mission was started
            if self._last_robot_state == 'NAVIGATING':
                schedule_id = self._mission_state.get_current_schedule_id() if self._mission_state.is_active() else None
                schedule = self._schedules.get(schedule_id) if schedule_id else None
                threshold = schedule.get('battery_threshold', self._default_battery_threshold) if schedule else self._default_battery_threshold
                auto_charge = schedule.get('auto_charge_return', self._default_auto_charge_return) if schedule else self._default_auto_charge_return
                if self._battery_level is not None and self._battery_level < threshold:
                    if self._battery_level < self._min_battery_threshold:
                        self._low_battery_lock = True
                        self._log_warn(f"Kritischer Batteriestand ({self._battery_level}%) - Low-Battery-Lock aktiviert.")
                    if auto_charge:
                        if not self._charge_return_triggered:
                            self._charge_return_triggered = True
                            if schedule_id:
                                self._overwritten_schedule_id = schedule_id
                            self._log_warn("Batteriestand kritisch – Rückkehr zur Basis eingeleitet.")
                            charging_request_msg = Bool()
                            charging_request_msg.data = True
                            self._charging_requested_pub.publish(charging_request_msg)
                            self._log_info("Ladevorgang angefordert - State Machine entscheidet.")
        except Exception:
            pass

    def _gps_callback(self, msg: NavSatFix):
        """Store GPS position."""
        if msg.latitude != 0 and msg.longitude != 0:
            self._robot_position = (msg.latitude, msg.longitude, msg.altitude)

    def _fusion_callback(self, msg: FusionEpoch):
        """Handle fusion status updates for RTK monitoring."""
        try:
            if not getattr(msg, "fpa_odomstatus_avail", False):
                return
            
            status = msg.fpa_odomstatus
            fusion_data = {
                'gnss1_status': int(getattr(status, "gnss1_status", -1)),
                'gnss2_status': int(getattr(status, "gnss2_status", -1)),
                'fusion_status': int(getattr(status, "init_status", -1)),
                'imu_status': int(getattr(status, "imu_status", -1))
            }
            self._rtk_monitor.update_fusion_state(fusion_data)
        except Exception as e:
            self.get_logger().error(f"Fusion callback error: {e}")

    def _route_completed_callback(self, msg: Bool):
        """Handle route completion."""
        if not msg.data or not self._mission_state.is_active():
            self.get_logger().info("Route completion received but no active mission.")
            return

        if self._mission_state.get_state() == MissionState.RETURNING_HOME:
            self.get_logger().info("Already returning home.")
            return

        schedule_id = self._mission_state.get_current_schedule_id()
        schedule = self._schedules.get(schedule_id)
        if not schedule:
            self.get_logger().info(f"Schedule with ID {schedule_id} not found.")
            return

        route_idx, repetition = self._mission_state.get_current_route_info()
        route_repetitions = schedule.get('route_repetitions', [])
        
        # Validate route_idx
        if route_idx < 0 or route_idx >= len(route_repetitions):
            self._log_error(f"Invalid route index {route_idx} for schedule with {len(route_repetitions)} routes.")
            self._stop_current_mission()
            return
        
        required_repetitions = route_repetitions[route_idx]
        repetition += 1

        # Repeat if needed
        if repetition < required_repetitions:
            self._mission_state.set_current_route_info(route_idx, repetition)
            route_name = schedule['route_names'][route_idx]
            self._log_info(f"🔄 Route '{route_name}' - Wiederholung {repetition + 1}/{required_repetitions}")
            # Reload the same route for repetition
            self._load_and_publish_next_route()
            return

        # All repetitions done, move to next route
        route_name = schedule['route_names'][route_idx]
        self._log_info(f"✅ Route '{route_name}' abgeschlossen ({required_repetitions} Wiederholungen). Wechsel zur nächsten Route.")
        repetition = 0
        route_mode = schedule.get('route_mode', 0)
        total_routes = len(schedule.get('route_names', []))

        if route_mode == 1:  # Random
            # Pick a random route (can be any route, including the one just completed)
            # This allows all routes to be executed multiple times if needed
            previous_route_idx = route_idx
            if total_routes > 1:
                # Pick a random route
                route_idx = random.randrange(total_routes)
                self.get_logger().info(f"Random mode: completed route {previous_route_idx}, selecting route {route_idx}")
            else:
                route_idx = 0
        else:  # Sequential
            route_idx += 1
            # Check if we've completed all routes
            if route_idx >= total_routes:
                # All routes completed - check if we should loop
                if schedule.get('loop_mode', False):
                    route_idx = 0
                    schedule_display = self._get_schedule_display_name(schedule, schedule_id)
                    self._log_info(f"✅ Alle Routen abgeschlossen. Starte von vorne (Schleifenmodus aktiv) - '{schedule_display}'")
                else:
                    # No loop mode - mission complete
                    schedule_display = self._get_schedule_display_name(schedule, schedule_id)
                    self._log_info(f"✅ Alle Routen abgeschlossen - '{schedule_display}'")
                    # Stop mission and mark as done - control loop will decide what to do next
                    self._mission_state.stop_mission()
                    schedule['done'] = True
                    self._schedule_manager.save(schedule)
                    return  # Don't load next route

        self._mission_state.set_current_route_info(route_idx, repetition)
        self._load_and_publish_next_route()

    def _charging_status_callback(self, msg: String):
        """Handle charging status."""
        try:
            parts = msg.data.split('|', 1)
            if len(parts) == 2:
                state, message = parts
                
                # Only log if status changed to avoid spam
                status_changed = state != self._last_charging_status
                self._last_charging_status = state
                
                if state == "docked":
                    # Reset undocking flag - if we're docked, we're definitely not undocking
                    if self._is_undocking:
                        self.get_logger().info("Received 'docked' status - resetting _is_undocking flag")
                        self._is_undocking = False
                    if status_changed:
                        self._log_info("🔌 Roboter erfolgreich angedockt")
                    # Don't enable charging relay here - it should only be enabled
                    # when the robot transitions to CHARGING state, not just DOCKED.
                    # The state machine will handle the DOCKED -> CHARGING transition.
                elif state == "undocking":
                    # Undocking started - disable charging relay (only log once)
                    if status_changed:
                        self._log_info("🚛 Undocking gestartet - Ladevorgang wird gestoppt")
                        self._enable_charging_relay(False)
                        self._is_undocking = True
                elif state == "completed":
                    # "completed" means undocking is complete, not charging
                    if status_changed:
                        self._log_info("✅ Undocking abgeschlossen")
                    self._is_undocking = False
                    # Trigger schedule start if we have a pending schedule
                    if self._pending_schedule_after_undock:
                        # Cancel timer if it exists and trigger immediately
                        if self._undock_timer:
                            self._undock_timer.cancel()
                            self._undock_timer = None
                        # Start schedule immediately since undocking is complete
                        self._on_undock_complete_once()
                elif state == "error":
                    if status_changed:
                        self._log_error(f"Ladefehler: {message}")
                    self._enable_charging_relay(False)
                    self._handle_charging_error()
                    self._is_undocking = False
        except Exception as e:
            self.get_logger().error(f"Error parsing charging status: {e}")

    def _go_to_charge_service(self, request, response):
        """Handle go to charge position service."""
        command_id = request.command_id
        state = request.state
        
        self.get_logger().info(f"Go to charge service called: {state} (command_id: {command_id})")
        
        if state:
            # Button press: save route info before deactivating (needed for home return)
            if self._mission_state.is_active():
                schedule_id = self._mission_state.get_current_schedule_id()
                if schedule_id:
                    route_idx, _ = self._mission_state.get_current_route_info()
                    self._home_return_schedule_id = schedule_id
                    self._home_return_route_idx = route_idx
            # Deactivate ALL active schedules
            self._deactivate_all_active_schedules()
            # Publish charging request to topic - State Machine will handle it
            charging_request_msg = Bool()
            charging_request_msg.data = True
            self._charging_requested_pub.publish(charging_request_msg)
            # Charging is handled by state machine via charging request topic
            self._log_info("Ladevorgang angefordert - State Machine entscheidet.")
            response.message = "Return to charging station started"
        else:
            # Clear charging request
            charging_request_msg = Bool()
            charging_request_msg.data = False
            self._charging_requested_pub.publish(charging_request_msg)
            self._log_info("Rückkehr zur Ladestation abgebrochen.")
            self._abort_charging_sequence()
            response.message = "Return to charging station aborted"
        
        response.success = True
        return response

    def _charge_manual_service(self, request, response):
        """Handle manual charge relay enable/disable service."""
        command_id = request.command_id
        state = request.state
        
        self.get_logger().info(f"Charge manual service called: {state} (command_id: {command_id})")
        
        self._enable_charging_relay(state)
        if state:
            self._log_info("🔋 Manueller Ladevorgang gestartet")
            response.message = "Manual charging started (relay enabled)"
        else:
            self._log_info("🔋 Manueller Ladevorgang gestoppt")
            response.message = "Manual charging stopped (relay disabled)"
        
        response.success = True
        return response

    def _check_schedules(self):
        """Check schedule state and update flags.
        
        This method no longer directly executes schedules or publishes GeoPath.
        The control loop queries via /control/get_schedule_action service to get
        the appropriate action and route data.
        
        This method only:
        1. Resets 'done' flags for schedules outside time window
        2. Checks if active mission exceeded time window (sets pending action)
        3. Updates internal state for service queries
        """
        now = datetime.now(self._timezone)
        current_time = now.time()
        current_weekday = now.weekday()
        
        # Reset 'done' flag for schedules outside their time window
        for schedule_id, schedule in self._schedules.items():
            if schedule.get('done', False):
                # Check if schedule is outside time window
                if not self._time_checker.is_schedule_active(schedule, current_weekday, current_time):
                    schedule['done'] = False
                    if self._schedule_manager.save(schedule):
                        schedule_display = self._get_schedule_display_name(schedule, schedule_id)
                        self.get_logger().info(f"Mission '{schedule_display}' done-Flag zurückgesetzt (außerhalb Zeitfenster)")
                    else:
                        schedule_display = self._get_schedule_display_name(schedule, schedule_id)
                        self.get_logger().error(f"Fehler beim Zurücksetzen des done-Flags für Mission '{schedule_display}'")

        # Check if current mission exceeded time window
        if self._mission_state.is_active():
            schedule_id = self._mission_state.get_current_schedule_id()
            schedule = self._schedules.get(schedule_id)
            if schedule:
                start_time = datetime.strptime(schedule['start_time'], '%H:%M').time()
                end_time = datetime.strptime(schedule['end_time'], '%H:%M').time()
                if not self._time_checker.is_in_window(current_time, start_time, end_time):
                    if self._mission_state.get_state() != MissionState.RETURNING_HOME:
                        schedule_display = self._get_schedule_display_name(schedule, schedule_id)
                        self._log_warn(f"⏰ Zeitfenster für Mission '{schedule_display}' beendet")
                        # Stop mission - control loop will detect no active schedule and decide what to do
                        self._mission_state.stop_mission()
                        # Mark schedule as done for today
                        schedule['done'] = True
                        self._schedule_manager.save(schedule)
        
        # Schedule execution is now handled by _get_schedule_action_service
        # The control loop queries for the next action instead of scheduler pushing commands

    def _get_schedule_action_service(self, request, response):
        """Service handler to query active schedule data.
        
        This service ONLY provides schedule/route DATA.
        NO robot state information is used.
        NO decisions are made - control loop makes ALL decisions.
        
        Args:
            request: GetScheduleAction request (only command_id for logging)
            response: GetScheduleAction response with schedule data
            
        Returns:
            Response with schedule/route data only
        """
        self.get_logger().debug(f"Schedule query (command_id: {request.command_id})")
        
        # Initialize response - ONLY data, no decisions
        response.success = True
        response.has_active_schedule = False
        response.schedule_id = ""
        response.schedule_display_name = ""
        response.route_name = ""
        response.waypoints = []
        response.geopath_mode = 0
        response.message = ""
        response.current_route_index = 0
        response.total_routes = 0
        response.current_repetition = 0
        response.total_repetitions = 0
        
        # Check if there's an active mission with next route ready (continuation)
        if self._mission_state.is_active():
            schedule_id = self._mission_state.get_current_schedule_id()
            schedule = self._schedules.get(schedule_id)
            if schedule and self._route_ready_for_pickup:
                response.has_active_schedule = True
                return self._fill_route_response(response, schedule_id, schedule)
        
        # Check for active schedules in time window
        now = datetime.now(self._timezone)
        current_time = now.time()
        current_weekday = now.weekday()
        
        for schedule_id, schedule in self._schedules.items():
            if not self._time_checker.is_schedule_active(schedule, current_weekday, current_time):
                continue
            if schedule.get('done', False):
                continue
            if not schedule.get('active', False):
                continue
                
            # Found an active schedule - provide the data
            response.has_active_schedule = True
            schedule_display = self._get_schedule_display_name(schedule, schedule_id)
            response.schedule_display_name = schedule_display
            response.schedule_id = schedule_id
            
            # Check low battery lock (this is data, not a decision)
            if self._low_battery_lock:
                battery_status = f"{self._battery_level}%" if self._battery_level is not None else "unbekannt"
                response.message = f"Low-Battery-Lock aktiv (Batterie: {battery_status}, benötigt: {self._target_charge_soc}%)"
                # Still report has_active_schedule=True so control loop knows there's a schedule
                return response
            
            # Start mission state tracking if not already started
            if not self._mission_state.is_active() or self._mission_state.get_current_schedule_id() != schedule_id:
                route_mode = schedule.get('route_mode', 0)
                route_idx = random.randrange(len(schedule['route_names'])) if route_mode == 1 else 0
                self._mission_state.start_mission(schedule_id, route_idx, 0)
                self._charge_return_triggered = False  # Re-arm for this new mission
            
            # Fill route data - control loop decides what to do with it
            return self._fill_route_response(response, schedule_id, schedule)
        
        # No active schedule found
        response.message = "Kein aktiver Zeitplan"
        return response
    
    def _fill_route_response(self, response, schedule_id: str, schedule: Dict):
        """Fill response with route data for current mission state.
        
        This method only provides route DATA - no action decisions.
        The control loop decides what to do with this data.
        
        Args:
            response: GetScheduleAction response to fill
            schedule_id: Schedule ID
            schedule: Schedule dictionary
            
        Returns:
            Filled response with route data
        """
        route_idx, repetition = self._mission_state.get_current_route_info()
        route_names = schedule.get('route_names', [])
        route_repetitions = schedule.get('route_repetitions', [])
        
        if route_idx < 0 or route_idx >= len(route_names):
            response.success = False
            response.message = f"Ungültiger Route-Index {route_idx}"
            return response
        
        route_name = route_names[route_idx]
        response.route_name = route_name
        response.schedule_id = schedule_id
        response.schedule_display_name = self._get_schedule_display_name(schedule, schedule_id)
        response.current_route_index = route_idx
        response.total_routes = len(route_names)
        response.current_repetition = repetition
        response.total_repetitions = route_repetitions[route_idx] if route_idx < len(route_repetitions) else 1
        
        # Create GeoPath for the route - provide DATA only
        try:
            geopath = self._route_manager.create_geopath(route_name)
            if geopath and len(geopath.waypoints) > 0:
                response.waypoints = list(geopath.waypoints)
                response.geopath_mode = geopath.mode
                response.message = f"Route '{route_name}' ({len(geopath.waypoints)} waypoints)"
                # Clear the pickup flag
                self._route_ready_for_pickup = False
            else:
                response.success = False
                response.message = f"Route '{route_name}' nicht gefunden oder leer"
        except Exception as e:
            response.success = False
            response.message = f"Fehler beim Laden der Route '{route_name}': {str(e)}"
            self.get_logger().error(f"Error loading route: {e}")
        
        return response
    
    def _notify_route_ready(self, schedule_id: str, route_idx: int):
        """Mark that a route is ready to be picked up by control loop.
        
        Instead of publishing GeoPath directly, this sets a flag that
        the control loop will see when it queries the service.
        
        Args:
            schedule_id: Schedule ID
            route_idx: Route index in schedule
        """
        self._route_ready_for_pickup = True
        self.get_logger().info(f"Route ready for pickup: schedule={schedule_id}, route_idx={route_idx}")

    def _execute_schedule(self, schedule_id: str):
        """Start executing a schedule."""
        # Check if autonomous operation is enabled
        schedule = self._schedules.get(schedule_id)
        schedule_display = self._get_schedule_display_name(schedule, schedule_id)
        
        if not self._autonomous_operation_enabled:
            self.get_logger().info(f"Mission '{schedule_display}' cannot start because autonomous mode is disabled.")
            return
        
        if not schedule:
            self._log_error(f"❌ Mission '{schedule_display}' kann nicht gestartet werden - Zeitplan nicht gefunden")
            return
        
        # Check if schedule is already done
        if schedule.get('done', False):
            self.get_logger().info(f"Mission '{schedule_display}' bereits abgeschlossen - wird übersprungen")
            return

        # Check low battery lock - prevent new missions if battery was critically low
        if self._low_battery_lock:
            battery_status = f"{self._battery_level}%" if self._battery_level is not None else "unbekannt"
            self._log_warn(f"⚠️ Mission '{schedule_display}' kann nicht gestartet werden: Low-Battery-Lock aktiv (Batterie: {battery_status}, benötigt: {self._target_charge_soc}%)")
            return

        # If we already have a pending schedule after undock, don't trigger another undocking
        if self._pending_schedule_after_undock:
            self.get_logger().info(f"Schedule '{schedule_display}' already pending after undock - waiting for undocking to complete")
            return

        # If already undocking, store schedule and wait
        if self._is_undocking:
            self._log_info(f"⏳ Undocking bereits im Gange - speichere Mission '{schedule_display}' für nach Undocking")
            self._pending_schedule_after_undock = schedule_id
            return

        # Check if robot is at home first - if at home, we're not docked
        schedule_display = self._get_schedule_display_name(schedule, schedule_id)
        if self._is_at_home():
            self._log_info(f"🚀 Starte Mission '{schedule_display}'")
            # Robot is at home, can start schedule directly
            route_mode = schedule.get('route_mode', 0)
            route_idx = random.randrange(len(schedule['route_names'])) if route_mode == 1 else 0
            self._mission_state.start_mission(schedule_id, route_idx, 0)
            self._charge_return_triggered = False  # Re-arm for this new mission
            self._load_and_publish_next_route()
            return

        if self._is_at_charge_position() or self._is_at_charge_position_by_gps():
            battery_status = f"{self._battery_level}%" if self._battery_level is not None else "unbekannt"
            
            # Check if battery is sufficient for new mission (only if charging after low battery)
            if self._overwritten_schedule_id and self._battery_level is not None:
                if self._battery_level < self._target_charge_soc:
                    self._log_warn(f"⏳ Mission '{schedule_display}' kann noch nicht gestartet werden: Batterie lädt noch ({self._battery_level}% < {self._target_charge_soc}%)")
                    # Store as pending schedule - will be started when battery reaches target
                    self._pending_schedule_after_undock = schedule_id
                    return
            
            self._log_info(f"🚛 Roboter ist an der Ladestation (Batterie: {battery_status}) - starte Undocking vor Mission '{schedule_display}'")
            
            # Store schedule to start after undocking completes
            self._pending_schedule_after_undock = schedule_id
            
            # Stop charging and trigger undocking
            self._enable_charging_relay(False)
            # Clear overwritten schedule since we're starting a new one
            self._overwritten_schedule_id = None
            
            undock_msg = Bool()
            undock_msg.data = True
            self._charging_undock_pub.publish(undock_msg)
            # Set undocking flag immediately to prevent stale state messages from interfering
            self._is_undocking = True
            # Set up timer to start schedule after undocking completes
            if self._undock_timer:
                self._undock_timer.cancel()
            self._undock_timer = self.create_timer(3.0, self._on_undock_complete_once)
            return

        # Not at home and not at charge position - can't start schedule
        self._log_warn(f"⚠️ Roboter nicht an der Basis - Mission '{schedule_display}' kann nicht gestartet werden")
        return

    def _load_and_publish_next_route(self):
        """Prepare next route and mark it ready for pickup.
        
        This method no longer publishes GeoPath directly.
        Instead, it sets _route_ready_for_pickup flag so the control loop
        can fetch the route via _get_schedule_action_service.
        
        The control loop is responsible for all robot control.
        """
        schedule_id = self._mission_state.get_current_schedule_id()
        if not schedule_id:
            return

        schedule = self._schedules.get(schedule_id)
        if not schedule:
            return

        route_idx, repetition = self._mission_state.get_current_route_info()
        route_names = schedule['route_names']
        route_mode = schedule.get('route_mode', 0)

        # Check if route_idx is out of bounds (safety check)
        if route_idx < 0 or route_idx >= len(route_names):
            # This should not happen in normal operation, but handle it gracefully
            if route_idx >= len(route_names) and route_mode == 0:  # Sequential mode
                # All routes completed - check loop_mode
                if schedule.get('loop_mode', False):
                    self._mission_state.set_current_route_info(0, 0)
                    self._log_info("Restarting schedule from beginning (safety check).")
                    # Recursively call to load the first route
                    self._load_and_publish_next_route()
                    return
                else:
                    self._log_info("All routes completed.")
                    # Stop mission - control loop will detect no active schedule and decide what to do
                    self._mission_state.stop_mission()
                    schedule['done'] = True
                    self._schedule_manager.save(schedule)
                    return
            else:
                # Invalid route_idx - try to recover
                self._log_error(f"Invalid route index {route_idx} for schedule with {len(route_names)} routes. Attempting recovery.")
                if route_mode == 1:  # Random mode - pick a random route
                    route_idx = random.randrange(len(route_names))
                    self._mission_state.set_current_route_info(route_idx, 0)
                    self._log_info(f"Recovered: selected random route {route_idx}")
                else:  # Sequential mode - start from beginning
                    route_idx = 0
                    self._mission_state.set_current_route_info(route_idx, 0)
                    self._log_info(f"Recovered: reset to route {route_idx}")

        route_name = route_names[route_idx]

        try:
            # Validate route exists
            waypoint_count = self._route_manager.get_waypoint_count(route_name)
            
            if waypoint_count > 0:
                # Set home position from first waypoint if not set
                if not self._home_position:
                    first_wp_tuple = self._route_manager.get_first_waypoint(route_name)
                    if first_wp_tuple:
                        self._home_position = first_wp_tuple
                        self.get_logger().info(f"Home position set from first waypoint: {self._home_position}")

                required_repetitions = schedule.get('route_repetitions', [])[route_idx]
                self._log_info(f"🛣️ Route '{route_name}' bereit ({repetition + 1}/{required_repetitions} Wiederholungen)")
                
                # Mark route as ready for pickup by control loop
                self._notify_route_ready(schedule_id, route_idx)
            else:
                self.get_logger().error(f"Route '{route_name}' not found or has no waypoints")
                self._log_error(f"Route '{route_name}' not found.")
                self._stop_current_mission()

        except Exception as e:
            self.get_logger().error(f"Failed to load route '{route_name}': {e}")
            self._log_error(f"Error loading route '{route_name}'.")
            self._stop_current_mission()

    def _deactivate_all_active_schedules(self):
        """Deactivate all active schedules internally (for button press)."""
        deactivated_count = 0
        for schedule_id, schedule in self._schedules.items():
            if schedule.get('active', False):
                schedule['active'] = False
                # Persist deactivation to YAML file
                if self._schedule_manager.save(schedule):
                    deactivated_count += 1
                    self._log_info(f"Mission '{schedule_id}' deaktiviert und gespeichert (Button-Druck).")
                else:
                    self.get_logger().error(f"Fehler beim Speichern der deaktivierten Mission '{schedule_id}'.")
        
        if deactivated_count > 0:
            self._log_info(f"{deactivated_count} Mission(en) deaktiviert.")
        
        # Stop current mission if active
        if self._mission_state.is_active():
            self._stop_current_mission()

    def _enable_charging_relay(self, enable: bool):
        """Enable/disable charging relay.
        
        This is the single point of control for the charging relay.
        Updates both the relay hardware and the unified tracking system.
        """
        # Update local tracking
        self._charging_relay_enabled = bool(enable)
        
        # Publish to hardware (via joy_controller -> ESP32)
        msg = Bool()
        msg.data = enable
        self._charging_enable_pub.publish(msg)
        
        # Log state change
        self.get_logger().info(f"Charging relay {'ENABLED' if enable else 'DISABLED'}")
        
        # Charging state is now determined from robot state machine, no persistence needed

    def is_charging_relay_enabled(self) -> bool:
        """Check if charging relay is currently enabled.
        
        Returns:
            True if charging relay is enabled, False otherwise
        """
        return self._charging_relay_enabled
    
    def _abort_charging_sequence(self):
        """Abort charging sequence."""
        self._charging_pending = False
        self._enable_charging_relay(False)
        
        undock_msg = Bool()
        undock_msg.data = True
        self._charging_undock_pub.publish(undock_msg)
        # Set undocking flag immediately to prevent stale state messages from interfering
        self._is_undocking = True

    def _handle_charging_completion(self):
        """Handle charging completion - check if schedule should be resumed."""
        # Check if we have an overwritten schedule (battery threshold case)
        if self._overwritten_schedule_id:
            # Check battery SoC
            if self._battery_level is None or self._battery_level < self._target_charge_soc:
                self._log_warn(f"Batteriestand ({self._battery_level}%) noch unter Ziel ({self._target_charge_soc}%). Warte auf weiteres Laden...")
                # Keep charging relay enabled and wait for battery to reach target
                self._enable_charging_relay(True)
                return
            
            # Check if autonomous operation is enabled
            if not self._autonomous_operation_enabled:
                self.get_logger().info(f"Batteriestand ausreichend, aber autonomer Modus ist deaktiviert - Mission wird nicht fortgesetzt.")
                self._enable_charging_relay(False)
                self._complete_charging_sequence()
                return
            
            # Battery is sufficient - disable charging and undock, then resume schedule
            self._enable_charging_relay(False)
            self._log_info(f"Batteriestand ausreichend ({self._battery_level}%). Starte Undocking und setze Mission fort.")
            self._resume_schedule_after_charging()
        else:
            # Normal charging completion (no schedule to resume)
            self._enable_charging_relay(False)
            self._complete_charging_sequence()
    
    def _handle_charging_error(self):
        """Handle charging error."""
        self._enable_charging_relay(False)
        self._is_undocking = False
        # Clear overwrite state on error
        self._clear_overwrite_state()
        self._complete_charging_sequence()
    
    def _resume_schedule_after_charging(self):
        """Undock and resume current active or new active schedule (battery threshold case)."""
        # Undock first
        self._log_info("Starte Undocking...")
        undock_msg = Bool()
        undock_msg.data = True
        self._charging_undock_pub.publish(undock_msg)
        # Set undocking flag immediately to prevent stale state messages from interfering
        self._is_undocking = True
        
        # Wait for undocking to complete, then check for active schedule
        # Use a one-shot timer to delay schedule resumption
        if self._undock_timer:
            self._undock_timer.cancel()
        # Create a one-shot timer (call once after 3 seconds)
        self._undock_timer = self.create_timer(3.0, self._on_undock_complete_once)
    
    def _on_undock_complete_once(self):
        """Called once after undocking to resume schedule.
        
        After undocking, the robot is at home position (2m behind charge point).
        Routes always start from home, so we just start the schedule normally from the beginning.
        """
        if self._undock_timer:
            self._undock_timer.cancel()
            self._undock_timer = None
        
        # Check if we have a pending schedule that needs to start after undocking (schedule started while docked)
        if self._pending_schedule_after_undock:
            schedule_id = self._pending_schedule_after_undock
            schedule = self._schedules.get(schedule_id)
            
            if schedule and schedule.get('active', False):
                # Schedule is still active - start it from the beginning
                # (routes start from home position, which is where we are after undocking)
                self._log_info(f"Starte Mission '{schedule_id}' nach Undocking.")
                route_mode = schedule.get('route_mode', 0)
                route_idx = random.randrange(len(schedule['route_names'])) if route_mode == 1 else 0
                self._mission_state.start_mission(schedule_id, route_idx, 0)
                self._charge_return_triggered = False  # Re-arm for this new mission
                self._load_and_publish_next_route()
                self._pending_schedule_after_undock = None
                return
            else:
                # Schedule no longer active - clear pending
                self._pending_schedule_after_undock = None
        
        # Check if we have a schedule that was interrupted (battery threshold case)
        if self._overwritten_schedule_id:
            schedule_id = self._overwritten_schedule_id
            schedule = self._schedules.get(schedule_id)
            
            if schedule and schedule.get('active', False):
                # Original schedule is still active - start it from the beginning
                # (routes start from home position, which is where we are after undocking)
                self._log_info(f"Starte ursprüngliche Mission '{schedule_id}' von Anfang (nach Undocking).")
                route_mode = schedule.get('route_mode', 0)
                route_idx = random.randrange(len(schedule['route_names'])) if route_mode == 1 else 0
                self._mission_state.start_mission(schedule_id, route_idx, 0)
                self._charge_return_triggered = False  # Re-arm for this new mission
                self._load_and_publish_next_route()
                self._clear_overwrite_state()
                return
        
        # Original schedule not active or not found - check for any active schedule
        self._log_info("Suche nach aktiver Mission...")
        self._clear_overwrite_state()
        
        # Check schedules immediately (will start if any are active)
        self._check_schedules()
    
    def _clear_overwrite_state(self):
        """Clear overwrite tracking state."""
        self._overwritten_schedule_id = None
    
    def _complete_charging_sequence(self):
        """Complete charging sequence (normal completion without schedule resume)."""
        self._enable_charging_relay(False)
        self._is_undocking = False
        self.get_logger().info("Ladesequenz abgeschlossen.")

    def _stop_current_mission(self):
        """Stop current mission and cancel any pending schedules.
        
        The control loop will detect mission stop via service query (has_active_schedule=False).
        """
        schedule_id = self._mission_state.get_current_schedule_id()
        self._log_warn(f"Mission '{schedule_id if schedule_id else ''}' gestoppt.")
        
        # Clear route ready flag
        self._route_ready_for_pickup = False
        
        # Clear pending schedule - if mission is stopped, don't start another one after undock
        if self._pending_schedule_after_undock:
            self.get_logger().info(f"Clearing pending schedule '{self._pending_schedule_after_undock}' due to mission stop")
            self._pending_schedule_after_undock = None
        
        # Cancel undock timer if active
        if self._undock_timer:
            self._undock_timer.cancel()
            self._undock_timer = None
        
        self._mission_state.stop_mission()
        self._charge_return_triggered = False  # Re-arm charge return for next mission

    def _is_at_home(self) -> bool:
        """Check if robot is at home."""
        if not self._home_position or not self._robot_position:
            return False
        return self._home_return.is_at_home(self._robot_position, self._home_position)

    def _distance_meters(self, a: Tuple[float, float], b: Tuple[float, float]) -> float:
        """Approximate distance in meters between two lat/lon points using Haversine."""
        try:
            lat1, lon1 = a
            lat2, lon2 = b
            # Haversine
            R = 6371000.0
            phi1 = math.radians(lat1)
            phi2 = math.radians(lat2)
            dphi = math.radians(lat2 - lat1)
            dlambda = math.radians(lon2 - lon1)
            sa = math.sin(dphi/2.0)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2.0)**2
            c = 2 * math.atan2(math.sqrt(sa), math.sqrt(max(0.0, 1-sa)))
            return R * c
        except Exception:
            return float('inf')

    def _is_at_charge_position(self) -> bool:
        """Determine whether robot is at the configured charge position.
        
        Uses charging state as primary indicator (most reliable), with GPS position as fallback.
        """
        # Primary check: if robot is in CHARGING state, it's definitely at charge position
        if self._unified_charging_state:
            return True
        
        # Fallback: check GPS position (less reliable due to GPS inaccuracy)
        return self._is_at_charge_position_by_gps()
    
    def _is_at_charge_position_by_gps(self) -> bool:
        """Check if robot is at charge position using GPS only (ignoring docked state).
        
        Returns:
            True if robot is within charge_detection_radius of charge position, False otherwise
        """
        if not self._robot_position or not self._charge_position:
            return False
        try:
            robot_lat, robot_lon = self._robot_position[0], self._robot_position[1]
            charge_lat, charge_lon = self._charge_position[0], self._charge_position[1]
            d = self._distance_meters((robot_lat, robot_lon), (charge_lat, charge_lon))
            return d <= float(self._charge_detection_radius)
        except Exception as e:
            self.get_logger().warn(f"Error checking if robot is at charge position by GPS: {e}")
            return False

    def _robot_state_machine_callback(self, msg: String):
        """Handle robot state machine updates to determine charging state.
        
        When robot enters CHARGING state and is at charge position, enable charging relay.
        This replaces the old StateManager-based persistence logic.
        """
        try:
            import json
            data = json.loads(msg.data)
            state = data.get('state')
            context = data.get('context', {})
            
            # Store last state for reference
            previous_state = self._last_robot_state
            self._last_robot_state = state
            
            # IMPORTANT: Don't process CHARGING/DOCKED state messages if undocking is in progress
            # This prevents stale messages from interfering with the undocking process
            if self._is_undocking:
                if state in ['CHARGING', 'DOCKED']:
                    self.get_logger().debug(f"Ignoring {state} state message - undocking in progress")
                    return
            
            # Clear charging request when robot reaches DOCKED state
            if state == 'DOCKED':
                # Clear charging request flag by publishing False
                charging_request_msg = Bool()
                charging_request_msg.data = False
                self._charging_requested_pub.publish(charging_request_msg)
                self._charge_return_triggered = False  # Re-arm so next mission can auto-return
                self.get_logger().debug("Robot reached DOCKED state - clearing charging request")
            
            # If robot is in CHARGING state and at charge position, enable charging relay
            # ONLY if we're not undocking (checked above)
            if state == 'CHARGING':
                is_at_charge_pos = context.get('is_at_charge_pos', False)
                # Only enable relay if:
                # 1. At charge position
                # 2. Relay not already enabled
                # 3. Not undocking (already checked above)
                # 4. Transitioning from DOCKED or first message (previous_state is None at startup)
                if is_at_charge_pos and not self._charging_relay_enabled:
                    if previous_state in ['DOCKED', None]:
                        self.get_logger().info("Robot in CHARGING state at charge position — enabling charging relay.")
                        self._enable_charging_relay(True)
                        if self._charge_return_triggered:
                            self._log_info("🔋 Automatischer Ladevorgang gestartet (Rückkehr wegen niedrigem Batteriestand).")
                        else:
                            self._log_info("🔌 Manueller Ladevorgang gestartet.")
                    else:
                        self.get_logger().debug(f"Ignoring CHARGING state relay enable - previous state was {previous_state}, expected DOCKED")
        except Exception as e:
            self.get_logger().warn(f"Failed to parse robot state machine message: {e}")


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = TacticalSchedulerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()