"""ROS2 Publisher methods"""
import json
import asyncio
from typing import List, Dict
from datetime import datetime
from geometry_msgs.msg import Point
from std_msgs.msg import String, Bool, Float32
from interfaces.msg import GeoPath, Joy, Schedule
from interfaces.srv import CommandControl, WaypointService, ScheduleService

from .status_manager import StatusManager


class PublisherMethods:
    """Handles all ROS2 publisher methods"""
    
    def __init__(self, node, status_manager: StatusManager):
        self.node = node
        self.status_manager = status_manager
    
    async def _call_service(self, client, state: bool, command_id: str, service_name: str) -> Dict:
        """Call a command control service.
        
        Since the ROS node is already spinning in a separate thread,
        we just need to call the service and wait for the future to complete.
        """
        # Wait for service to be available
        if not client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().error(f'Service {service_name} nicht verfügbar')
            return {"success": False, "message": f"Service {service_name} nicht verfügbar"}
        
        request = CommandControl.Request()
        request.command_id = command_id
        request.state = state
        
        self.node.get_logger().info(f'Calling service {service_name} with command_id: {command_id}, state: {state}')
        
        # Call service asynchronously
        future = client.call_async(request)
        
        # Wait for response with timeout
        # The node is already spinning in a separate thread, so we just wait
        timeout = 5.0
        start_time = asyncio.get_event_loop().time()
        
        while not future.done():
            elapsed = asyncio.get_event_loop().time() - start_time
            if elapsed > timeout:
                self.node.get_logger().error(f'Service {service_name} Timeout nach {timeout}s')
                return {"success": False, "message": f"Service-Aufruf Timeout nach {timeout}s"}
            await asyncio.sleep(0.01)  # Small sleep to yield to event loop
        
        try:
            response = future.result()
            result = {
                "success": response.success,
                "message": response.message
            }
            self.node.get_logger().info(f'Service {service_name} response: {result}')
            return result
        except Exception as e:
            self.node.get_logger().error(f'Service {service_name} Fehler: {e}', exc_info=True)
            return {"success": False, "message": f"Service-Fehler: {str(e)}"}
    
    async def call_light_service(self, state: bool, command_id: str) -> Dict:
        """Call light control service."""
        return await self._call_service(self.node.light_service_client, state, command_id, "light")
    
    async def call_alarm_service(self, state: bool, command_id: str) -> Dict:
        """Call alarm control service."""
        return await self._call_service(self.node.alarm_service_client, state, command_id, "alarm")
    
    async def call_siren_service(self, state: bool, command_id: str) -> Dict:
        """Call siren control service."""
        return await self._call_service(self.node.siren_service_client, state, command_id, "siren")
    
    async def call_emergency_stop_service(self, state: bool, command_id: str) -> Dict:
        """Call emergency stop service."""
        return await self._call_service(self.node.emergency_stop_service_client, state, command_id, "emergency_stop")
    
    async def call_autonomous_operation_service(self, state: bool, command_id: str) -> Dict:
        """Call autonomous operation service."""
        return await self._call_service(self.node.autonomous_operation_service_client, state, command_id, "autonomous_operation")
    
    async def call_go_to_charge_service(self, state: bool, command_id: str) -> Dict:
        """Call go to charge service."""
        return await self._call_service(self.node.go_to_charge_service_client, state, command_id, "go_to_charge")
    
    async def call_charge_manual_service(self, state: bool, command_id: str) -> Dict:
        """Call charge manual service."""
        return await self._call_service(self.node.charge_manual_service_client, state, command_id, "charge_manual")
    
    async def call_waypoint_service(self, waypoints: List[Dict], loop_mode: bool, command_id: str = None, route_name: str = '') -> Dict:
        """Call waypoint service."""
        from geometry_msgs.msg import Point
        import uuid
        
        if not self.node.waypoint_service_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().error('Service waypoints nicht verfügbar')
            return {"success": False, "message": "Service waypoints nicht verfügbar"}
        
        request = WaypointService.Request()
        request.command_id = command_id if command_id else str(uuid.uuid4())
        request.route_name = route_name
        
        # Convert waypoints to Point[]
        for wp in waypoints:
            point = Point()
            point.x = float(wp.get('latitude', 0.0))
            point.y = float(wp.get('longitude', 0.0))
            point.z = float(wp.get('altitude', 0.0))
            request.waypoints.append(point)
        
        # Set mode: 0=STOP, 1=LOOP, 2=PING_PONG
        if len(waypoints) == 0:
            request.mode = 0  # STOP
        elif loop_mode:
            request.mode = 1  # LOOP
        else:
            request.mode = 2  # PING_PONG
        
        self.node.get_logger().info(f'Calling waypoint service with {len(waypoints)} waypoints, mode={request.mode} (command_id: {command_id})')
        
        future = self.node.waypoint_service_client.call_async(request)
        
        timeout = 10.0
        start_time = asyncio.get_event_loop().time()
        
        while not future.done():
            elapsed = asyncio.get_event_loop().time() - start_time
            if elapsed > timeout:
                self.node.get_logger().error(f'Service waypoints Timeout nach {timeout}s')
                return {"success": False, "message": f"Service-Aufruf Timeout nach {timeout}s"}
            await asyncio.sleep(0.01)
        
        try:
            response = future.result()
            result = {
                "success": response.success,
                "message": response.message
            }
            self.node.get_logger().info(f'Waypoint service response: {result}')
            return result
        except Exception as e:
            self.node.get_logger().error(f'Waypoint Service Fehler: {e}', exc_info=True)
            return {"success": False, "message": f"Service-Fehler: {str(e)}"}
    
    async def call_schedule_service(self, action: str, schedule: Dict = None, schedule_id: str = None, command_id: str = None) -> Dict:
        """Call schedule service (add, remove, or update)."""
        import uuid
        
        # Service might take a moment to appear; retry a few times
        service_available = False
        for attempt in range(3):
            if self.node.schedule_service_client.wait_for_service(timeout_sec=2.0):
                service_available = True
                break
            await asyncio.sleep(0.2)
        if not service_available:
            self.node.get_logger().error('Service schedule nicht verfügbar (nach 3 Versuchen)')
            return {"success": False, "message": "Service schedule nicht verfügbar"}
        
        request = ScheduleService.Request()
        request.command_id = command_id if command_id else str(uuid.uuid4())
        request.action = action
        
        if action == "add" and schedule:
            # Convert dict to Schedule message
            from interfaces.msg import Schedule
            from ..utils.file_manager import load_settings
            
            schedule_msg = Schedule()
            schedule_msg.schedule_id = schedule.get('schedule_id', '')
            schedule_msg.weekdays = schedule.get('weekdays', [])
            schedule_msg.start_time = schedule.get('start_time', '')
            schedule_msg.end_time = schedule.get('end_time', '')
            schedule_msg.route_names = schedule.get('route_names', [])
            schedule_msg.route_repetitions = schedule.get('route_repetitions', [])
            schedule_msg.route_mode = 1 if schedule.get('route_mode', 'sequential') == 'random' else 0
            schedule_msg.loop_mode = schedule.get('loop_mode', False)
            schedule_msg.active = schedule.get('active', True)
            schedule_msg.require_home_return = schedule.get('require_home_return', True)
            
            # Load settings for defaults
            settings = load_settings()
            schedule_msg.battery_threshold = schedule.get('battery_threshold', settings.get('battery_threshold', 20))
            schedule_msg.home_tolerance = schedule.get('home_tolerance', settings.get('home_tolerance', 5.0))
            schedule_msg.auto_charge_return = schedule.get('auto_charge_return', settings.get('auto_charge_return', True))
            
            request.schedule = schedule_msg
        elif action == "update" and schedule and schedule_id:
            # Convert dict to Schedule message for update
            from interfaces.msg import Schedule
            from ..utils.file_manager import load_settings
            
            schedule_msg = Schedule()
            schedule_msg.schedule_id = schedule.get('schedule_id', '')
            schedule_msg.weekdays = schedule.get('weekdays', [])
            schedule_msg.start_time = schedule.get('start_time', '')
            schedule_msg.end_time = schedule.get('end_time', '')
            schedule_msg.route_names = schedule.get('route_names', [])
            schedule_msg.route_repetitions = schedule.get('route_repetitions', [])
            schedule_msg.route_mode = 1 if schedule.get('route_mode', 'sequential') == 'random' else 0
            schedule_msg.loop_mode = schedule.get('loop_mode', False)
            schedule_msg.active = schedule.get('active', True)
            schedule_msg.require_home_return = schedule.get('require_home_return', True)
            
            # Load settings for defaults
            settings = load_settings()
            schedule_msg.battery_threshold = schedule.get('battery_threshold', settings.get('battery_threshold', 20))
            schedule_msg.home_tolerance = schedule.get('home_tolerance', settings.get('home_tolerance', 5.0))
            schedule_msg.auto_charge_return = schedule.get('auto_charge_return', settings.get('auto_charge_return', True))
            
            request.schedule = schedule_msg
            request.schedule_id = schedule_id  # Old schedule ID
        elif action == "remove" and schedule_id:
            request.schedule_id = schedule_id
        else:
            return {"success": False, "message": f"Ungültige Aktion oder fehlende Daten: action={action}"}
        
        self.node.get_logger().info(f'Calling schedule service: action={action} (command_id: {command_id})')
        
        future = self.node.schedule_service_client.call_async(request)
        
        timeout = 10.0
        start_time = asyncio.get_event_loop().time()
        
        while not future.done():
            elapsed = asyncio.get_event_loop().time() - start_time
            if elapsed > timeout:
                self.node.get_logger().error(f'Service schedule Timeout nach {timeout}s')
                return {"success": False, "message": f"Service-Aufruf Timeout nach {timeout}s"}
            await asyncio.sleep(0.01)
        
        try:
            response = future.result()
            result = {
                "success": response.success,
                "message": response.message
            }
            self.node.get_logger().info(f'Schedule service response: {result}')
            return result
        except Exception as e:
            self.node.get_logger().error(f'Schedule Service Fehler: {e}', exc_info=True)
            return {"success": False, "message": f"Service-Fehler: {str(e)}"}
    
    
    def publish_joy_command(self, left_stick_forward: int, right_stick_left: int) -> None:
        msg = Joy()
        msg.left_stick_forward = int(left_stick_forward)
        msg.left_stick_right = 0
        msg.right_stick_forward = 0
        msg.right_stick_right = int(right_stick_left)
        msg.select = False
        msg.start = False
        msg.x = False
        msg.square = False
        msg.triangle = False
        msg.circle = False
        msg.l1 = False
        msg.l2 = 0
        msg.r1 = False
        msg.r2 = 0
        msg.up = False
        msg.down = False
        msg.left = False
        msg.right = False
        self.node.joy_web_pub.publish(msg)

    # Removed set_light - now using service call_light_service

    # Removed set_alarm - now using service call_alarm_service
    
    # Removed set_siren - now using service call_siren_service

    def publish_speed_factor(self, factor: float) -> None:
        msg = Float32()
        speed = round(float(factor), 1)
        msg.data = speed
        self.node.set_speed_pub.publish(msg)
        self.node.speed_factor = speed
        self.node.get_logger().info(f'Speed factor set to {factor:.1f}x')

    def publish_obstacle_avoidance_enabled(self, enabled: bool) -> None:
        msg = Bool()
        msg.data = enabled
        self.node.obstacle_avoidance_pub.publish(msg)
        self.node.get_logger().info(f'Obstacle avoidance {"enabled" if enabled else "disabled"}')

    def send_move_command(self, x: float, y: float) -> None:
        if self.status_manager.emergency_active:
            x = 0.0
            y = 0.0
        msg = Point()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = 0.0
        self.node.move_pub.publish(msg)
        self.node.get_logger().debug(f'Move command: X={x:.2f}, Y={y:.2f}')

    async def trigger_emergency_stop(self, command_id: str = None) -> Dict:
        """Trigger emergency stop via service."""
        result = await self.call_emergency_stop_service(True, command_id or "")
        if result.get("success"):
            self.status_manager.emergency_active = True
            try:
                self.publish_joy_command(0, 0)
            except Exception as e:
                self.node.get_logger().error(f'Fehler beim Veröffentlichen von Joy-Null: {e}')
        return result

    async def clear_emergency_stop(self, command_id: str = None) -> Dict:
        """Clear emergency stop via service."""
        result = await self.call_emergency_stop_service(False, command_id or "")
        if result.get("success"):
            self.status_manager.emergency_active = False
        return result

    # Removed old set_autonomous_operation - now using service call_autonomous_operation_service

