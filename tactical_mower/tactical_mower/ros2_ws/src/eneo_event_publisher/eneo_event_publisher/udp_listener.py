"""UDP Listener for Eneo camera events."""

import socket
import json
import threading
from typing import Callable, Optional

from .eneo_event import EneoEvent


def parse_eneo_json(data: bytes) -> list[EneoEvent]:
    """
    Parse Eneo event JSON data and return a list of events.
    
    Args:
        data: Raw UDP payload bytes
        
    Returns:
        List of EneoEvent objects
    """
    events = []
    
    try:
        json_data = json.loads(data.decode('utf-8'))
        
        if json_data.get('result') != 'success':
            return events
        
        event_data = json_data.get('data', {})
        device_name = event_data.get('DeviceName', '')
        ip_address = event_data.get('IPAddress', '')
        mac_address = event_data.get('MacAddress', '')
        
        for alarm in event_data.get('alarm_list', []):
            event = EneoEvent(
                event_type=alarm.get('EventType', ''),
                event_time=alarm.get('EventTime', ''),
                event_action=alarm.get('EventAction', ''),
                channels=alarm.get('Chn', []),
                device_name=device_name,
                ip_address=ip_address,
                mac_address=mac_address
            )
            events.append(event)
            
    except (json.JSONDecodeError, UnicodeDecodeError) as e:
        # Error is logged by calling function
        pass
    
    return events


class UDPListener:
    """UDP Listener for Eneo camera events in separate thread."""
    
    def __init__(self, port: int, buffer_size: int, callback: Callable[[EneoEvent], None],
                 logger, camera_ip_filter: Optional[str] = None):
        """
        Initialize UDP Listener.
        
        Args:
            port: UDP port to listen on
            buffer_size: Buffer size for UDP packets
            callback: Function called for each event
            logger: ROS2 logger for logging
            camera_ip_filter: Optional: Only process events from this IP
        """
        self.port = port
        self.buffer_size = buffer_size
        self.callback = callback
        self.logger = logger
        self.camera_ip_filter = camera_ip_filter
        self.sock = None
        self.running = False
        self.thread = None
    
    def start(self):
        """Start UDP Listener in separate thread."""
        self.thread = threading.Thread(target=self._listen, daemon=True)
        self.thread.start()
    
    def stop(self):
        """Stop UDP Listener."""
        self.running = False
        if self.sock:
            self.sock.close()
    
    def _listen(self):
        """Main loop for UDP event reception."""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        
        try:
            self.sock.bind(('', self.port))
            self.running = True
            self.logger.info(f'Listening on UDP port {self.port} for Eneo events...')
            
            while self.running:
                try:
                    data, addr = self.sock.recvfrom(self.buffer_size)
                    self._process_data(data, addr)
                except socket.error as e:
                    if self.running:
                        self.logger.error(f'UDP Socket error: {e}')
        except Exception as e:
            self.logger.error(f'Error in UDP Listener: {e}', exc_info=True)
        finally:
            if self.sock:
                self.sock.close()
    
    def _process_data(self, data: bytes, addr: tuple):
        """Process received UDP data."""
        try:
            events = parse_eneo_json(data)
            
            for event in events:
                # Filter: Only process events from configured camera (optional)
                if self.camera_ip_filter and event.ip_address != self.camera_ip_filter:
                    continue
                
                # Call callback
                self.callback(event)
                
        except Exception as e:
            self.logger.error(f'Error processing UDP data: {e}', exc_info=True)

