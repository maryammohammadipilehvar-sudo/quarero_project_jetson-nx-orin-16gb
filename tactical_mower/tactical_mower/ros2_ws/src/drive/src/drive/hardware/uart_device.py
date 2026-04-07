"""UART device interface for ESP32 communication."""

import serial
from typing import Optional


def crc16_ccitt(data: str) -> int:
    """
    Calculate CRC16-CCITT checksum (Polynomial 0x1021, Initial value 0xFFFF).
    Must match ESP32 implementation.
    
    Args:
        data: String to calculate CRC for
        
    Returns:
        CRC16 value as integer
    """
    if isinstance(data, str):
        data = data.encode('utf-8')
        
    crc = 0xFFFF
    
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF  # Limit to 16 bits
            
    return crc


class UartDevice:
    """UART communication device for ESP32 control messages.
    
    Handles serial communication with CRC16-CCITT checksum validation.
    Supports light control, charging control, and other ESP32 commands.
    """
    
    def __init__(self, port: str, baudrate: int = 19200, timeout: float = 1.0):
        """Initialize UART device.
        
        Args:
            port: Serial port path (e.g., '/dev/ttyTHS1')
            baudrate: Baud rate for serial communication
            timeout: Read timeout in seconds
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._serial: Optional[serial.Serial] = None
        self._is_open = False
    
    def open(self) -> bool:
        """Open UART connection.
        
        Returns:
            True if connection opened successfully, False otherwise
        """
        try:
            self._serial = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            self._is_open = True
            return True
        except Exception:
            self._serial = None
            self._is_open = False
            return False
    
    def close(self):
        """Close UART connection."""
        if self._serial is not None and self._serial.is_open:
            self._serial.close()
        self._serial = None
        self._is_open = False
    
    def is_open(self) -> bool:
        """Check if UART connection is open.
        
        Returns:
            True if connection is open, False otherwise
        """
        return self._is_open and self._serial is not None and self._serial.is_open
    
    def send_message(self, payload: str) -> bool:
        """Send message with CRC checksum.
        
        Args:
            payload: Message payload string (comma-separated values)
            
        Returns:
            True if message sent successfully, False otherwise
        """
        if not self.is_open():
            return False
        
        try:
            # Calculate CRC
            checksum = crc16_ccitt(payload)
            
            # Format message: payload,CRC\n
            message = f"{payload},{checksum:X}\n"
            
            # Send message
            self._serial.write(message.encode('utf-8'))
            return True
        except Exception:
            return False
    
    def send_light_command(self, light_on: bool) -> bool:
        """Send light control command.
        
        Args:
            light_on: True to turn light on, False to turn off
            
        Returns:
            True if command sent successfully, False otherwise
        """
        # Payload format: mapRX,mapY,right,left,select,licht,gps
        light_value = 1 if light_on else 0
        payload = f"0,0,0,0,0,{light_value},0"
        return self.send_message(payload)
    
    def send_charging_command(self, enable: bool) -> bool:
        """Send charging control command.
        
        Args:
            enable: True to enable charging, False to disable
            
        Returns:
            True if command sent successfully, False otherwise
        """
        # Payload format: mapRX,mapY,right,left,select,licht,gps,charging
        charge_value = 1 if enable else 0
        payload = f"0,0,0,0,0,0,0,{charge_value}"
        return self.send_message(payload)
    
    def __enter__(self):
        """Context manager entry."""
        self.open()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.close()

