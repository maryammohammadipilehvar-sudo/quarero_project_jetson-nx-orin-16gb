"""Light control service for ESP32."""

from typing import Optional
from ..hardware.uart_device import UartDevice


class LightService:
    """Service for controlling robot lights via UART."""
    
    def __init__(self, uart_device: Optional[UartDevice] = None):
        """Initialize light service.
        
        Args:
            uart_device: UART device instance (can be None if not available)
        """
        self._uart_device = uart_device
    
    def set_uart_device(self, uart_device: UartDevice):
        """Set or update UART device.
        
        Args:
            uart_device: UART device instance
        """
        self._uart_device = uart_device
    
    def turn_on(self) -> bool:
        """Turn light on.
        
        Returns:
            True if command sent successfully, False otherwise
        """
        if self._uart_device is None:
            return False
        return self._uart_device.send_light_command(True)
    
    def turn_off(self) -> bool:
        """Turn light off.
        
        Returns:
            True if command sent successfully, False otherwise
        """
        if self._uart_device is None:
            return False
        return self._uart_device.send_light_command(False)
    
    def is_available(self) -> bool:
        """Check if light service is available (UART connected).
        
        Returns:
            True if UART device is available and open, False otherwise
        """
        return self._uart_device is not None and self._uart_device.is_open()

