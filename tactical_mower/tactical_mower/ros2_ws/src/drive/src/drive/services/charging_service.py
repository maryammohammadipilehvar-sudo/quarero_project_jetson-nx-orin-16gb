"""Charging control service for ESP32."""

from typing import Optional
from ..hardware.uart_device import UartDevice


class ChargingService:
    """Service for controlling robot charging relay via UART."""
    
    def __init__(self, uart_device: Optional[UartDevice] = None):
        """Initialize charging service.
        
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
    
    def enable(self) -> bool:
        """Enable charging relay.
        
        Returns:
            True if command sent successfully, False otherwise
        """
        if self._uart_device is None:
            return False
        return self._uart_device.send_charging_command(True)
    
    def disable(self) -> bool:
        """Disable charging relay.
        
        Returns:
            True if command sent successfully, False otherwise
        """
        if self._uart_device is None:
            return False
        return self._uart_device.send_charging_command(False)
    
    def is_available(self) -> bool:
        """Check if charging service is available (UART connected).
        
        Returns:
            True if UART device is available and open, False otherwise
        """
        return self._uart_device is not None and self._uart_device.is_open()

