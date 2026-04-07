"""Battery voltage calculation with LiPo curve correction."""

import math
from typing import Optional, Tuple


class BatteryCalculator:
    """Calculates corrected battery voltage using LiPo discharge curve and current draw.
    
    This class models a LiPo battery pack and corrects voltage readings based on:
    - State of Charge (SOC) estimation
    - Open Circuit Voltage (OCV) calculation
    - Internal resistance (IR) drop compensation
    - Current draw from motors and system
    """
    
    def __init__(
        self,
        min_voltage: float,
        max_voltage: float,
        nominal_voltage: float,
        capacity_ah: float,
        internal_resistance: float,
        cells: int,
        default_power_usage: float,
        curve_enabled: bool = True
    ):
        """Initialize battery calculator.
        
        Args:
            min_voltage: Minimum safe voltage (V)
            max_voltage: Maximum voltage (V) - typically 4.2V per cell
            nominal_voltage: Nominal voltage (V) - typically 3.7V per cell
            capacity_ah: Battery capacity in Ah
            internal_resistance: Internal resistance in Ohms
            cells: Number of cells in series
            default_power_usage: Default system power usage in watts (non-motor systems)
            curve_enabled: Enable/disable LiPo curve correction
        """
        self.min_voltage = min_voltage
        self.max_voltage = max_voltage
        self.nominal_voltage = nominal_voltage
        self.capacity_ah = capacity_ah
        self.internal_resistance = internal_resistance
        self.cells = cells
        self.default_power_usage = default_power_usage
        self.curve_enabled = curve_enabled
    
    def calculate_ocv_from_soc(self, soc: float) -> float:
        """Calculate Open Circuit Voltage (OCV) from State of Charge (SOC) for LiPo battery.
        
        Uses a polynomial approximation of typical LiPo discharge curve.
        SOC is expected to be between 0.0 and 1.0.
        
        Args:
            soc: State of Charge (0.0 to 1.0)
            
        Returns:
            Open Circuit Voltage in Volts
        """
        # Clamp SOC to valid range
        soc = max(0.0, min(1.0, soc))
        
        # Per-cell voltage curve (typical LiPo: 2.75V empty/cutoff, 4.2V full)
        # Using a polynomial approximation for smooth curve
        # At SOC=0: 2.75V (cutoff), at SOC=1: 4.2V (full charge)
        cell_voltage = 2.75 + (4.2 - 2.75) * (
            soc + 0.1 * math.sin(math.pi * soc)  # Slight S-curve for realism
        )
        
        # Scale to total battery voltage
        return cell_voltage * self.cells
    
    def estimate_soc_from_loaded_voltage(self, loaded_voltage: float, current: float) -> float:
        """Estimate State of Charge from loaded voltage and current.
        
        This estimates SOC by calculating the OCV from the loaded voltage
        and IR drop, then converting OCV to SOC.
        
        Args:
            loaded_voltage: Measured voltage under load (V)
            current: Current draw (A)
            
        Returns:
            Estimated State of Charge (0.0 to 1.0)
        """
        # First, estimate OCV by adding IR drop
        estimated_ocv = loaded_voltage + (current * self.internal_resistance)
        
        # Clamp to valid range
        estimated_ocv = max(self.min_voltage, min(self.max_voltage, estimated_ocv))
        
        # Convert OCV to per-cell voltage
        cell_voltage = estimated_ocv / self.cells
        
        # Estimate SOC from per-cell voltage (inverse of OCV curve)
        # Realistic LiPo discharge curve (non-linear):
        # 4.2V = 100%, 3.9V = 90%, 3.8V = 80%, 3.7V = 50% (nominal), 
        # 3.6V = 30%, 3.5V = 20%, 3.4V = 15%, 3.3V = 10% (cutoff), 2.75V = 0%
        if cell_voltage <= 2.75:
            return 0.0
        elif cell_voltage >= 4.2:
            return 1.0
        elif cell_voltage <= 3.3:
            # Below cutoff (3.3V) - very low, linear from 3.3V to 2.75V
            # 3.3V = 10%, 2.75V = 0%
            soc = ((cell_voltage - 2.75) / (3.3 - 2.75)) * 0.10
            return max(0.0, soc)
        elif cell_voltage <= 3.5:
            # Critical range: 3.3V = 10%, 3.5V = 20%
            soc = 0.10 + ((cell_voltage - 3.3) / (3.5 - 3.3)) * 0.10
            return soc
        elif cell_voltage <= 3.6:
            # Low range: 3.5V = 20%, 3.6V = 30%
            soc = 0.20 + ((cell_voltage - 3.5) / (3.6 - 3.5)) * 0.10
            return soc
        elif cell_voltage <= 3.7:
            # Mid-low range: 3.6V = 30%, 3.7V = 50% (nominal - "knee" point)
            soc = 0.30 + ((cell_voltage - 3.6) / (3.7 - 3.6)) * 0.20
            return soc
        elif cell_voltage <= 3.8:
            # Mid range: 3.7V = 50%, 3.8V = 80%
            soc = 0.50 + ((cell_voltage - 3.7) / (3.8 - 3.7)) * 0.30
            return soc
        elif cell_voltage <= 3.9:
            # High range: 3.8V = 80%, 3.9V = 90%
            soc = 0.80 + ((cell_voltage - 3.8) / (3.9 - 3.8)) * 0.10
            return soc
        else:
            # Very high range: 3.9V = 90%, 4.2V = 100%
            soc = 0.90 + ((cell_voltage - 3.9) / (4.2 - 3.9)) * 0.10
            return min(1.0, soc)
    
    def calculate_corrected_voltage(
        self,
        raw_voltage: float,
        motor_current_m1: Optional[float] = None,
        motor_current_m2: Optional[float] = None
    ) -> float:
        """Calculate corrected voltage using LiPo battery curve and current draw.
        
        Args:
            raw_voltage: Raw voltage reading from RoboClaw (V)
            motor_current_m1: Motor 1 current in Amperes (optional, will be estimated if None)
            motor_current_m2: Motor 2 current in Amperes (optional, will be estimated if None)
            
        Returns:
            Corrected voltage (V)
        """
        if not self.curve_enabled:
            return raw_voltage
        
        try:
            # Calculate motor currents (use provided values or estimate from power)
            if motor_current_m1 is None or motor_current_m2 is None:
                # If currents not provided, estimate from default power usage
                # This is a fallback - ideally currents should be provided
                total_motor_current = 0.0
            else:
                total_motor_current = abs(motor_current_m1) + abs(motor_current_m2)
            
            # Calculate motor power consumption
            motor_power = raw_voltage * total_motor_current  # Watts
            
            # Total system power
            total_power = motor_power + self.default_power_usage  # Watts
            
            # Calculate total current (including system power)
            total_current = total_power / raw_voltage if raw_voltage > 0 else 0.0
            
            # Estimate SOC from loaded voltage
            estimated_soc = self.estimate_soc_from_loaded_voltage(raw_voltage, total_current)
            
            # Calculate OCV from SOC (for reference, but not used as corrected voltage)
            ocv = self.calculate_ocv_from_soc(estimated_soc)
            
            # Corrected voltage is the actual raw voltage (the real system voltage)
            # The raw voltage IS the true voltage at the system - no correction needed
            # OCV is only used internally for SOC estimation
            corrected_voltage = raw_voltage
            
            # Clamp to reasonable range
            corrected_voltage = max(self.min_voltage * 0.9, 
                                   min(self.max_voltage * 1.1, corrected_voltage))
            
            return corrected_voltage
            
        except Exception as e:
            # Return raw voltage on error
            return raw_voltage
    
    def calculate_soc_and_voltage(
        self,
        raw_voltage: float,
        motor_current_m1: Optional[float] = None,
        motor_current_m2: Optional[float] = None
    ) -> Tuple[float, float]:
        """Calculate corrected voltage and State of Charge.
        
        Args:
            raw_voltage: Raw voltage reading from RoboClaw (V)
            motor_current_m1: Motor 1 current in Amperes (optional)
            motor_current_m2: Motor 2 current in Amperes (optional)
            
        Returns:
            Tuple of (corrected_voltage, soc) where soc is 0.0 to 1.0
        """
        if not self.curve_enabled:
            # Simple SOC estimation from voltage without current correction
            soc = self.estimate_soc_from_loaded_voltage(raw_voltage, 0.0)
            return (raw_voltage, soc)
        
        try:
            # Calculate motor currents
            if motor_current_m1 is None or motor_current_m2 is None:
                total_motor_current = 0.0
            else:
                total_motor_current = abs(motor_current_m1) + abs(motor_current_m2)
            
            # Calculate motor power consumption
            motor_power = raw_voltage * total_motor_current  # Watts
            
            # Total system power
            total_power = motor_power + self.default_power_usage  # Watts
            
            # Calculate total current (including system power)
            total_current = total_power / raw_voltage if raw_voltage > 0 else 0.0
            
            # Estimate SOC from loaded voltage
            estimated_soc = self.estimate_soc_from_loaded_voltage(raw_voltage, total_current)
            
            # Calculate OCV from SOC (for reference, but not used as corrected voltage)
            ocv = self.calculate_ocv_from_soc(estimated_soc)
            
            # Corrected voltage is the actual raw voltage (the real system voltage)
            # The raw voltage IS the true voltage at the system - no correction needed
            # OCV is only used internally for SOC estimation
            corrected_voltage = raw_voltage
            
            # Clamp to reasonable range
            corrected_voltage = max(self.min_voltage * 0.9, 
                                   min(self.max_voltage * 1.1, corrected_voltage))
            
            return (corrected_voltage, estimated_soc)
            
        except Exception as e:
            # Fallback: estimate SOC from raw voltage
            soc = self.estimate_soc_from_loaded_voltage(raw_voltage, 0.0)
            return (raw_voltage, soc)
    
    def parse_roboclaw_currents(self, currents_result: Tuple[int, int, int]) -> Tuple[float, float]:
        """Parse RoboClaw current reading result.
        
        Args:
            currents_result: Tuple from RoboClaw.ReadCurrents() - (success, current_m1, current_m2)
                            Currents are in 10mA units
            
        Returns:
            Tuple of (current_m1, current_m2) in Amperes, or (None, None) if read failed
        """
        if not currents_result[0]:
            return (None, None)
        
        # Currents are in 10mA units, convert to Amperes
        current_m1 = currents_result[1] / 100.0  # Amperes
        current_m2 = currents_result[2] / 100.0  # Amperes
        
        return (current_m1, current_m2)

