"""ROS2 node for RoboClaw motor controller wrapper."""

import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32

from ..hardware.roboclaw_3 import Roboclaw
from ..hardware.battery_calculator import BatteryCalculator
from ..hardware.runtime_estimator import RuntimeEstimator
from interfaces.msg import CommandDrive


class RoboclawWrapperNode(Node):
    """Simplified wrapper for RoboClaw motor controllers without encoders.

    This node receives velocity commands and sends them to the RoboClaw motor
    controller. It also periodically publishes the main battery voltage.
    """

    def __init__(self):
        """Initialize the RoboClawWrapperNode.

        Declares parameters, initializes the RoboClaw connection, sets up motor
        mappings, publishers, and subscribers.
        """
        super().__init__("roboclaw_wrapper")
        self.log = self.get_logger()
        self.log.info("Initializing motor controllers (encoderless)")

        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('baud_rate', 115200),
                ('device', "/dev/ttyACM0"),
                ('addresses', [128]),
                ('duty_mode', True),
                ('velocity_qpps_to_duty_factor', 8),
                ('drive_acceleration_factor', 0.8),
                # /cmd_drive watchdog: if no command in this many seconds, brake.
                # Without this, the Roboclaw holds the last DutyAccel target
                # indefinitely (CLAUDE.md §1 missing-watchdog hazard).
                ('velocity_timeout', 0.5),
                ('roboclaw_mapping.drive_left.address', 128),
                ('roboclaw_mapping.drive_left.channel', 'M1'),
                ('roboclaw_mapping.drive_left.flip', False),
                ('roboclaw_mapping.drive_right.address', 128),
                ('roboclaw_mapping.drive_right.channel', 'M2'),
                ('roboclaw_mapping.drive_right.flip', False),
                ('voltage_publish_rate', 10.0),  # Hz
                # Battery configuration (7S12P 18650 pack: 30Ah, 24V nominal, 720Wh)
                ('battery.min_voltage', 19.25),  # Minimum voltage (V) - cutoff voltage (2.75V per cell * 7 cells)
                ('battery.max_voltage', 29.4),  # Maximum voltage (V) - charge voltage (4.2V per cell * 7 cells)
                ('battery.nominal_voltage', 24.0),  # Nominal voltage (V) - 24V as specified
                ('battery.capacity_ah', 30.0),  # Battery capacity in Ah
                ('battery.internal_resistance', 0.02),  # Internal resistance in Ohms (estimated for 7S12P pack)
                ('battery.cells', 7),  # Number of cells in series (7S configuration)
                ('battery.curve_enabled', True),  # Enable LiPo curve correction
                # Power consumption
                ('system.default_power_usage', 10.0),  # Default system power usage in watts (non-motor systems)
                # Runtime estimation
                ('runtime_estimation.safety_margin', 0.10),  # Safety margin (10%)
                ('runtime_estimation.short_term_window', 30.0),  # Short-term average window (seconds)
                ('runtime_estimation.long_term_window', 120.0),  # Long-term average window (seconds)
            ]
        )

        # Load parameters
        self.duty_mode = self.get_parameter('duty_mode').get_parameter_value().bool_value
        self.drive_accel = int(2**15 * self.get_parameter('drive_acceleration_factor').get_parameter_value().double_value)
        self.velocity_qpps_to_duty_factor = self.get_parameter('velocity_qpps_to_duty_factor').get_parameter_value().integer_value
        
        # Load battery configuration
        self.battery_min_voltage = self.get_parameter('battery.min_voltage').get_parameter_value().double_value
        self.battery_max_voltage = self.get_parameter('battery.max_voltage').get_parameter_value().double_value
        self.battery_nominal_voltage = self.get_parameter('battery.nominal_voltage').get_parameter_value().double_value
        self.battery_capacity_ah = self.get_parameter('battery.capacity_ah').get_parameter_value().double_value
        self.battery_internal_resistance = self.get_parameter('battery.internal_resistance').get_parameter_value().double_value
        self.battery_cells = self.get_parameter('battery.cells').get_parameter_value().integer_value
        self.battery_curve_enabled = self.get_parameter('battery.curve_enabled').get_parameter_value().bool_value
        
        # Load system power configuration
        self.default_power_usage = self.get_parameter('system.default_power_usage').get_parameter_value().double_value

        # Initialize battery calculator
        self.battery_calculator = BatteryCalculator(
            min_voltage=self.battery_min_voltage,
            max_voltage=self.battery_max_voltage,
            nominal_voltage=self.battery_nominal_voltage,
            capacity_ah=self.battery_capacity_ah,
            internal_resistance=self.battery_internal_resistance,
            cells=self.battery_cells,
            default_power_usage=self.default_power_usage,
            curve_enabled=self.battery_curve_enabled
        )
        
        # Initialize runtime estimator
        safety_margin = self.get_parameter('runtime_estimation.safety_margin').get_parameter_value().double_value
        short_term_window = self.get_parameter('runtime_estimation.short_term_window').get_parameter_value().double_value
        long_term_window = self.get_parameter('runtime_estimation.long_term_window').get_parameter_value().double_value
        
        self.runtime_estimator = RuntimeEstimator(
            capacity_ah=self.battery_capacity_ah,
            safety_margin=safety_margin,
            short_term_window=short_term_window,
            long_term_window=long_term_window
        )

        # Initialize RoboClaw connection
        self.serial_port = self.get_parameter('device').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.addresses = self.get_parameter('addresses').get_parameter_value().integer_array_value
        self.rc = Roboclaw(self.serial_port, self.baud_rate)
        if not self.rc.Open():
            self.log.fatal(f"Could not open serial port {self.serial_port}")
            raise RuntimeError("RoboClaw serial connection failed")

        # Motor mapping
        self.roboclaw_mapping = {
            "drive_left": {
                "address": self.get_parameter('roboclaw_mapping.drive_left.address').get_parameter_value().integer_value,
                "channel": self.get_parameter('roboclaw_mapping.drive_left.channel').get_parameter_value().string_value,
                "flip": self.get_parameter('roboclaw_mapping.drive_left.flip').get_parameter_value().bool_value
            },
            "drive_right": {
                "address": self.get_parameter('roboclaw_mapping.drive_right.address').get_parameter_value().integer_value,
                "channel": self.get_parameter('roboclaw_mapping.drive_right.channel').get_parameter_value().string_value,
                "flip": self.get_parameter('roboclaw_mapping.drive_right.flip').get_parameter_value().bool_value
            }
        }

        # Debug voltage override
        self._debug_voltage = None  # Debug voltage value (overrides real voltage)
        self._last_debug_voltage_time = None  # Timestamp of last debug voltage message
        self._debug_voltage_timeout = 3.0  # Seconds to wait before reverting to real voltage

        # Publisher for battery percentage
        self.battery_percentage_pub = self.create_publisher(Float32, '/drive/battery_percentage', 1)
        # Publisher for raw battery voltage (exact value from RoboClaw)
        self.battery_voltage_raw_pub = self.create_publisher(Float32, '/drive/battery_voltage_raw', 1)
        # Publisher for corrected battery voltage (intelligent calculation)
        self.battery_voltage_corrected_pub = self.create_publisher(Float32, '/drive/battery_voltage_corrected', 1)
        # Publisher for remaining runtime estimate
        self.runtime_estimate_pub = self.create_publisher(Float32, '/drive/runtime_estimate', 1)
        voltage_rate = self.get_parameter('voltage_publish_rate').get_parameter_value().double_value

        # All callbacks share a ReentrantCallbackGroup so a slow battery USB read
        # cannot starve drive_cmd_cb or the cmd_drive watchdog. Hardware access
        # is still serialised by self._rc_lock, since concurrent reads/writes
        # on the same /dev/ttyACM0 would garble the Roboclaw byte stream.
        self._cb_group = ReentrantCallbackGroup()
        self._rc_lock = threading.Lock()

        self.voltage_timer = self.create_timer(
            1.0 / voltage_rate, self.publish_battery_percentage,
            callback_group=self._cb_group,
        )

        # Track movement state for runtime estimation
        self._is_moving = False

        # Subscriber for drive commands
        self.drive_cmd_sub = self.create_subscription(
            CommandDrive, "/cmd_drive", self.drive_cmd_cb, 10,
            callback_group=self._cb_group,
        )

        # Subscriber for debug voltage override
        self.debug_voltage_sub = self.create_subscription(
            Float32, "/debug/motor_voltage", self._debug_voltage_callback, 1,
            callback_group=self._cb_group,
        )

        # /cmd_drive watchdog. Roboclaw firmware holds the last DutyAccel target
        # forever on serial silence; if upstream stops publishing (gamepad
        # disconnect, joy_controller crash, dropped L1-release zero) we need to
        # brake here.
        self._velocity_timeout = self.get_parameter('velocity_timeout').get_parameter_value().double_value
        self._last_cmd_time = None  # monotonic seconds; None = no command yet, watchdog quiet
        self._watchdog_braked = False
        self.create_timer(
            0.05, self._cmd_watchdog_check,
            callback_group=self._cb_group,
        )  # 20 Hz

        self.log.info(str(self.rc.ReadVersion(self.addresses[0])))
        self.log.info("RoboClaw wrapper initialized (encoderless)")
        self.log.info(f"Battery config: {self.battery_min_voltage}V - {self.battery_max_voltage}V, "
                     f"{self.battery_capacity_ah}Ah, {self.battery_cells} cells, "
                     f"R_internal={self.battery_internal_resistance}Ω")
        self.log.info(f"System default power usage: {self.default_power_usage}W")

    def _read_battery_data(self):
        """Read battery voltage and currents from RoboClaw.
        
        Returns:
            Tuple of (raw_voltage, current_m1, current_m2, success)
            Returns (None, None, None, False) on error
        """
        try:
            with self._rc_lock:
                voltage_result = self.rc.ReadMainBatteryVoltage(self.addresses[0])
                if not voltage_result[0]:
                    return (None, None, None, False)
                raw_voltage = voltage_result[1] / 10.0  # RoboClaw returns voltage * 10
                currents_result = self.rc.ReadCurrents(self.addresses[0])

            if not currents_result[0]:
                return (raw_voltage, None, None, True)

            current_m1, current_m2 = self.battery_calculator.parse_roboclaw_currents(currents_result)
            return (raw_voltage, current_m1, current_m2, True)

        except Exception as e:
            self.log.warn(f"Failed to read battery data: {e}")
            return (None, None, None, False)
    
    def _debug_voltage_callback(self, msg: Float32):
        """Callback for debug voltage override.
        
        Args:
            msg: Debug voltage value to override real voltage
        """
        self._debug_voltage = msg.data
        self._last_debug_voltage_time = self.get_clock().now()
        self.log.info(f"Debug voltage override set to: {self._debug_voltage}V")

    def publish_battery_percentage(self):
        """Read battery data, calculate percentage, and publish.
        
        Also updates runtime estimator with power consumption data.
        """
        try:
            current_time = self.get_clock().now().nanoseconds / 1e9
            
            # Check if debug voltage is active and not timed out
            use_debug = False
            if self._debug_voltage is not None and self._last_debug_voltage_time is not None:
                time_since_debug = (self.get_clock().now() - self._last_debug_voltage_time).nanoseconds / 1e9
                if time_since_debug < self._debug_voltage_timeout:
                    use_debug = True
                else:
                    # Timeout reached - revert to real voltage
                    if self._debug_voltage is not None:
                        self.log.info("Debug voltage timeout - reverting to real voltage")
                    self._debug_voltage = None
                    self._last_debug_voltage_time = None
            
            if use_debug:
                # Use debug voltage, estimate SOC from it
                raw_voltage = self._debug_voltage
                voltage = self._debug_voltage
                soc = self.battery_calculator.estimate_soc_from_loaded_voltage(voltage, 0.0)
                current_m1 = None
                current_m2 = None
                
                # Publish raw voltage (debug override)
                raw_voltage_msg = Float32()
                raw_voltage_msg.data = raw_voltage
                self.battery_voltage_raw_pub.publish(raw_voltage_msg)
                
                # Publish corrected voltage (same as raw in debug mode)
                corrected_voltage_msg = Float32()
                corrected_voltage_msg.data = voltage
                self.battery_voltage_corrected_pub.publish(corrected_voltage_msg)
            else:
                # Read battery data from RoboClaw
                raw_voltage, current_m1, current_m2, success = self._read_battery_data()
                
                if not success or raw_voltage is None:
                    self.log.warn("Failed to read battery data")
                    return
                
                # Publish raw voltage (exact value from RoboClaw)
                raw_voltage_msg = Float32()
                raw_voltage_msg.data = raw_voltage
                self.battery_voltage_raw_pub.publish(raw_voltage_msg)
                
                # Calculate corrected voltage and SOC
                voltage, soc = self.battery_calculator.calculate_soc_and_voltage(
                    raw_voltage=raw_voltage,
                    motor_current_m1=current_m1,
                    motor_current_m2=current_m2
                )
                
                # Publish corrected voltage (intelligent calculation)
                corrected_voltage_msg = Float32()
                corrected_voltage_msg.data = voltage
                self.battery_voltage_corrected_pub.publish(corrected_voltage_msg)
                
                # Update runtime estimator with power consumption
                if current_m1 is not None and current_m2 is not None:
                    total_current = abs(current_m1) + abs(current_m2)
                    # Add system power to get total current
                    total_power = voltage * total_current + self.default_power_usage
                    total_current_with_system = total_power / voltage if voltage > 0 else total_current
                    
                    self.runtime_estimator.update_power_consumption(
                        voltage=voltage,
                        current=total_current_with_system,
                        timestamp=current_time
                    )
            
            # Convert SOC (0.0-1.0) to percentage (0-100)
            battery_percentage = soc * 100.0
            battery_percentage = max(0.0, min(100.0, battery_percentage))
            
            # Publish battery percentage
            msg = Float32()
            msg.data = battery_percentage
            self.battery_percentage_pub.publish(msg)
            
            # Calculate and publish runtime estimate
            runtime_estimate = self.runtime_estimator.estimate_runtime(soc, current_time)
            
            if runtime_estimate is not None:
                # Only publish meaningful estimates (positive values)
                if runtime_estimate >= 0:
                    runtime_msg = Float32()
                    runtime_msg.data = runtime_estimate
                    self.runtime_estimate_pub.publish(runtime_msg)
            else:
                # If no estimate available (insufficient data), publish -1.0 to indicate unavailable
                runtime_msg = Float32()
                runtime_msg.data = -1.0
                self.runtime_estimate_pub.publish(runtime_msg)
            
        except Exception as e:
            self.log.warn(f"Failed to read battery percentage: {e}")

    def drive_cmd_cb(self, cmd: CommandDrive):
        """Callback for drive command messages.

        Args:
            cmd (CommandDrive): Incoming drive command with left/right wheel velocities.
        """
        # Stamp watchdog first so a slow send_velocity write can't trip the timeout
        self._last_cmd_time = time.monotonic()
        self._watchdog_braked = False

        # Check if robot is moving (non-zero velocity)
        is_moving = abs(cmd.left_vel) > 0.01 or abs(cmd.right_vel) > 0.01
        self._is_moving = is_moving

        # Update runtime estimator movement state
        current_time = self.get_clock().now().nanoseconds / 1e9
        self.runtime_estimator.set_moving(is_moving, current_time)

        self.send_velocity("drive_left", cmd.left_vel)
        self.send_velocity("drive_right", cmd.right_vel)

    def send_velocity(self, motor_name: str, velocity: float):
        """Send a velocity command (rad/s) to a motor.

        Args:
            motor_name (str): Name of the motor ("drive_left" or "drive_right").
            velocity (float): Desired velocity in rad/s.
        """
        props = self.roboclaw_mapping[motor_name]
        qpps = int(velocity * self.velocity_qpps_to_duty_factor)
        qpps = max(-2**15 + 1, min(2**15 - 1, qpps))
        if props["flip"]:
            qpps = -qpps

        with self._rc_lock:
            if props["channel"] == "M1":
                if self.duty_mode:
                    self.rc.DutyAccelM1(props["address"], self.drive_accel, qpps)
                else:
                    self.rc.SpeedAccelM1(props["address"], self.drive_accel, qpps)
            else:  # M2
                if self.duty_mode:
                    self.rc.DutyAccelM2(props["address"], self.drive_accel, qpps)
                else:
                    self.rc.SpeedAccelM2(props["address"], self.drive_accel, qpps)

    def _slam_brake(self):
        """DutyAccel(drive_accel, 0) on both motors. Used by watchdog and shutdown."""
        try:
            with self._rc_lock:
                self.rc.DutyAccelM1(self.addresses[0], self.drive_accel, 0)
                self.rc.DutyAccelM2(self.addresses[0], self.drive_accel, 0)
        except Exception as e:
            self.log.error(f"[brake] slam_brake write failed: {e}")

    def _cmd_watchdog_check(self):
        """Brake if no /cmd_drive arrived within velocity_timeout."""
        if self._last_cmd_time is None:
            return  # no command yet — stay quiet at startup
        elapsed = time.monotonic() - self._last_cmd_time
        if elapsed <= self._velocity_timeout:
            return
        if not self._watchdog_braked:
            self.log.warn(
                f"[watchdog] no /cmd_drive in {elapsed:.2f}s > {self._velocity_timeout}s — braking"
            )
            self._watchdog_braked = True
        self._slam_brake()
        self._is_moving = False

    def destroy_node(self):
        """Brake motors before tearing down. Avoids leaving last duty on the Roboclaw."""
        try:
            self._slam_brake()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    """Main entry point for the RoboClawWrapperNode.

    Uses a MultiThreadedExecutor so the slow Roboclaw battery USB poll cannot
    starve drive_cmd_cb / the cmd_drive watchdog. Hardware access is serialised
    by the node's internal lock.
    """
    rclpy.init(args=args)
    wrapper = RoboclawWrapperNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(wrapper)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        wrapper.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

