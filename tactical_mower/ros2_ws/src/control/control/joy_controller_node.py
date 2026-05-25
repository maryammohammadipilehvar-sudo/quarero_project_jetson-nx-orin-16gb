#!/usr/bin/env python3
import re
import serial
import threading
import time
import rclpy
from rclpy.node import Node
from interfaces.msg import Joy
from std_msgs.msg import Bool

# ESP UART0 (USB-CP2102) debug-printf input. The original UART1 binary joy protocol
# (RX,RY,...,CRC) is unusable on Orin NX — serial-tegra @ 19200 corrupts mid-frame
# bytes. UART0 prints structured text at 115200 that contains the same controller
# state; we parse that here.
ESP_JOY_LINE = re.compile(
    r'idx=\d+,\s*'
    r'dpad:\s*(0x[0-9a-fA-F]+),\s*'
    r'buttons:\s*(0x[0-9a-fA-F]+),\s*'
    r'axis L:\s*(-?\d+),\s*(-?\d+),\s*'
    r'axis R:\s*(-?\d+),\s*(-?\d+),\s*'
    r'brake:\s*(-?\d+),\s*'
    r'throttle:\s*(-?\d+)'
)
L1_BIT = 0x0010
CIRCLE_BIT = 0x0002


class JoyController(Node):
    """ROS 2 node that reads joystick inputs via UART and publishes as Joy messages."""

    def __init__(self):
        """Initialize the JoyController node.

        Declares parameters, sets up UART connection, starts the reading thread, and
        initializes the publisher.
        """
        super().__init__('joy_controller')

        # Declare parameters
        self.declare_parameter('port', '/dev/ttyTHS1')
        self.declare_parameter('baudrate', 19200)
        self.declare_parameter('topic_output', '/joy_drive_raw')
        self.declare_parameter('topic_light_control', '/control/light')
        self.declare_parameter('topic_charging_control', '/control/enable_charging')

        # Load parameters
        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.topic_output = self.get_parameter('topic_output').get_parameter_value().string_value
        topic_light = self.get_parameter('topic_light_control').get_parameter_value().string_value
        topic_charging = self.get_parameter('topic_charging_control').get_parameter_value().string_value

        # Publishers
        self.pub = self.create_publisher(Joy, self.topic_output, 10)
        self._save_waypoint_pub = self.create_publisher(Bool, '/control/save_waypoint', 10)

        # Circle button edge detection for waypoint saving
        self._circle_was_pressed = False

        # Track GPIO states so commands don't reset each other
        self._light_state = False
        self._charging_state = False

        # Subscribers for light and charging commands (forwarded to ESP32 via UART)
        self.create_subscription(Bool, topic_light, self._light_control_callback, 10)
        self.create_subscription(Bool, topic_charging, self._charging_control_callback, 10)

        # Open UART
        try:
            self.ser = serial.Serial(port, baudrate=baudrate, timeout=0.1)
            self.get_logger().info(f"UART opened: {port} @ {baudrate} baud")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open UART: {e}")
            raise e

        # L1 edge state. On held->released, open a wall-clock zero-publish window:
        # for the next _ZERO_WINDOW_S seconds we publish a zeroed Joy on every
        # outer loop iteration regardless of UART read rate (the previous fixed
        # "3 burst frames" was iteration-coupled and frequently fell short, so
        # the watchdog had to do the braking 0.5-1.0s later). After the window
        # we go silent so /joy_web takeover after gamepad_timeout still works.
        self._l1_was_held = False
        self._l1_release_time = None  # monotonic seconds; None = no release yet
        self._ZERO_WINDOW_S = 0.5

        # L1 hold timer. Bluepad32 occasionally reports L1=false for a
        # single frame while the button is physically held. With the ESP
        # failsafe heartbeat already filtered (all-zero fingerprint), the
        # only remaining ghost L1=false frames come from real gamepad data.
        # Ignore any L1=false that arrives within this window of the last
        # L1=true frame.
        self._last_l1_true_time = 0.0
        self._L1_HOLD_GRACE_S = 0.08

        # Stream-silence watchdog. Independent of the L1-release edge path:
        # if the ESP debug stream goes quiet (USB hiccup, ESP reboot, BT
        # disconnect not signalled in-band, scheduler jitter) while L1 was
        # last held, publish one zero Joy so robot_controller stops trusting
        # the last gamepad value. After firing we disarm; robot_controller's
        # gamepad_timeout (0.5 s) then takes over and falls back to /joy_web.
        self._FRAME_SILENCE_S = 0.15
        self._last_frame_time = None       # monotonic; None until first parsed line
        self._silence_watchdog_fired = False
        self.create_timer(0.05, self._silence_watchdog_check)

        # Thread for reading UART continuously
        self.running = True
        self.thread = threading.Thread(target=self.read_uart_loop, daemon=True)
        self.thread.start()

    def read_uart_loop(self):
        """Read ESP debug stream from UART0 and publish Joy messages.

        The ESP transmits debug output unconditionally on UART0 (unlike UART1,
        which gated transmit on L1) and interleaves joy + IMU/accel lines at
        a rate that can outpace per-line readline() processing. To avoid
        publishing stale frames (the symptom: ~6 s lag on L1 release because
        we replay buffered L1-held frames first), each iteration drains the
        serial buffer and only acts on the LATEST valid joy line.

        Legacy deadman semantic is "no Joy unless L1 held", so we drop frames
        with L1 clear -- except for the held->released edge, where we emit one
        zeroed Joy to force the cascade to stop the motors.
        """
        while self.running and self.ser.is_open:
            try:
                first = self.ser.readline()
                if not first:
                    continue
                chunks = [first]
                # Bounded drain: ESP streams at the UART wire rate, so an
                # unbounded `while in_waiting > 0` loop spins forever
                # (every line we consume, another arrives). 32 lines per
                # outer iteration is enough to keep up with ~200 lines/sec
                # at our publish cadence without starving the publish step.
                for _ in range(32):
                    if self.ser.in_waiting <= 0:
                        break
                    more = self.ser.readline()
                    if not more:
                        break
                    chunks.append(more)

                latest_match = None
                for raw in chunks:
                    line = raw.decode('utf-8', errors='ignore').strip()
                    if not line.startswith('idx='):
                        continue
                    m = ESP_JOY_LINE.match(line)
                    if not m:
                        continue
                    # The ESP failsafe heartbeat emits idx= lines with
                    # every field zero. Real gamepad data always has
                    # non-zero stick rest bias. Skip the failsafe lines
                    # so they can't trigger false L1-release events.
                    _d, _b, _lx, _ly, _rx, _ry, _br, _th = m.groups()
                    if _b == '0x0000' and _lx == '0' and _ly == '0' and _rx == '0' and _ry == '0':
                        continue
                    latest_match = m

                if latest_match is None:
                    continue
                m = latest_match
                self._last_frame_time = time.monotonic()
                self._silence_watchdog_fired = False

                _dpad_s, buttons_s, lx, ly, rx, ry, brake, throttle = m.groups()
                buttons = int(buttons_s, 16)
                l1_in_frame = bool(buttons & L1_BIT)

                if l1_in_frame:
                    self._last_l1_true_time = time.monotonic()

                # Ignore brief L1=false glitches from Bluepad32 HID jitter.
                if not l1_in_frame and (time.monotonic() - self._last_l1_true_time) < self._L1_HOLD_GRACE_S:
                    continue

                l1_held = l1_in_frame

                if not l1_held:
                    if self._l1_was_held:
                        self._l1_release_time = time.monotonic()
                        self._l1_was_held = False
                        self.get_logger().info(
                            f"L1 released -- zero window {self._ZERO_WINDOW_S}s"
                        )
                    if self._l1_release_time is not None:
                        elapsed = time.monotonic() - self._l1_release_time
                        if elapsed < self._ZERO_WINDOW_S:
                            self.pub.publish(Joy())
                    continue

                self._l1_was_held = True
                msg = Joy()
                msg.left_stick_right    = int(lx)
                # Bluepad32 PS5 Y axis is negative when stick is pushed up/forward
                # (standard Y-down gamepad convention). The kinematics expects
                # positive=forward (differential_drive.py:34). The old UART1
                # binary path inverted Y on the ESP (PS5_ESP32.ino:301), but the
                # UART0 text dump we parse here prints raw ctl->axisY/RY without
                # that inversion -- so we re-apply it here.
                msg.left_stick_forward  = -int(ly)
                msg.right_stick_right   = int(rx)
                msg.right_stick_forward = -int(ry)
                msg.l1 = True
                msg.l2 = int(brake)
                msg.r2 = int(throttle)
                self.pub.publish(msg)

                # Circle button → save waypoint (rising edge only)
                circle_pressed = bool(buttons & CIRCLE_BIT)
                if circle_pressed and not self._circle_was_pressed:
                    wp_msg = Bool()
                    wp_msg.data = True
                    self._save_waypoint_pub.publish(wp_msg)
                    self.get_logger().info("Circle pressed — waypoint save published")
                self._circle_was_pressed = circle_pressed

            except Exception as e:
                self.get_logger().error(f"UART read error: {e}")

    def _silence_watchdog_check(self):
        """Publish one zero Joy if the ESP stream has been silent while L1 was held.

        Symptom this guards against: ESP stops emitting idx= lines (USB
        glitch, ESP reset, BT drop not detected in firmware). Without this,
        the only thing that stops the motors is roboclaw_wrapper's 0.5 s
        /cmd_drive watchdog -- a perceptible delay on a manually-driven robot.
        """
        if self._last_frame_time is None:
            return
        if not self._l1_was_held:
            return
        if self._silence_watchdog_fired:
            return
        if time.monotonic() - self._last_frame_time < self._FRAME_SILENCE_S:
            return
        self.get_logger().warn(
            f"ESP stream silent >{self._FRAME_SILENCE_S}s while L1 held -- publishing zero Joy"
        )
        self.pub.publish(Joy())
        self._silence_watchdog_fired = True
        self._l1_was_held = False  # robot_controller's gamepad_timeout takes it from here

    def destroy_node(self):
        """Close the UART and stop the reading thread before destroying the node."""
        self.running = False
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

    def crc16_ccitt(self, data: bytes, crc: int = 0xFFFF) -> int:
        """Compute CRC16-CCITT (0x1021) checksum.

        Args:
            data (bytes): Input byte sequence to calculate CRC over.
            crc (int, optional): Initial CRC value. Defaults to 0xFFFF.

        Returns:
            int: Calculated CRC16-CCITT value.
        """
        for b in data:
            crc ^= b << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = ((crc << 1) & 0xFFFF) ^ 0x1021
                else:
                    crc = (crc << 1) & 0xFFFF
        return crc

    def _send_gpio_command(self):
        """Send current light + charging state to ESP32 as a single frame.

        Payload: mapRX,mapY,right,left,select,licht,gps,charging
        Both GPIO states are always included so one command never resets the other.
        """
        if not hasattr(self, 'ser') or self.ser is None or not self.ser.is_open:
            self.get_logger().warn("UART not available, cannot send GPIO command")
            return

        light_val = 1 if self._light_state else 0
        charge_val = 1 if self._charging_state else 0
        payload = f"0,0,0,0,0,{light_val},0,{charge_val}"

        crc = self.crc16_ccitt(payload.encode('ascii'))
        message = f"{payload},{crc:X}\n"

        try:
            self.ser.write(message.encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f"Failed to send GPIO command: {e}")

    def _light_control_callback(self, msg: Bool):
        """Handle light control command -- forwards to ESP32 via UART."""
        self._light_state = msg.data
        self._send_gpio_command()
        self.get_logger().info(f"Light {'ON' if msg.data else 'OFF'} sent to ESP32")

    def _charging_control_callback(self, msg: Bool):
        """Handle charging control command -- forwards to ESP32 via UART."""
        self._charging_state = msg.data
        self._send_gpio_command()
        self.get_logger().info(f"Charging {'ENABLED' if msg.data else 'DISABLED'} sent to ESP32")


def main(args=None):
    """Main entry point for JoyController node."""
    rclpy.init(args=args)
    node = JoyController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
