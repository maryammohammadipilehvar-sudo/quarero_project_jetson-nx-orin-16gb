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
# Verified empirically on 2026-05-16: L1 sets bit 4 of `buttons` (0x0000 -> 0x0010).
L1_BIT = 0x0010


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

        # L1 edge state. On held→released, open a wall-clock zero-publish window:
        # for the next _ZERO_WINDOW_S seconds we publish a zeroed Joy on every
        # outer loop iteration regardless of UART read rate (the previous fixed
        # "3 burst frames" was iteration-coupled and frequently fell short, so
        # the watchdog had to do the braking 0.5–1.0s later). After the window
        # we go silent so /joy_web takeover after gamepad_timeout still works.
        self._l1_was_held = False
        self._l1_release_time = None  # monotonic seconds; None = no release yet
        self._ZERO_WINDOW_S = 0.5

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
        with L1 clear — except for the held→released edge, where we emit one
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
                    if m:
                        latest_match = m

                if latest_match is None:
                    continue
                m = latest_match

                _dpad_s, buttons_s, lx, ly, rx, ry, brake, throttle = m.groups()
                buttons = int(buttons_s, 16)
                l1_held = bool(buttons & L1_BIT)

                if not l1_held:
                    if self._l1_was_held:
                        self._l1_release_time = time.monotonic()
                        self._l1_was_held = False
                        self.get_logger().info(
                            f"L1 released — zero window {self._ZERO_WINDOW_S}s"
                        )
                    if self._l1_release_time is not None:
                        elapsed = time.monotonic() - self._l1_release_time
                        if elapsed < self._ZERO_WINDOW_S:
                            self.pub.publish(Joy())
                    continue

                self._l1_was_held = True
                msg = Joy()
                msg.left_stick_right    = int(lx)
                msg.left_stick_forward  = int(ly)
                msg.right_stick_right   = int(rx)
                msg.right_stick_forward = int(ry)
                msg.l1 = True
                msg.l2 = int(brake)
                msg.r2 = int(throttle)
                self.pub.publish(msg)

            except Exception as e:
                self.get_logger().error(f"UART read error: {e}")

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

    def _light_control_callback(self, msg: Bool):
        """Handle light control command from robot_controller.
        
        Forwards the command to ESP32 via UART.
        
        Args:
            msg: Light control message (True = on, False = off)
        """
        if not hasattr(self, 'ser') or self.ser is None or not self.ser.is_open:
            self.get_logger().warn("UART not available, cannot send light command")
            return
        
        # Send light command via UART
        # Payload format: mapRX,mapY,right,left,select,licht,gps
        light_value = 1 if msg.data else 0
        payload = f"0,0,0,0,0,{light_value},0"
        
        # Calculate CRC
        crc = self.crc16_ccitt(payload.encode('ascii'))
        
        # Format message: payload,CRC\n
        message = f"{payload},{crc:X}\n"
        
        try:
            self.ser.write(message.encode('utf-8'))
            self.get_logger().info(f"Light {'ON' if msg.data else 'OFF'} sent to ESP32 via UART")
        except Exception as e:
            self.get_logger().error(f"Failed to send light command: {e}")

    def _charging_control_callback(self, msg: Bool):
        """Handle charging control command from robot_controller.
        
        Forwards the command to ESP32 via UART.
        
        Args:
            msg: Charging control message (True = enable, False = disable)
        """
        if not hasattr(self, 'ser') or self.ser is None or not self.ser.is_open:
            self.get_logger().warn("UART not available, cannot send charging command")
            return
        
        # Send charging command via UART
        # Payload format: mapRX,mapY,right,left,select,licht,gps,charging
        charge_value = 1 if msg.data else 0
        payload = f"0,0,0,0,0,0,0,{charge_value}"
        
        # Calculate CRC
        crc = self.crc16_ccitt(payload.encode('ascii'))
        
        # Format message: payload,CRC\n
        message = f"{payload},{crc:X}\n"
        
        try:
            self.ser.write(message.encode('utf-8'))
            self.get_logger().info(f"Charging {'ENABLED' if msg.data else 'DISABLED'} sent to ESP32 via UART")
        except Exception as e:
            self.get_logger().error(f"Failed to send charging command: {e}")


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
