#!/usr/bin/env python3
import serial
import threading
import rclpy
from rclpy.node import Node
from interfaces.msg import Joy
from std_msgs.msg import Bool

# UART field mapping for PS5 controller
BUTTONS = {
    "right_stick_right": 0,
    "left_stick_forward": 1,
    "r2": 2,
    "l2": 3,
    "select": 4,
    "x": 5,
    "square": 6,
    "start": 7,
    "triangle": 8,
    "circle": 9,
    "l1": 10,
    "left_stick_right": 11,
    "right_stick_forward": 12,
    "r1": 13,
    "up": 14,
    "down": 15,
    "left": 16,
    "right": 17,
}


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

        # Thread for reading UART continuously
        self.running = True
        self.thread = threading.Thread(target=self.read_uart_loop, daemon=True)
        self.thread.start()

    def parse_line_to_vals(self, line: str):
        """Parse a UART line into a list of integer values.

        Args:
            line (str): Raw line received from UART.

        Returns:
            list[int] | None: List of integers corresponding to button/axis values,
                or None if line is invalid.
        """
        if not line:
            self.get_logger().warning("Received empty line")
            return None

        valid, vals = self.check_message(line)
        if not valid:
            self.get_logger().error(f"Invalid line: '{line}'")
            return None

        # Normalize length to match BUTTONS mapping
        if len(vals) < len(BUTTONS):
            vals += [0] * (len(BUTTONS) - len(vals))
        elif len(vals) > len(BUTTONS):
            vals = vals[:len(BUTTONS)]

        return vals

    def read_uart_loop(self):
        """Continuously read UART messages and publish them as Joy messages."""
        while self.running and self.ser.is_open:
            try:
                raw = self.ser.readline()
                if not raw:
                    continue
                line = raw.decode('utf-8', errors='ignore').strip()
                if not line:
                    continue

                vals = self.parse_line_to_vals(line)
                if vals is None:
                    continue

                msg = Joy()
                # Map UART values to Joy message
                msg.right_stick_right = vals[BUTTONS["right_stick_right"]]
                msg.right_stick_forward = vals[BUTTONS["right_stick_forward"]]
                msg.left_stick_right = vals[BUTTONS["left_stick_right"]]
                msg.left_stick_forward = vals[BUTTONS["left_stick_forward"]]
                msg.select = vals[BUTTONS["select"]] == 1
                msg.start = vals[BUTTONS["start"]] == 1
                msg.x = vals[BUTTONS["x"]] == 1
                msg.square = vals[BUTTONS["square"]] == 1
                msg.triangle = vals[BUTTONS["triangle"]] == 1
                msg.circle = vals[BUTTONS["circle"]] == 1
                msg.l1 = vals[BUTTONS["l1"]] == 1
                msg.l2 = vals[BUTTONS["l2"]]
                msg.r1 = vals[BUTTONS["r1"]] == 1
                msg.r2 = vals[BUTTONS["r2"]]
                msg.up = vals[BUTTONS["up"]] == 1
                msg.down = vals[BUTTONS["down"]] == 1
                msg.left = vals[BUTTONS["left"]] == 1
                msg.right = vals[BUTTONS["right"]] == 1

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

    def check_message(self, message: str) -> tuple[bool, list[int]]:
        """Validate and parse a UART message with CRC.

        The expected format is: "RX,RY,Right,Left,Select,Licht,GPS,CRC".

        Args:
            message (str): Raw UART message.

        Returns:
            tuple[bool, list[int]]: A tuple where the first element indicates
                if the message is valid, and the second element is the list of
                integer values.
        """
        parts = message.strip().split(',')
        if len(parts) < 8:  # ???
            return False, []

        # CRC is the last field
        crc_received = int(parts[-1], 16)
        data_str = ','.join(parts[:-1])
        data_bytes = data_str.encode('ascii')

        # Compute CRC
        crc_calc = self.crc16_ccitt(data_bytes)

        if crc_calc != crc_received:
            return False, []

        # Convert remaining fields to int
        values = [int(x) for x in parts[:-1]]
        return True, values

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
