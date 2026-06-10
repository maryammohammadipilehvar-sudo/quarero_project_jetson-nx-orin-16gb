#!/usr/bin/env python3
"""Bridge /person_detected (Bool) -> /security_alert (SecurityAlert).

Person-detection events become persistent items in the Ereignisse tab.

Debounce: do not republish more often than once per DEBOUNCE_S, and only on
the rising edge (False -> True). If /person_detected stays True for an
extended period, we re-emit one event every REPEAT_S so long-running
detections are still represented in the timeline.
"""
import os, time, signal, sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from interfaces.msg import SecurityAlert
from builtin_interfaces.msg import Time as TimeMsg

DEBOUNCE_S = 10.0
REPEAT_S = 60.0


class PersonEventBridge(Node):
    def __init__(self):
        super().__init__('person_event_bridge')
        self.alert_pub = self.create_publisher(SecurityAlert, '/security_alert', 10)
        self.create_subscription(Bool, '/person_detected', self.cb, 10)
        self._last_state = False
        self._last_emit = 0.0
        self.get_logger().info('person_event_bridge: /person_detected -> /security_alert')

    def cb(self, msg: Bool):
        now = time.time()
        # Rising edge OR long-running re-emit
        rising = msg.data and not self._last_state
        repeat = msg.data and (now - self._last_emit) >= REPEAT_S
        if rising or repeat:
            if (now - self._last_emit) < DEBOUNCE_S and not repeat:
                self._last_state = msg.data
                return
            self._emit()
            self._last_emit = now
        self._last_state = msg.data

    def _emit(self):
        alert = SecurityAlert()
        alert.event_type = 'person'
        sec = int(time.time())
        ns = int((time.time() - sec) * 1e9)
        alert.event_time = TimeMsg(sec=sec, nanosec=ns)
        alert.device_name = 'oak'
        alert.description = 'Person erkannt'
        alert.camera_ids = ['person_detection']
        self.alert_pub.publish(alert)
        self.get_logger().info('SecurityAlert published: person')


def main():
    rclpy.init()
    node = PersonEventBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
