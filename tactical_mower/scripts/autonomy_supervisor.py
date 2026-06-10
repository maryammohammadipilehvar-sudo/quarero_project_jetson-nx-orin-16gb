#!/usr/bin/env python3
"""Auto pause/resume supervisor for autonomy nodes.

Listens on /control/autonomous_operation (Bool):
  true  -> SIGCONT all autonomy nodes (resume)
  false -> SIGSTOP all autonomy nodes (pause)

Also listens on /tactical/control/charging/requested (Bool):
  rising edge -> SIGCONT all autonomy nodes + call /control/autonomous_operation
                 service with state=True so the robot can actually drive home
                 even if the operator left autonomy OFF before pressing
                 "Zur Ladestation". Without this, the wp_follower receives
                 need_charge=True but stays in MANUAL because
                 autonomous_operation_enabled is False.

Safety:
  - atexit always SIGCONTs (nodes never stuck paused if supervisor dies)
  - SIGTERM handler resumes before exit
  - Never calls rclpy.shutdown() (avoids the rcl_shutdown race)
"""
import os, signal, subprocess, atexit, sys, uuid
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from interfaces.srv import CommandControl

NODE_PATTERNS = ['livox_obstacle_node', 'simple_obstacle_detector']


def find_pids(pattern):
    try:
        out = subprocess.check_output(['pgrep', '-f', pattern], text=True)
        return [int(p) for p in out.split() if p]
    except subprocess.CalledProcessError:
        return []


def sweep(sig, label):
    for pat in NODE_PATTERNS:
        for pid in find_pids(pat):
            try:
                os.kill(pid, sig)
                print(f'  {label} {pat} PID {pid}', flush=True)
            except ProcessLookupError:
                pass


def _resume_on_exit():
    print('exit -- defensive resume', flush=True)
    sweep(signal.SIGCONT, 'exit-resume')


atexit.register(_resume_on_exit)


def _term_handler(signum, frame):
    sweep(signal.SIGCONT, 'sigterm-resume')
    os._exit(0)


signal.signal(signal.SIGTERM, _term_handler)


class Sup(Node):
    def __init__(self):
        super().__init__('autonomy_supervisor')
        self.create_subscription(Bool, '/control/autonomous_operation', self.cb, 10)
        self.create_subscription(Bool, '/tactical/control/charging/requested', self.cb_charge, 10)
        self._auto_client = self.create_client(CommandControl, '/control/autonomous_operation')
        self.last = None
        self.last_charge = False
        self.get_logger().info('autonomy_supervisor: pause-on-manual / resume-on-autonomous / auto-enable-on-RTH')

    def cb(self, msg):
        if msg.data == self.last:
            return
        self.last = msg.data
        if msg.data:
            self.get_logger().info('AUTO ON  -> SIGCONT')
            sweep(signal.SIGCONT, 'resumed')
        else:
            self.get_logger().info('AUTO OFF -> SIGSTOP')
            sweep(signal.SIGSTOP, 'paused')

    def cb_charge(self, msg):
        if msg.data and not self.last_charge:
            self.last_charge = True
            self.get_logger().info('charging_requested rising edge -> resume + enable autonomy')
            sweep(signal.SIGCONT, 'rth-resume')
            if self._auto_client.wait_for_service(timeout_sec=2.0):
                req = CommandControl.Request()
                req.command_id = f'supervisor_rth_{uuid.uuid4().hex[:8]}'
                req.state = True
                self._auto_client.call_async(req)
                self.get_logger().info('autonomous_operation service called: state=True')
            else:
                self.get_logger().warn('autonomous_operation service unavailable!')
        elif not msg.data:
            self.last_charge = False


def main():
    rclpy.init()
    rclpy.spin(Sup())


if __name__ == '__main__':
    main()
