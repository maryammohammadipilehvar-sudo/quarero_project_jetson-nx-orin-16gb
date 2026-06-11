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

Boot behavior:
  - Defaults to PAUSED (self.last = False) so target processes that spawn after
    supervisor startup are caught by the maintain-pause timer.

Safety:
  - atexit always SIGCONTs (nodes never stuck paused if supervisor dies)
  - SIGTERM handler resumes before exit
  - Never calls rclpy.shutdown() (avoids the rcl_shutdown race)

pgrep robustness:
  - find_pids() filters out shell processes via /proc/<pid>/comm so a bash
    script whose own argv happens to contain a node name does NOT get matched.
"""
import os, signal, subprocess, atexit, sys, uuid
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from interfaces.srv import CommandControl

NODE_PATTERNS = ["livox_obstacle_node", "simple_obstacle_detector", "livox_ros_driver2_node", "tactical_wp_follower", "nav2_navigation_node"]

_SHELL_COMMS = {'bash', 'sh', 'dash', 'zsh'}


def find_pids(pattern):
    """Return PIDs whose argv contains *pattern*, excluding shell processes.

    Why exclude shells: pgrep -f matches against the full command line, so a
    bash script that contains the node name as a literal string in its body
    will be returned. We don't want to signal those â€” only the actual python
    nodes.
    """
    try:
        out = subprocess.check_output(['pgrep', '-f', pattern], text=True)
    except subprocess.CalledProcessError:
        return []
    pids = []
    for p in out.split():
        try:
            with open(f'/proc/{p}/comm') as f:
                comm = f.read().strip()
        except OSError:
            continue
        if comm in _SHELL_COMMS:
            continue
        try:
            pids.append(int(p))
        except ValueError:
            pass
    return pids


def _is_stopped(pid):
    """True if the process is currently SIGSTOPped (State: T)."""
    try:
        with open(f'/proc/{pid}/status') as f:
            for line in f:
                if line.startswith('State:'):
                    return '\tT' in line or '(stopped)' in line
    except OSError:
        return False
    return False


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
        # Default to PAUSED at boot. Late-spawned target processes are caught
        # by the maintain timer.
        self.last = False
        self.last_charge = False
        self.create_timer(1.0, self._maintain_pause)
        self.get_logger().info('autonomy_supervisor: starting paused (default) â€” maintain timer 1 Hz')

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

    def _maintain_pause(self):
        """While paused, SIGSTOP any newly-spawned target processes.

        This makes the supervisor robust against:
          - target processes that spawned AFTER the initial boot publish
          - target processes that died and were respawned by ros2 launch
        """
        if self.last is not False:
            return
        for pat in NODE_PATTERNS:
            for pid in find_pids(pat):
                if _is_stopped(pid):
                    continue
                try:
                    os.kill(pid, signal.SIGSTOP)
                    self.get_logger().info(f'maintain-pause {pat} PID {pid}')
                except ProcessLookupError:
                    pass


def main():
    rclpy.init()
    rclpy.spin(Sup())


if __name__ == '__main__':
    main()
