#!/usr/bin/env python3
"""
mission_supervisor.py — turns /mission/mode into a launched (or killed) frontier stack.

One instance per machine. On '/mission/mode' == 'frontier' it spawns its --launch-file as
a new process group; on anything else it SIGINT->SIGTERM->SIGKILL escalates that group.
Publishes a per-machine latched state string so the dashboard can show real status.

  python3 deploy/nodes/mission_supervisor.py \
      --role pi --launch-file /home/amr/ros2_ws/deploy/launch/pi_nav2_delayed.launch.py \
      --state-topic /mission/state/pi
"""

import argparse
import os
import signal
import subprocess
import time


def decide_action(running, requested):
    """requested in {'manual','frontier','idle',...}; returns 'start'|'stop'|'noop'."""
    if requested == 'frontier':
        return 'noop' if running else 'start'
    return 'stop' if running else 'noop'


class ProcessManager:
    """Owns a single child process group; start is idempotent, stop escalates signals."""

    def __init__(self, cmd):
        self.cmd = cmd
        self.proc = None

    @property
    def running(self):
        return self.proc is not None and self.proc.poll() is None

    def start(self):
        if self.running:
            return
        self.proc = subprocess.Popen(self.cmd, start_new_session=True)

    def stop(self, sigint_timeout=10.0, sigterm_timeout=5.0):
        if not self.running:
            self.proc = None
            return
        pgid = os.getpgid(self.proc.pid)
        for sig, timeout in ((signal.SIGINT, sigint_timeout),
                             (signal.SIGTERM, sigterm_timeout),
                             (signal.SIGKILL, 2.0)):
            try:
                os.killpg(pgid, sig)
            except ProcessLookupError:
                break
            if self._wait(timeout):
                break
        self.proc = None

    def _wait(self, timeout):
        end = time.time() + timeout
        while time.time() < end:
            if not self.running:
                return True
            time.sleep(0.1)
        return not self.running


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--role', required=True)
    parser.add_argument('--launch-file', required=True)
    parser.add_argument('--state-topic', required=True)
    parser.add_argument('--mode-topic', default='/mission/mode')
    args, _ = parser.parse_known_args(argv)

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (QoSProfile, QoSDurabilityPolicy,
                           QoSReliabilityPolicy, QoSHistoryPolicy)
    from std_msgs.msg import String

    latched = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

    class MissionSupervisor(Node):
        def __init__(self):
            super().__init__(f'mission_supervisor_{args.role}')
            self.pm = ProcessManager(['ros2', 'launch', args.launch_file])
            self.state_pub = self.create_publisher(String, args.state_topic, latched)
            self.create_subscription(String, args.mode_topic, self._on_mode, latched)
            self._publish_state('idle')

        def _publish_state(self, s):
            self.state_pub.publish(String(data=s))

        def _on_mode(self, msg):
            action = decide_action(self.pm.running, msg.data.strip())
            if action == 'start':
                self.get_logger().info(f'[{args.role}] frontier requested -> launching')
                self._publish_state('frontier:starting')
                self.pm.start()
                self._publish_state('frontier:up')
            elif action == 'stop':
                self.get_logger().info(f'[{args.role}] manual requested -> tearing down')
                self._publish_state('frontier:stopping')
                self.pm.stop()
                self._publish_state('idle')

    rclpy.init(args=argv)
    node = MissionSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pm.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
