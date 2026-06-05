#!/usr/bin/env python3
"""Read-only: watch /odom for ~12s and report if the robot is moving (under agent control)."""
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry


class Watch(Node):
    def __init__(self):
        super().__init__('odom_watch')
        self.samples = []
        self.create_subscription(Odometry, 'odom', self._cb, QoSProfile(depth=10))

    def _cb(self, m):
        p = m.pose.pose.position
        self.samples.append((round(p.x, 4), round(p.y, 4)))


rclpy.init()
n = Watch()
t0 = time.time()
while time.time() - t0 < 12:
    rclpy.spin_once(n, timeout_sec=0.2)

if not n.samples:
    print("NO /odom received")
else:
    xs = [s[0] for s in n.samples]
    ys = [s[1] for s in n.samples]
    span = (max(xs) - min(xs)) + (max(ys) - min(ys))
    print(f"/odom samples: {len(n.samples)} | first: {n.samples[0]} | last: {n.samples[-1]}")
    print(f"position span over 12s: {round(span,4)} =>", "ROBOT MOVING under agent ✓" if span > 0.02 else "robot static (likely paused/between episodes)")
n.destroy_node()
rclpy.shutdown()
