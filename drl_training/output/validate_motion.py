#!/usr/bin/env python3
"""Validate SCUTTLE diff_drive + lidar in Gazebo Classic:
command /cmd_vel and check whether /odom and /scan change."""
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class Validator(Node):
    def __init__(self):
        super().__init__('motion_validator')
        self.pub = self.create_publisher(Twist, 'cmd_vel', QoSProfile(depth=10))
        self.odom = None
        self.scan = None
        self.create_subscription(Odometry, 'odom', self._odom, QoSProfile(depth=10))
        self.create_subscription(LaserScan, 'scan', self._scan, qos_profile_sensor_data)

    def _odom(self, m):
        p = m.pose.pose.position
        self.odom = (round(p.x, 4), round(p.y, 4))

    def _scan(self, m):
        self.scan = [r for r in m.ranges]


def wait_for_data(n, timeout=10):
    t0 = time.time()
    while (n.odom is None or n.scan is None) and time.time() - t0 < timeout:
        rclpy.spin_once(n, timeout_sec=0.1)


rclpy.init()
n = Validator()
wait_for_data(n)
print("initial /odom:", n.odom)
print("/scan received:", n.scan is not None,
      "| len:", len(n.scan) if n.scan else 0,
      "| min:", round(min(n.scan), 3) if n.scan else None)
o0 = n.odom
s0 = list(n.scan) if n.scan else None

# Command forward + turn for 8 seconds
tw = Twist()
tw.linear.x = 0.3
tw.angular.z = 0.5
print(">>> commanding cmd_vel linear.x=0.3 angular.z=0.5 for 8s ...")
t0 = time.time()
while time.time() - t0 < 8:
    n.pub.publish(tw)
    rclpy.spin_once(n, timeout_sec=0.05)
    time.sleep(0.05)
n.pub.publish(Twist())  # stop
for _ in range(10):
    rclpy.spin_once(n, timeout_sec=0.1)

print("final   /odom:", n.odom)
print("final   /scan min:", round(min(n.scan), 3) if n.scan else None)
if o0 and n.odom:
    moved = abs(n.odom[0] - o0[0]) + abs(n.odom[1] - o0[1])
    print("ODOM delta:", round(moved, 4), "=>", "ROBOT MOVED ✓" if moved > 0.02 else "NOT MOVING ✗")
else:
    print("ODOM: could not read (no /odom data) ✗")
if s0 and n.scan:
    changed = sum(1 for a, b in zip(s0, n.scan) if abs(a - b) > 0.01)
    print("SCAN changed buckets:", changed, "/", len(n.scan),
          "=>", "SCAN LIVE ✓" if changed > 0 else "scan static")
n.destroy_node()
rclpy.shutdown()
