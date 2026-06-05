#!/usr/bin/env python3
"""
Odom reset monitor — detects whether Gazebo Harmonic's diff_drive plugin
correctly zeros /odom after world reset (reset:{all:true}).

A "reset event" is inferred when the robot position suddenly jumps more than
JUMP_THRESHOLD metres in a single callback (the physics reset teleports the
robot back to spawn; if odom resets too, the position should be near zero).

Run standalone:
  ros2 run scuttle_gazebo odom_reset_monitor.py

Output flags:
  [OK]   odom returned near (0,0) after reset — diff_drive reset correctly
  [WARN] odom carried an offset after reset — persistent drift detected
         This means goal_dist / goal_angle in the environment node are wrong.
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

JUMP_THRESHOLD   = 0.5   # m — sudden position change signals a reset
OFFSET_THRESHOLD = 0.15  # m — odom value after reset that indicates a drift bug
WINDOW           = 5     # messages to average after a detected reset


class OdomResetMonitor(Node):
    def __init__(self):
        super().__init__('odom_reset_monitor')
        self.prev_x = None
        self.prev_y = None
        self.post_reset_buf = []
        self.waiting_post = False
        self.reset_count   = 0
        self.drift_count   = 0

        self.sub = self.create_subscription(
            Odometry, '/odom', self._odom_cb, 10)
        self.get_logger().info('Odom reset monitor started — watching /odom')

    def _odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.prev_x is None:
            self.prev_x, self.prev_y = x, y
            return

        dx = x - self.prev_x
        dy = y - self.prev_y
        dist = math.sqrt(dx*dx + dy*dy)

        if dist > JUMP_THRESHOLD:
            # Likely a reset event — start collecting post-reset samples
            self.waiting_post   = True
            self.post_reset_buf = []
            self.reset_count   += 1
            self.get_logger().info(
                f'[RESET #{self.reset_count}] Jump of {dist:.2f} m detected '
                f'({self.prev_x:.3f},{self.prev_y:.3f}) → ({x:.3f},{y:.3f})')

        if self.waiting_post:
            self.post_reset_buf.append((x, y))
            if len(self.post_reset_buf) >= WINDOW:
                avg_x = sum(p[0] for p in self.post_reset_buf) / WINDOW
                avg_y = sum(p[1] for p in self.post_reset_buf) / WINDOW
                offset = math.sqrt(avg_x**2 + avg_y**2)
                if offset > OFFSET_THRESHOLD:
                    self.drift_count += 1
                    self.get_logger().warn(
                        f'[WARN] odom OFFSET after reset #{self.reset_count}: '
                        f'avg=({avg_x:.3f},{avg_y:.3f})  |offset|={offset:.3f} m  '
                        f'— diff_drive did NOT reset. Goal distances will be wrong!')
                else:
                    self.get_logger().info(
                        f'[OK]   odom near zero after reset #{self.reset_count}: '
                        f'avg=({avg_x:.3f},{avg_y:.3f})  |offset|={offset:.3f} m')
                self.waiting_post = False

        self.prev_x, self.prev_y = x, y

    def summary(self):
        if self.reset_count == 0:
            return
        self.get_logger().info(
            f'Summary: {self.reset_count} resets observed, '
            f'{self.drift_count} had odom offset > {OFFSET_THRESHOLD} m')


def main():
    rclpy.init()
    node = OdomResetMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.summary()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
