#!/usr/bin/env python3

import sys
import select
import tty
import termios
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class TeleopAutoStop(Node):
    def __init__(self):
        super().__init__('teleop_autostop')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.linear = 0.0
        self.angular = 0.0
        self.speed = 0.5
        self.turn = 1.0

        self.last_key_time = time.time()
        self.timeout = 0.1  # Auto-stop after 0.1 seconds of no input

        # Track last movement key to allow simple combination (e.g. W then A)
        self.last_move_key = None
        self.combo_window = 0.2  # seconds in which keys are considered a combo

        self.get_logger().info('Teleop Auto-Stop started')
        print("""
Control Your Robot!
---------------------------
  W : forward
  S : backward
  A : rotate left
  D : rotate right
  Space : force stop

  W + A : forward and left
  W + D : forward and right

  R/F : increase/decrease speed
  T/G : increase/decrease turn rate

RELEASES AUTOMATICALLY - no need to press Space!
CTRL-C to quit
        """)

    def timer_callback(self):
        # Auto-stop if no key pressed recently
        if time.time() - self.last_key_time > self.timeout:
            self.linear = 0.0
            self.angular = 0.0

        twist = Twist()
        twist.linear.x = self.linear
        twist.angular.z = self.angular
        self.publisher.publish(twist)

    def set_motion(self, move_key, second_key=None):
        """
        Map key (or key pair) to linear/angular velocities.
        W/S: linear
        A/D: angular
        Combos: W+A, W+D = forward with turn.
        """
        # Normalize to lowercase for simplicity
        if move_key:
            move_key = move_key.lower()
        if second_key:
            second_key = second_key.lower()

        # Default
        linear = 0.0
        angular = 0.0

        # Handle combos only when W is involved
        keys = {move_key}
        if second_key:
            keys.add(second_key)

        if 'w' in keys:
            linear = self.speed
        elif 's' in keys:
            linear = -self.speed

        if 'a' in keys and 'd' not in keys:
            angular = self.turn
        elif 'd' in keys and 'a' not in keys:
            angular = -self.turn

        self.linear = linear
        self.angular = angular

    def handle_key(self, key):
        self.last_key_time = time.time()

        # Normalize letters to handle upper/lower case
        key = key.lower()

        # Motion keys
        motion_keys = {'w', 'a', 's', 'd'}

        if key in motion_keys:
            # Check for combination with previous motion key
            now = time.time()
            if (
                self.last_move_key is not None and
                (now - self.last_key_time) < self.combo_window and
                self.last_move_key != key
            ):
                # Combine last motion key with current
                self.set_motion(self.last_move_key, key)
            else:
                # Single key motion
                self.set_motion(key)
            self.last_move_key = key

        elif key == ' ':
            # Force stop
            self.linear = 0.0
            self.angular = 0.0
            self.last_move_key = None

        # Speed and turn tuning (optional remap)
        elif key == 'r':  # increase speed
            self.speed = min(self.speed + 0.1, 2.0)
            print(f"Speed: {self.speed:.1f} m/s")
        elif key == 'f':  # decrease speed
            self.speed = max(self.speed - 0.1, 0.1)
            print(f"Speed: {self.speed:.1f} m/s")
        elif key == 't':  # increase turn rate
            self.turn = min(self.turn + 0.2, 3.0)
            print(f"Turn rate: {self.turn:.1f} rad/s")
        elif key == 'g':  # decrease turn rate
            self.turn = max(self.turn - 0.2, 0.2)
            print(f"Turn rate: {self.turn:.1f} rad/s")


def get_key(settings):
    if select.select([sys.stdin], [], [], 0.1)[0]:
        return sys.stdin.read(1)
    return None


def main():
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    node = TeleopAutoStop()

    try:
        tty.setraw(sys.stdin.fileno())
        while rclpy.ok():
            key = get_key(settings)
            if key == '\x03':  # Ctrl+C
                break
            if key:
                node.handle_key(key)
            rclpy.spin_once(node, timeout_sec=0.01)
    except Exception as e:
        print(e)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

        # Send final stop command
        stop = Twist()
        node.publisher.publish(stop)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
