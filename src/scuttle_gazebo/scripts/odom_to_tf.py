#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Pose, Quaternion
from tf2_ros import TransformBroadcaster


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quaternion_from_yaw(yaw):
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf')

        self.tf_broadcaster = TransformBroadcaster(self)

        # Episode-origin odom offset.
        # Diff-drive odom accumulates indefinitely in Gazebo Harmonic — it never
        # resets when the robot is teleported back to (0,0) between episodes.
        # We capture the raw odom at each episode start and subtract it so that
        # the published TF always shows the robot relative to its spawn position.
        self.x_off = 0.0
        self.y_off = 0.0
        self.yaw_off = 0.0
        self.needs_reset = False

        latch_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Pose, 'goal_pose', self._on_goal, latch_qos)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)

        self.get_logger().info('Odom to TF node started')

    def _on_goal(self, _msg):
        self.needs_reset = True

    def _on_odom(self, msg):
        raw_x   = msg.pose.pose.position.x
        raw_y   = msg.pose.pose.position.y
        raw_yaw = yaw_from_quaternion(msg.pose.pose.orientation)

        if self.needs_reset:
            self.x_off   = raw_x
            self.y_off   = raw_y
            self.yaw_off = raw_yaw
            self.needs_reset = False
            self.get_logger().info(
                f'Odom reset: offset x={raw_x:.3f} y={raw_y:.3f} yaw={math.degrees(raw_yaw):.1f}°')

        cx = raw_x - self.x_off
        cy = raw_y - self.y_off
        cyaw = raw_yaw - self.yaw_off
        while cyaw >  math.pi: cyaw -= 2 * math.pi
        while cyaw < -math.pi: cyaw += 2 * math.pi

        t = TransformStamped()
        t.header.stamp    = msg.header.stamp
        t.header.frame_id = msg.header.frame_id
        t.child_frame_id  = msg.child_frame_id
        t.transform.translation.x = cx
        t.transform.translation.y = cy
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation      = quaternion_from_yaw(cyaw)

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
