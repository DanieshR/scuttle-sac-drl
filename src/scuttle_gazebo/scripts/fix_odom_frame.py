#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class OdomFrameFixer(Node):
    def __init__(self):
        super().__init__('odom_frame_fixer')
        
        self.subscription = self.create_subscription(
            Odometry,
            '/odom_raw',
            self.odom_callback,
            10)
        
        self.publisher = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.get_logger().info('Odom frame fixer started - remapping to odom->base_link and publishing TF')
    
    def odom_callback(self, msg):
        # Fix frame_ids
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        
        # Publish the fixed odom message
        self.publisher.publish(msg)
        
        # Publish TF transform
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'scuttle/odom'
        t.child_frame_id = 'scuttle/base_link'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomFrameFixer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
