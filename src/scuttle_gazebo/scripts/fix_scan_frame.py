#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanFrameFixer(Node):
    def __init__(self):
        super().__init__('scan_frame_fixer')
        
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan_raw',
            self.scan_callback,
            10)
        
        self.publisher = self.create_publisher(LaserScan, '/scan', 10)
        
        self.get_logger().info('Scan frame fixer node started - remapping frame_id to lidar_1')
    
    def scan_callback(self, msg):
        # Change frame_id to match TF tree
        msg.header.frame_id = 'lidar_1'
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ScanFrameFixer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
