#!/usr/bin/env python3
"""Bridge /gas_sensor (std_msgs/Float32 from gz bridge) → /gdm/gas_1 (olfaction_msgs/GasSensor)."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from olfaction_msgs.msg import GasSensor


class GasSensorBridge(Node):
    def __init__(self):
        super().__init__('gas_sensor_bridge')
        self.pub = self.create_publisher(GasSensor, '/gdm/gas_1', 10)
        self.create_subscription(Float32, '/gas_sensor', self._cb, 10)

    def _cb(self, msg: Float32):
        out = GasSensor()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        out.technology = GasSensor.TECH_MOX
        out.raw = float(msg.data)
        out.raw_units = GasSensor.UNITS_PPM
        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(GasSensorBridge())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
