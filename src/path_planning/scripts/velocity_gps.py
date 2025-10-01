#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import TwistWithCovarianceStamped
from std_msgs.msg import Float64


class VelocityMagnitudeNode(Node):
    def __init__(self):
        super().__init__('velocity_gps_node')
        
        self.gps_vel_sub = self.create_subscription(
            TwistWithCovarianceStamped,
            '/f9r/fix_velocity',
            self.gps_velocity_callback,
            10
        )

        # Publishers
        self.gps_mag_pub = self.create_publisher(
            Float64,
            '/f9r/current_speed',
            10
        )
        self.get_logger().info('Velocity node started')
        self.get_logger().info('Subscribing to /f9r/fix_velocity')
        self.get_logger().info('Publishing to /f9r/current_speed')
         
    def gps_velocity_callback(self, msg):
        # Extract linear velocity from GPS
        linear = msg.twist.twist.linear
        
        # Calculate magnitude (only x and y)
        magnitude = math.sqrt(linear.x**2 + linear.y**2)
        
        # Publish magnitude
        mag_msg = Float64()
        mag_msg.data = magnitude
        self.gps_mag_pub.publish(mag_msg)
        
        self.get_logger().debug(f'GPS velocity: {magnitude:.3f} m/s')


def main(args=None):
    rclpy.init(args=args)
    
    node = VelocityMagnitudeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()