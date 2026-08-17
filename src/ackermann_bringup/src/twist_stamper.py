#!/usr/bin/env python3
"""
Purpose:
    Nav2 publishes Twist messages in ROS Humble.
    ros2_control requires TwistStamped messages.
    This node converts Twist to TwistStamped.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class TwistStamper(Node):
    def __init__(self):
        super().__init__('twist_stamper')
        
        self.frame_id = self.declare_parameter('frame_id', 'base_footprint').value

        self.publisher_ = self.create_publisher(TwistStamped, 'cmd_vel_stamped', 10)
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.listener_callback, 10)
            
        self.get_logger().info(f'Twist Stamper initialized. Frame ID: {self.frame_id}')

    def listener_callback(self, msg):
        stamped_msg = TwistStamped()
        stamped_msg.header.stamp = self.get_clock().now().to_msg()
        stamped_msg.header.frame_id = self.frame_id
        stamped_msg.twist = msg
        self.publisher_.publish(stamped_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TwistStamper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
