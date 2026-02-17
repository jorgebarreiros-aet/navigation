#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TwistPublisher(Node):

    def __init__(self):
        super().__init__('twist_publisher')
        # Create a publisher for the /cmd_vel topic
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        # Create a timer that calls the publish_twist function every 0.5 seconds
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.publish_twist)
        self.get_logger().info('Twist Publisher Node Started')

    def publish_twist(self):
        msg = Twist()
        # Set linear velocity (forward)
        msg.linear.x = 0.5
        # Set angular velocity (rotation around Z axis)
        msg.angular.z = 0.5
        self.publisher_.publish(msg)
        self.get_logger().info(
            f'Publishing: linear.x={msg.linear.x}, angular.z={msg.angular.z}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = TwistPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
