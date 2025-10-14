#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class vel_node(Node):
    def __init__(self):
        super().__init__('vel_node')

        # Publisher to cmd_vel (standard velocity topic)
        self.__publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        #Timer at 10 Hz
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("vel_node started, publishing Twist messages...")

    def timer_callback(self, x = 0.5, z = 0.5):
        command_msg = Twist()
        command_msg.linear.x = float(x)
        command_msg.angular.z = float(z)

        self.__publisher.publish(command_msg)
        self.get_logger().info(f"linear velocity is {float(x)} and angular velocity is {float(z)}")


def main(args=None):
    rclpy.init(args=args)
    node = vel_node()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()


