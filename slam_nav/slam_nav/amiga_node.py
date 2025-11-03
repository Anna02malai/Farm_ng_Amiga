#!/usr/bin/env python3

import rclpy
import serial
import subprocess
from rclpy.node import Node
from geometry_msgs.msg import Twist

class amiga_node(Node):

    def __init__(self, device_id):
        super().__init__('amiga_node')
        # ------------------------------------ Timer ------------------------------------ #
        timer_period = 0.1  # seconds, frequency of communication with Amiga robot
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # ---------------------------------- Parameters --------------------------------- #
        ###############
        self.max_robot_speed = 0.75 #m/s
        ###############

        self.max_robot_turn = 0.5 #rad/s
        self.robot_control_state = 1 # 1 -> State: AutoActive
        self.increment_test = 0.1
        self.start_speed = 0.1
        # ------------------------------ Serial Communication --------------------------- #
        self.ser = serial.Serial()
        #self.ser.baudrate = 115200
        self.ser.baudrate = 9600
        self.ser.port = device_id
        self.ser.timeout = 10
        # ------------------------------- Subscriptions --------------------------------- #
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
    
    def timer_callback(self):
        pass

    def cmd_vel_callback(self, twist):
        linear_speed = twist.linear.x
        angular_speed = twist.angular.z
        self.get_logger().info(f"Received linear speed: {linear_speed}")
        self.get_logger().info(f"Received angular speed: {angular_speed}")
        self.send_command_to_robot(linear_speed, angular_speed)
    
    def send_command_to_robot(self, linear_speed, angular_speed):

        #Bound the linear_speed, angular_speed by max_robot_speed, max_robot_turn
        linear_speed = max(-self.max_robot_speed, min(self.max_robot_speed, linear_speed))
        angular_speed = max(-self.max_robot_turn, min(self.max_robot_turn, angular_speed))

        self.get_logger().info(f"Linear speed: {linear_speed}")
        self.get_logger().info(f"Angular speed: {angular_speed}")
        self.ser.open()
        command = f'$MOV,{linear_speed},{angular_speed}\n'.encode()
        # command = f'[{self.robot_control_state},{linear_speed},{angular_speed}]\r\n'.encode()
        self.ser.write(command)
        self.ser.close()
        pass

def main(args=None):
    rclpy.init(args=args)
    device_id = "/dev/ttyACM0"
    Amiga_Node = amiga_node(device_id)
    rclpy.spin(Amiga_Node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


