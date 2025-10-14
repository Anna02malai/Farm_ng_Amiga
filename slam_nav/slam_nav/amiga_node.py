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

######################################################################

# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist

# # If you use pyserial, keep the import; otherwise mock/remove.
# import serial


# class AmigaNode(Node):
#     """
#     Listens on /cmd_vel and streams robot-friendly commands over serial.
#     - Opens the serial port once and keeps it open.
#     - Clamps speeds to configured maxima.
#     - Watchdog sends zero if no cmd_vel is received for a short time.
#     """

#     def __init__(self, device_id: str = "/dev/ttyACM0"):
#         super().__init__("amiga_node")

#         # -------------------- Parameters (you can also expose these via ROS params) --------------------
#         self.device_id = device_id
#         self.max_robot_speed = 0.75   # m/s  (linear x)
#         self.max_robot_turn  = 0.50   # rad/s (yaw z)
#         self.watchdog_timeout_s = 0.5 # stop if no cmd for this long
#         self.tx_rate_hz = 10.0        # timer frequency for watchdog & keepalive

#         # ----------------------------- State -----------------------------
#         self.last_cmd_time = self.get_clock().now()
#         self.last_sent_linear = 0.0
#         self.last_sent_angular = 0.0

#         # --------------------------- Serial Setup ------------------------
#         self.ser = serial.Serial()
#         self.ser.port = self.device_id
#         self.ser.baudrate = 9600
#         self.ser.timeout = 0.1  # short read/write timeout

#         try:
#             self.ser.open()
#             self.get_logger().info(f"Serial opened on {self.device_id}")
#         except Exception as e:
#             self.get_logger().error(f"Failed to open serial on {self.device_id}: {e}")

#         # ------------------------- ROS Interfaces ------------------------
#         self.subscription = self.create_subscription(
#             Twist, "cmd_vel", self.cmd_vel_callback, 10
#         )
#         self.timer = self.create_timer(1.0 / self.tx_rate_hz, self.timer_callback)

#         self.get_logger().info("amiga_node is up. Listening on /cmd_vel.")

#     # --------------------------------- Callbacks ---------------------------------
#     def cmd_vel_callback(self, twist: Twist):
#         # Clamp inputs
#         linear_speed  = max(-self.max_robot_speed, min(self.max_robot_speed, float(twist.linear.x)))
#         angular_speed = max(-self.max_robot_turn,  min(self.max_robot_turn,  float(twist.angular.z)))

#         self.last_cmd_time = self.get_clock().now()

#         # Send immediately upon receipt (low latency)
#         self.send_command_to_robot(linear_speed, angular_speed)

#     def timer_callback(self):
#         """Watchdog/keepalive: if we've not heard from /cmd_vel recently, send zeros."""
#         elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
#         if elapsed > self.watchdog_timeout_s:
#             # Only send zeros if we're not already stopped (avoid unnecessary serial writes)
#             if self.last_sent_linear != 0.0 or self.last_sent_angular != 0.0:
#                 self.get_logger().warn(
#                     f"No /cmd_vel for {elapsed:.2f}s (>{self.watchdog_timeout_s}s). Stopping."
#                 )
#                 self.send_command_to_robot(0.0, 0.0)

#     # ------------------------------- Hardware I/O -------------------------------
#     def send_command_to_robot(self, linear_speed: float, angular_speed: float):
#         if not self.ser or not self.ser.is_open:
#             self.get_logger().warn("Serial not open; dropping command")
#             return

#         # Format command (adjust to your robot’s protocol)
#         cmd = f"$MOV,{linear_speed:.3f},{angular_speed:.3f}\n".encode()

#         try:
#             self.ser.write(cmd)
#             self.last_sent_linear = linear_speed
#             self.last_sent_angular = angular_speed
#             # Optional: uncomment for detailed logs
#             # self.get_logger().info(f"TX -> {cmd.decode().strip()}")
#         except Exception as e:
#             self.get_logger().error(f"Serial write failed: {e}")

#     # ------------------------------- Shutdown Hooks ------------------------------
#     def destroy_node(self):
#         # Try to stop the robot on shutdown
#         try:
#             if self.ser and self.ser.is_open:
#                 self.ser.write(b"$MOV,0.000,0.000\n")
#         except Exception as e:
#             self.get_logger().warn(f"Failed to send stop on shutdown: {e}")

#         # Close serial
#         try:
#             if self.ser and self.ser.is_open:
#                 self.ser.close()
#                 self.get_logger().info("Serial closed.")
#         except Exception as e:
#             self.get_logger().warn(f"Failed to close serial: {e}")

#         super().destroy_node()


# def main(args=None):
#     rclpy.init(args=args)

#     # You can swap this for a declared ROS parameter if you prefer
#     device_id = "/dev/ttyACM0"

#     node = AmigaNode(device_id=device_id)
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == "__main__":
#     main()


#####################################################################################################################

