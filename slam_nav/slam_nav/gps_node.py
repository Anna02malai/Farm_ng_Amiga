import os
import sys
import math
import rclpy
import pyproj
import serial
import datetime
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point

# ---------------------------------- Define RTK fix type names ---------------------------------- #
fix_type_names = {
    '0': 'Invalid, no position available',
    '1': 'Autonomous GPS fix, no correction data used',
    '2': 'DGPS fix, using a local DGPS base station or correction service such as WAAS or EGNOS',
    '3': 'PPS fix',
    '4': 'RTK fix, high accuracy Real Time Kinematic',
    '5': 'RTK Float, better than DGPS, but not quite RTK',
    '6': 'Estimated fix (dead reckoning)',
    '7': 'Manual input mode',
    '8': 'Simulation mode',
    '9': 'WAAS fix (not NMEA standard, but NovAtel receivers report this instead of a 2)'
}

# ------------------------------------------- GPS Node ------------------------------------------ #
class gps_node(Node):

    def __init__(self):
        super().__init__('gps_node')
        # ------------------------------------ Name argument ------------------------------------ #
        if len(sys.argv) > 1:
            self.name_argument = sys.argv[1]
        else:
            self.name_argument = 'default_name'  # Use a default name if 'name_argument' is not provided.
        self.get_logger().info(f"GPS node initialized with name argument '{self.name_argument}'")

        # -------------------------------- Serial communication --------------------------------- #      
        port = '/dev/serial/by-id/usb-Emlid_ReachRS2_8243D27C83B34ACE-if02'
        baud_rate = 115200  # Update with the appropriate baud rate
        self.ser = serial.Serial(port, baud_rate)
        # ---------------------------------- Loading Waypoints ---------------------------------- #
        # Load the text file from the GPS_waypoints folder
        folder_name = "1_GPS_Waypoints"
        file_name = "GPS_Waypoints.txt"
        file_path = os.path.join(os.getcwd(), folder_name, file_name)
        self.GPS_waypoints_lat_long = np.genfromtxt(file_path, delimiter=',', usecols=(0, 2))        
        self.calculate_UTM_waypoints()
        
        # -------------------------------- Saving GPS trajectory -------------------------------- #
        # Create the folder to store GPS files
        folder_name = "3_Trajectory_GPS_data"
        NVME_addrss = '/mnt/8534d304-519c-4e47-bad5-9ceaf74d5753'
        name_arg = 'Aug2023_Aiken_data_collection'
        folder_path = os.path.join(NVME_addrss, name_arg, folder_name)
        os.makedirs(folder_path, exist_ok=True)
        # Open a file to save the data
        timestamp = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
        file_name = f"gps_data_{timestamp}_{self.name_argument}.txt"
        file_path = os.path.join(folder_path, file_name)
        self.file = open(file_path, "w")
        # ------------------------------------- Parameters -------------------------------------- #
        self.Waypoint_index = 0
        self.timestamp = self.latitude = self.nmea_latitude = None
        self.nmea_longitude = self.longitude = self.gps_speed = None
        self.track_angle = self.fix_type_code = self.fix_type = None
        self.previous_easting = self.previous_northing = None

        # ------------------------------------ Publishments ------------------------------------- #
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self.desired_heading_angle_publisher = self.create_publisher(Point, 'GPS_desired_heading_angle', 1)
        self.calculated_heading_angle_publisher = self.create_publisher(Point, 'GPS_calculated_heading_angle', 1)
        self.track_angle_publisher = self.create_publisher(Point, 'GPS_track_angle', 5)

        self.main_loop_GPS()

    def main_loop_GPS(self):
        
        while True:
            try:
                self.get_GPS_data()

            except KeyboardInterrupt:
                # Close the serial connection and the file
                self.shutdown()

    def get_GPS_data(self):
        # ------------------------------------- Read values ------------------------------------- #                
        line = self.ser.readline().decode().strip()
        self.file.write(f"{line}\n")
        if line.startswith('$GNRMC'):
            self.decode_RMC_message(line)
                                                                                        
        elif line.startswith('$GNGGA'):
            self.decode_GGA_message(line)

    # ----------------------------------- Process NMEA messages --------------------------------- #
    def process_NMEA_message(self):
        self.latitude = self.convert_lat_long_format(self.nmea_latitude)
        self.longitude = self.convert_lat_long_format(self.nmea_longitude)                            
        current_easting, current_northing = self.convert_to_utm(self.latitude, -self.longitude)
        self.calculate_heading_direction(current_easting, current_northing)
        easting_difference = self.UTM_waypoints_easting_northing[self.Waypoint_index][0] - current_easting # in meters
        northing_difference = self.UTM_waypoints_easting_northing[self.Waypoint_index][1] - current_northing # in meters
        desired_angle_radians = math.atan2(northing_difference, easting_difference) # in radian, with respect to east (verified)
        desired_angle_degrees = math.degrees(desired_angle_radians)
        self.get_logger().info(f"easting difference: {easting_difference}")
        self.get_logger().info(f"northing difference: {northing_difference}")
        self.get_logger().info(f"Desired angle from east: {desired_angle_degrees}")

        # ------------------------------------ Publish values ----------------------------------- #
        # When fix_type_code of 4 is available, change the other to sth other than 1 #
        if self.fix_type_code == '4' or self.fix_type_code == '1':
            self.publish_desired_heading_angle(desired_angle_radians)
            self.publish_calculated_heading_angle()
            self.publish_track_angle(self.track_angle)
            
                                            
        if abs(easting_difference) < 0.05 and abs(northing_difference) < 0.05:
            self.get_logger().info("GOAL REACHED!")
            self.Waypoint_index += 1
            self.publish_velocity(0,0)

    def shutdown(self):
        # Close the serial connection and the file
        self.ser.close()
        self.file.close()

    def print_attributes(self):
        attributes = {
            "Timestamp": self.timestamp,
            "Latitude": self.latitude,
            "Longitude": self.longitude,
            "Speed": self.gps_speed,
            "Track angle": self.track_angle,
            "Fix type": self.fix_type}
        
        for name, value in attributes.items():
            self.get_logger().info(f"{name}: {value}")


    def decode_RMC_message(self, line):
        RMC_values = line.split(',')
        
        if len(RMC_values) >= 2:
            self.timestamp = RMC_values[1]
            self.nmea_latitude = RMC_values[3]
            self.nmea_longitude = RMC_values[5]
            self.gps_speed = RMC_values[7]
            self.track_angle = RMC_values[8]

    def decode_GGA_message(self, line):
        GGA_values = line.split(',')
        if len(GGA_values) >= 1 and GGA_values[1] == self.timestamp:
            self.fix_type_code = GGA_values[6]
            self.fix_type = fix_type_names.get(self.fix_type_code, 'Unknown')
        # -------------------------------- Check waypoint index --------------------------------- #
        if self.Waypoint_index >= (self.GPS_waypoints_lat_long.shape[0]):
            self.get_logger().info("Waypoints fully covered.")
            self.publish_velocity(0,0)      
        else:
            self.get_logger().info(
                f"Target waypoint: {self.Waypoint_index + 1}/{self.GPS_waypoints_lat_long.shape[0]}")  
            pass                                      
            if not self.nmea_latitude or not self.nmea_longitude:
                self.get_logger().info("GPS data not available")
                self.latitude = None # To be shown in printing 
                self.longitude = None # To be shown in printing
            else:
                self.process_NMEA_message()
        self.print_attributes()


    # --------------------------------- Calculate heading angle --------------------------------- #
    def calculate_heading_direction(self, current_easting, current_northing):
        if current_easting is None or current_northing is None:
            self.calculated_heading_angle_rad = None
            self.get_logger().info("GPS data is missing. Cannot calculate heading direction.")
            return
        if self.previous_easting is None or self.previous_northing is None:
            self.calculated_heading_angle_rad = None
            self.get_logger().info("Previous GPS data is missing. Cannot calculate heading direction.")
            self.previous_easting = current_easting
            self.previous_northing = current_northing
            return
        distance_lower_threshold = 0.15
        distance_higher_threshold = 2
        distance = math.sqrt((current_easting - self.previous_easting) ** 2 + (current_northing - self.previous_northing) ** 2)
        if distance < distance_lower_threshold:
            self.calculated_heading_angle_rad = None
            self.get_logger().info("Distance threshold not met")
            return
        if distance > distance_higher_threshold:
            self.calculated_heading_angle_rad = None
            self.get_logger().info("Too much direction error")
            self.previous_easting = current_easting
            self.previous_northing = current_northing
            return        
        dx = current_easting - self.previous_easting
        dy = current_northing - self.previous_northing
        self.calculated_heading_angle_rad = math.atan2(dy, dx) # in rad with respect to east
        calculated_heading_angle_deg = (self.calculated_heading_angle_rad * 180.0) / math.pi
        self.get_logger().info(f"Calculated heading angle in degrees from east: {calculated_heading_angle_deg}")


    # --------------------------------------- Conversions --------------------------------------- #
    def convert_lat_long_format(self, nmea_val):
        degrees = int(float(nmea_val) // 100)  # Extract the degree portion
        minutes = (float(nmea_val) % 100) / 60  # Convert the minute portion to decimal
        converted_position_val = degrees + minutes
        return converted_position_val

    def calculate_UTM_waypoints(self):
        self.UTM_waypoints_easting_northing = np.zeros_like(self.GPS_waypoints_lat_long)
        for i in range(self.GPS_waypoints_lat_long.shape[0]):
            latitude = self.GPS_waypoints_lat_long[i, 0]
            longitude = self.GPS_waypoints_lat_long[i, 1]
            easting, northing = self.convert_to_utm(latitude, -longitude) # in meters
            self.UTM_waypoints_easting_northing[i, 0] = easting 
            self.UTM_waypoints_easting_northing[i, 1] = northing  

    def convert_to_utm(self,latitude, longitude):
        # Define the UTM zone based on the longitude
        utm_zone = int((longitude + 180) / 6) + 1
        #print (f"Current UTM zone: {utm_zone}")
        # Create a pyproj transformer for the conversion
        utm_proj = pyproj.Proj(proj='utm', zone=utm_zone, ellps='WGS84')
        # Convert the latitude and longitude to UTM coordinates
        easting, northing = utm_proj(longitude, latitude)
        return easting, northing

    # -------------------------------------- Publishments --------------------------------------- #
    def publish_velocity(self, x, z):
            command_message = Twist()
            command_message.linear.x = float(x)
            command_message.angular.z = float(z)
            self.velocity_publisher.publish(command_message) 
    
    def publish_track_angle(self, track_angle):
            track_anlge_message = Point()            
            if track_angle != '':
                track_angle_deg = float(track_angle)
                track_angle_rad_from_east = math.radians(track_angle_deg + 90.0)
                # Wrap the angle within the range of -π to π
                track_angle_rad_from_east = (track_angle_rad_from_east + math.pi) % (2 * math.pi) - math.pi
                track_anlge_message.x = track_angle_rad_from_east
                self.track_angle_publisher.publish(track_anlge_message)                        

    def publish_desired_heading_angle(self, heading_angle):
            heading_anlge_message = Point()            
            heading_anlge_message.x = heading_angle
            self.desired_heading_angle_publisher.publish(heading_anlge_message) 

    def publish_calculated_heading_angle(self):
            heading_anlge_message = Point()            
            if self.calculated_heading_angle_rad != None:
                heading_anlge_message.x = self.calculated_heading_angle_rad
                self.calculated_heading_angle_publisher.publish(heading_anlge_message) 


def main(args=None):
    rclpy.init(args=args)
    GPS_Node = gps_node()
    rclpy.spin(GPS_Node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()


###################################################################################################
############################## Reach RS3 Serial Connection with USB ###############################
# REACH RS3 USB Connection:                                                                       #
# The name and password of the Router is written on its bottom.                                   #
# The Emlid RS3 can be connected to Jetson using USB Cable                                        #
#                                                                                                 #
# Settings for USB connection to RS3:                                                             #
# 1- Connect the Jetson and the RS3 to the same Wifi (by the router)                              #
# 2- Search the IP address of the GPS receiver on Jetson: http://192.168.0.190/                   #
# 3- Go to "Position output" tab, and enable Output2, define the Output as Serial, and configure  #
#    the Device as "USB-to-PC".                                                                   #
#    Next, put the Baud rate as "115200". For the format, use RMC and GGA from NMEA               #
# 4- On the Jetson, get the position from Serial connection.                                      #
###################################################################################################