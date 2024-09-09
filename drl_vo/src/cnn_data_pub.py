#!/usr/bin/python3

# Usage: This script is modified to be compatible with ROS2 and handle Oradar MS200 lidar data.
# It processes pedestrian kinematic maps and lidar data, and publishes the processed data.

import numpy as np
import rclpy
from rclpy.node import Node
from cnn_msgs.msg import CNN_data
from geometry_msgs.msg import Point, Twist
from pedsim_msgs.msg import TrackedPersons
from sensor_msgs.msg import LaserScan
from scipy.interpolate import interp1d

# Lidar specification
LIDAR_FOV = 360 
LIDAR_RESOLUTION = 0.8
LIDAR_MIN_RANGE = 0.03
LIDAR_MAX_RANGE = 12.0
NUM_LIDAR_BEAMS = int(LIDAR_FOV / LIDAR_RESOLUTION)
LIDAR_FRAME_RATE = 10  # Frame rate of lidar scans in Hz

# Front-facing lidar scan indices for 180-degree FOV (wrapping around 0 degrees)
FRONT_FACING_PART_1_START = 337  # index corresponding to -90 degrees
FRONT_FACING_PART_1_END = 450    # end of the lidar array (360 degrees)
FRONT_FACING_PART_2_START = 0    # start of lidar array (0 degrees)
FRONT_FACING_PART_2_END = 113    # index corresponding to +90 degrees
NUM_FRONT_FACING_BEAMS = (FRONT_FACING_PART_1_END - FRONT_FACING_PART_1_START) + FRONT_FACING_PART_2_END

# Time steps for collecting historical data
NUM_TP = 10  # Number of timestamps for accumulating data
NUM_PEDS = 34 + 1  # Number of total pedestrians

class CnnData(Node):
    def __init__(self):
        super().__init__('cnn_data_pub_node')

        # Initialize data for publishing
        self.ped_pos_map = np.zeros((2, 80, 80))  # Cartesian velocity map for pedestrians
        self.scan = []  # Store processed lidar data for the front-facing 180-degree FOV
        self.scan_all = np.zeros(NUM_LIDAR_BEAMS)
        self.goal_cart = np.zeros(2)
        self.vel = np.zeros(2)

        # ROS2 subscriptions and publishers
        self.ped_sub = self.create_subscription(TrackedPersons, '/track_ped', self.ped_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.goal_sub = self.create_subscription(Point, '/cnn_goal', self.goal_callback, 10)
        self.vel_sub = self.create_subscription(Twist, '/mobile_base/commands/velocity', self.vel_callback, 10)
        self.cnn_data_pub = self.create_publisher(CNN_data, '/cnn_data', 10)

        # Timer for controlling data publishing rate
        self.rate = LIDAR_FRAME_RATE  # 10 Hz
        self.ts_cnt = 0  # Timestamp counter for historical data
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)

    # Callback to process pedestrian data and generate the pedestrian position map
    def ped_callback(self, trackPed_msg):
        # Reset pedestrian velocity map at each callback
        self.ped_pos_map_tmp = np.zeros((2, 80, 80))  # Reset to clear old data
        
        # Process pedestrian positions from TrackedPersons message
        if len(trackPed_msg.tracks) > 0:
            for ped in trackPed_msg.tracks:
                x = ped.pose.pose.position.x
                y = ped.pose.pose.position.y
                vx = ped.twist.twist.linear.x
                vy = ped.twist.twist.linear.y

                # Only consider pedestrians within 20m x 20m region in front of the robot
                if 0 <= x <= 20 and np.abs(y) <= 10:
                    # Convert the position into a 0.25m bin size grid (80x80 grid)
                    row = int(np.floor(x / 0.25))  # x-coordinate
                    col = int(np.floor(-(y - 10) / 0.25))

                    # Ensure bins are within valid range
                    row = min(row, 79)
                    col = min(col, 79)

                    # Update velocity map with pedestrian's velocity
                    self.ped_pos_map_tmp[0, row, col] = vx  # velocity in x-direction
                    self.ped_pos_map_tmp[1, row, col] = vy  # velocity in y-direction

    # Callback function for lidar scan data
    def scan_callback(self, laserScan_msg):
        # Extract lidar data and filter for valid points
        scan_data = np.array(laserScan_msg.ranges, dtype=np.float32)
        scan_data[np.isnan(scan_data)] = 0.
        scan_data[np.isinf(scan_data)] = 0.

        # Identify invalid values (zero means no reading) and perform interpolation
        valid_mask = (scan_data > LIDAR_MIN_RANGE) & (scan_data <= LIDAR_MAX_RANGE)

        if np.any(~valid_mask):  # If there are any invalid values
            valid_indices = np.where(valid_mask)[0]
            valid_scan_data = scan_data[valid_mask]

            # Perform linear interpolation to replace missing data
            interp_func = interp1d(valid_indices, valid_scan_data, kind='linear', bounds_error=False, fill_value='extrapolate')
            interpolated_scan = interp_func(np.arange(len(scan_data)))

            # Update the scan with interpolated data
            scan_data = np.clip(interpolated_scan, 0, LIDAR_MAX_RANGE)

        # Limit the data to the front-facing 180 degrees (combine two parts of the lidar array)
        front_facing_scan_part_1 = scan_data[FRONT_FACING_PART_1_START:FRONT_FACING_PART_1_END]
        front_facing_scan_part_2 = scan_data[FRONT_FACING_PART_2_START:FRONT_FACING_PART_2_END]
        front_facing_scan = np.concatenate((front_facing_scan_part_1, front_facing_scan_part_2))

        # Store the processed scan data
        self.scan_all = scan_data  # Store the full 360-degree scan
        self.scan_tmp = front_facing_scan  # Store the front-facing 180-degree scan

    # Callback function for goal position
    def goal_callback(self, goal_msg):
        self.goal_cart[0] = goal_msg.x
        self.goal_cart[1] = goal_msg.y

    # Callback function for velocity
    def vel_callback(self, vel_msg):
        self.vel[0] = vel_msg.linear.x  # linear velocity
        self.vel[1] = vel_msg.angular.z  # angular velocity

    # Timer callback to publish CNN data periodically
    def timer_callback(self):
        self.ts_cnt += 1
        self.scan.append(self.scan_tmp.tolist())

        # When enough data has been collected, publish the CNN data
        if self.ts_cnt >= NUM_TP:
            cnn_data = CNN_data()
            cnn_data.ped_pos_map = [float(val) for sublist in self.ped_pos_map_tmp for subb in sublist for val in subb]
            cnn_data.scan = [float(val) for sublist in self.scan for val in sublist]
            cnn_data.scan_all = self.scan_all
            cnn_data.goal_cart = self.goal_cart
            cnn_data.vel = self.vel

            # Publish the CNN data
            self.cnn_data_pub.publish(cnn_data)

            # Reset the scan history and time step counter
            self.ts_cnt = 0
            self.scan = []

def main(args=None):
    rclpy.init(args=args)
    node = CnnData()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
