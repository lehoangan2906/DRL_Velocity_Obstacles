#!/usr/bin/python3

import numpy as np
import numpy.matlib
import random
import math
import gym
import time
import threading

import rclpy
from rclpy.node import Node

from gym import spaces
from gym.utils import seeding
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import GetModelState, SetModelState
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import Pose, Twist, Point, PoseStamped, PoseWithCovarianceStamped

from sensor_msgs.msg import LaserScan
from action_msgs.msg import GoalStatusArray
from track_ped_msgs.msg import TrackedPersons
from cnn_msgs.msg import CnnData

from scipy.interpolate import interp1d # For preprocessing the lidar data
from scipy.optimize import linprog, minimize # For optimization 

from rclpy.duration import Duration
from rclpy.exceptions import TimeoutException


class DRLNavEnv(Node):
    """
    Gazebo env converts standard openai gym methods into Gazebo commands

    To check any topic we need to have the simulations running, we need to do two things:
        1) Unpause the simulation: without that the stream of data doesn't flow. This is for simulations
        that are paused for whatever reason.
        2) If the simulation was running already for some reason, we need to reset the controllers.
        This has to do with the fact that some plugins with tf, don't understand the reset of the simulation and need to be reset to work properly.
    """

    def __init__(self):
        rclpy.init(args = None)
        Node.__init__(self, 'drl_nav_env')
        self.seed()

        # Robot parameters:
        self.ROBOT_RADIUS = 0.3
        self.GOAL_RADIUS = 0.3
        self.DIST_NUM = 10
        self.pos_valid_flag = True

        # Bumper
        self.bump_flag = False
        self.bump_num = 0

        # Reward:
        self.dist_to_goal_reg = np.zeros(self.DIST_NUM)
        self.num_iterations = 0

        # Action limits
        self.max_linear_speed = 0.5
        self.max_angular_speed = 0.6
        
        # Action space
        self.high_action = np.array([1, 1])
        self.low_action = np.array([-1, -1])
        self.action_space = spaces.Box(low=self.low_action, high=self.high_action, dtype=np.float32)

        # Observation space
        self.cnn_data = CnnData()
        self.ped_pos = []
        self.scan = []
        self.goal = []
        self.observation_space = spaces.Box(low=-1, high=1, shape=(19202,), dtype=np.float32)

        # info, initial position and goal position
        self.init_pose = Pose()
        self.curr_pose = Pose()
        self.curr_vel = Twist()
        self.goal_position = Point()
        self.info = {}

        # Episode done flag:
        self._episode_done = False

        # Goal reached flag:
        self._goal_reached = False

        # Reset flag:
        self._reset = True

        # vo algorithm:
        self.mht_peds = TrackedPersons()

        # Gazebo services
        self.gazebo_client = self.create_client(SetModelState, '/gazebo/set_model_state')
        self.get_model_state = self.create_client(GetModelState, '/gazebo/get_model_state')

        # ROS 2 Subscriptions
        qos = QoSProfile(depth = 10, reliability=QoSReliabilityPolicy.RELIABLE)
        self._map_sub = self.create_subscription(OccupancyGrid, "/map", self._map_callback, qos)
        self._cnn_data_sub = self.create_subscription(CnnData, "/cnn_data", self._cnn_data_callback, qos)
        self._robot_pos_sub = self.create_subscription(PoseStamped, "/robot_pose", self._robot_pose_callback, qos)
        self._robot_vel_sub = self.create_subscription(Odometry, '/odom', self._robot_vel_callback, qos)
        self._final_goal_sub = self.create_subscription(PoseStamped, "/move_base/current_goal", self._final_goal_callback, qos)
        self._goal_status_sub = self.create_subscription(GoalStatusArray, "/move_base/status", self._goal_status_callback, qos)
        self._ped_sub = self.create_subscription(TrackedPersons, "/track_ped", self._ped_callback, qos)

        # ROS 2 Publishers
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', qos)
        self._initial_goal_pub = self.create_publisher(PoseStamped, '/move_base_simple/goal', qos)
        self._initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', qos)

        # Ensure all system are ready
        self._check_all_systems_ready()


    # Env methods
    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        """
        Gives env an action to enter the next state, 
        obs, reward, done, info = env.step(action)
        """
        self._take_action(action)
        obs = self._get_observation()
        reward = self._compute_reward()
        done = self._is_done(reward)
        info = self._pose_information()
        return obs, reward, done, info

    def reset(self):
        """
        Resets the env to the initial state and returns the initial observation
        obs, info = env.reset()
        """
        self._reset_sim()
        obs = self._get_observation()
        info = self._post_information()
        return obs

    def close(self):
        """
        Function executed when closing the environment.
        Use it for closing GUIs and other systems that need closing

        :return:
        """
        rclpy.shutdown()

    def _reset_sim(self):
        """
        Resets the simulation to initial conditions
        """
        self._set_init()
        return True

    def _set_init(self):
        """
        Set initial condition for simulation
        """
        self._cmd_vel_pub.publish(Twist()) # Stop the robot by sending 0 velocity

        if self._reset:
            self._reset = False
            self._check_all_systems_ready()

            self.pos_valid_flag = False
            map = self.map 

            while not self.pos_valid_flag:
                seed_initial_pose = random.randint(0, 18)
                self._set_initial_pose(seed_initial_pose)
                time.sleep(4)
                x = self.curr_pose.position.x
                y = self.curr_pose.position.y
                radius = self.ROBOT_RADIUS
                self.pos_valid_flag = self._is_pos_valid(x, y, radius, map)

        goal_x, goal_y, goal_yaw = self._publish_random_goal()

        time.sleep(1)
        self._check_all_systems_ready()

        self.init_pose = self.curr_pose
        self.curr_pose = self.curr_pose
        self.goal_position.x = goal_x
        self.goal_position.y = goal_y

        self.pos_valid_flag = True
        self.bump_flag = False
        self.num_iterations = 0
        self.dist_to_goal_reg = np.zeros(self.DIST_NUM)
        self._episode_done = False

        return self.init_pose, self.goal_position
    
    def _set_initial_pose(self, seed_initial_pose):
        # Set initial pose of the robot

        poses = [
            [1, 1, 0], [14, 7, 1.5705], [1, 16, 0], [14, 22.5, -1.3113],
            [4, 4, 1.5705], [2, 9, 0], [30, 9, 3.14], [25, 17, 3.14],
            [5, 8, 0], [10, 12, 0], [14, 15, 1.576], [18.5, 15.7, 3.14],
            [18.5, 11.3, 3.14], [14, 11.3, 3.14], [12.5, 13.2, 0.78],
            [12.07, 16.06, 0], [21, 14, -1.576], [14, 22.5, 1.576], [18, 8.5, -1.576]
        ]

        pose = poses[seed_initial_pose]         # Randomly select initial pose
        self._pub_initial_model_state(*pose)   # Publish initial pose
        time.sleep(1)
        self._pub_initial_position(*pose)

    # check functions
    def _check_all_systems_ready(self):
        self._check_all_subscribers_ready() # Check if all subscribers are ready
        self._check_all_publishers_ready() # Check if all publishers are ready
        self._check_service_ready('/gazebo/set_model_state')
        return True
    

    def _check_all_subscribers_ready(self):
        """
        Check if all subscribers are ready
        """
        self._check_subscriber_ready("/map", OccupancyGrid)
        self._check_subscriber_ready("/cnn_data", CnnData)
        self._check_subscriber_ready("/robot_pose", PoseStamped)
        self._check_subscriber_ready("/move_base/status", GoalStatusArray)

    def _check_subscriber_ready(self, name, type, timeout=5.0):
        """
        Check if a subscriber is ready
        """
        var = None
        while var is None:
            try:
                var = rclpy.wait_for_message(name, type, timeout)
            except:
                pass
        return var
    
    def _check_all_publishers_ready(self):
        """
        Check if all publishers are ready
        """
        self._check_publisher_ready(self._cmd_vel_pub)
        self._check_publisher_ready(self._initial_goal_pub)
        self._check_publisher_ready('/gazebo/set_model_state')
        self._check_publisher_ready(self._initial_pose_pub)

    def _check_publisher_ready(self, obj):
        # Check if a publisher is ready
        while obj.get_subscription_count() == 0:
            pass
    
    def _check_service_ready(self, node, service_name, timeout=5.0):
        """
        Waits for a service to get ready in ROS2
        """
        self.get_logger().debug(f"Waiting for '{service_name}' to be READY...")
        try:
            # Start counting time for the timeout
            start_time = node.get_clock().now()
            while (node.get_clock().now() - start_time) < Duration(seconds=timeout):
                if node.get_service(service_name):
                    self.get_logger().debug(f"Service '{service_name}' is READY.")
                    return
                rclpy.spin_once(node, timeout_sec=0.1)  # Short sleep to allow other processes

            raise TimeoutException(f"Service '{service_name}' is not available after waiting for {timeout} seconds.")
        except TimeoutException as e:
            self.get_logger().fatal(f"Service '{service_name}' unavailable: {str(e)}")



    # callback functions for ROS2
    # ========================================================================= #

    def _map_callback(self, map_msg):
        # Callback function to get the map
        self.map = map_msg

    def _cnn_data_callback(self, cnn_data_msg):
        # callback function to get the CNN data
        self.cnn_data = cnn_data_msg

    def _robot_pose_callback(self, robot_pose_msg):
        # callback function to get the robot pose
        self.curr_pose = robot_pose_msg.pose

    def _robot_vel_callback(self, robot_vel_msg):
        # callback function to get the robot velocity
        self.curr_vel = robot_vel_msg.twist.twist

    def _final_goal_callback(self, final_goal_msg):
        # callback function to get the final goal
        self.goal_position = final_goal_msg.pose.position

    def _goal_status_callback(self, goal_status_msg):
        # callback function to get the goal status
        if len(goal_status_msg.status_list) > 0:
            last_element = goal_status_msg.status_list[-1]
            if last_element.status == 3:
                self._goal_reached = True
            else:
                self._goal_reached = False
        else:
            self._goal_reached = False
    
    def _ped_callback(self, trackPed_msg):
        # callback function to get the pedestrian data
        self.mht_peds = trackPed_msg


    # ========================================================================= #
    # Publisher functions for publishing informations about the robot and goal position

    # Publisher functions to set and publish the initial state of the robot in the environment
    def _pub_initial_model_state(self, x, y, theta):
        # Create a ModelState message to represent the robot's state in Gazebo
        robot_state = ModelState()

        # Specify the model name of the robot to be controlled (in this case: mobile_base)
        robot_state.model_name = "mobile_base" # Get the pose/twist relative to the frame of the model_base
        
        # Set the robot's initial position in the Gazebo world (x, y, z coordinates)
        robot_state.pose.position.x = x
        robot_state.pose.position.y = y
        robot_state.pose.position.z = 0 # The robot is assumed to be on the ground

        # Convert the yaw angle (theta) to quaternion for setting the robot's orientation
        # Gazebo uses quaternions to represent orientation in 3D space
        robot_state.pose.orientation.x = 0  # No roll
        robot_state.pose.orientation.y = 0  # No pitch
        robot_state.pose.orientation.z = np.sin(theta/2)    # Yaw converted to quaternion
        robot_state.pose.orientation.w = np.cos(theta/2)    # Quaternion w component

        # Set the reference frame for the robot, which is "world" in Gazebo
        robot_state.reference_frame = "world"

        # Attempt to call the Gazebo service to update the model's state
        # This service request will plae the robot at the specified location in the Gazebo world
        try:
            result = self.gazebo_client.call(robot_state) # Call the Gzebo client to update the robot's state
        except rclpy.ServiceException:
            # If the Gazebo service fails (e.g., Gazebo is not available), handle the exception.
            pass

    # Function to publish the initial position of the robot (x, y, theta) in the environment --> for localization
    def _pub_initial_position(self, x, y, theta):
        # Create a PoseWithCovarienaceStamped message to represent the robot's pose with uncertainty (covariance)
        initial_pose = PoseWithCovarianceStamped()

        # Set the frame of reference for this pose to "map", which is commonly used in ROS2 for navigation
        initial_pose.header.frame_id = "map"

        # Get the current time and add it to the header (important for time synchronization)
        initial_pose.header.stamp = self.get_clock().now().to_msg()

        # Set the robot's position (x, y, z coordinates)
        initial_pose.pose.pose.position.x = x
        initial_pose.pose.pose.position.y = y
        initial_pose.pose.pose.position.z = 0 # Assuming the robot is on the ground (z = 0)

        # Convert the yaw angle (theta) to quaternion for setting the robot's orientation
        initial_pose.pose.pose.orientation.x = 0    # No roll
        initial_pose.pose.pose.orientation.y = 0    # No pitch
        initial_pose.pose.pose.orientation.z = np.sin(theta/2) # Yaw converted to quaternion (z-axis rotation)
        initial_pose.pose.pose.orientation.w = np.cos(theta/2) # Quaternion w component

        # Publish the initial pose message to the ROS topic that handles localization (e,g., for AMCL)
        self._initial_pose_pub.publish(initial_pose)

    # Function to randomly publish the goal position (x, y, theta) for the robot in the environment --> for global planner
    def _publish_random_goal(self):
        """
        The function randomly selects a goal position (x, y, theta) on the map for the robot, 
        ensuring that the goal is within a certain distance range from the robot's current position.
        It then publishes this goal to be used by the robot's navigation system.
        """

        dis_diff = 21 
        while dis_diff >= 7 or dis_diff < 4.2:
            x, y, theta = self._get_random_pos_on_map(self.map)
            dis_diff = np.linalg_norm(
                np.array([self.curr_pose.position.x - x, self.curr_pose.position.y - y])
            )
        self._publish_goal(x, y, theta)
        return x, y, theta
    
    # Function to publish a goal position and orientation (x, y, theta) for the robot to navigate to
    def _publish_goal(self, x, y, theta):
        """
        Publishes a goal position and orientation for the robot to navigate to.

        Args:
            x (float): The x-coordinate of the goal position in the map frame.
            y (float): The y-coordinate of the goal position in the map frame.
            theta (float): The orientation (yaw) angle of the robot at the goal position, in radians.

        This function creates a PoseStamped message with the given position and orientation,
        and publishes it to the relevant topic, which is typically used by the robot's navigation system
        to plan a path to the goal.
        """

        # Convert the yaw angle (theta) to a format usable for quaternion calculations
        goal_yaw = theta

        # Create a PoseStamped message, which includes position and orientation in a specific frame
        goal = PoseStamped()

        # Set the timestamp for the message to the current time, ensuring synchronization with other nodes
        goal.header.stamp = self.get_clock().now().to_msg()

        # Specify that the goal coordinates are relative to the "map" frame, which is the global reference frame.
        goal.header.frame_id = "map"

        # Set the x, y, and z coordinates for the goal position
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0    # Assuming a 2D plane, so z is set to 0

        # Convert the yaw angle (theta) into a quaternion for the robot's orientation at the goal
        # Quaternions are used in 3D space to avoid issues like gimbal lock
        goal.pose.orientation.x = 0 # No rotation around the x-axis
        goal.pose.orientation.y = 0 # No rotation around the y-axis
        goal.pose.orientation.z = np.sin(goal_yaw/2) # Rotation around the z-axis (yaw)
        goal.pose.orientation.w = np.cos(goal_yaw/2) # Rotation component to complete the quaternion

        # Publish the goal to the topic that the robot's navigation stack listens to 
        # This triggers the navigation system to start planning a path to the specified goal
        self._initial_goal_pub.publish(goal)

    # Function to find a valid (free) random position (x, y, theta) on the map 
    # that the robot can potentially naviagate to
    def _get_random_pos_on_map(self, map):
        """
        Generates a random valid position (x, y) and orientation (theta) on the map.

        Args:
            map (OccupancyGrid): The map data, which includes information about the map's dimentions, resolution,
            and any obstacles.

        Returns:
            tuple: A tuple (x, y, theta) representing a random valid position (x, y) and orientation (theta) on the map.
        """

        # Calculate the total width of the map in the global coordinate system
        map_width = map.info.width * map.info.resolution + map.info.origin.position.x

        # Calculate the total height of the map in the global coordinate system
        map_height = map.info.height * map.info.resolution + map.info.origin.position.y

        # Generate a random x-coordinate within the map's width
        x = random.uniform(0.0, map_width)

        # Generate a random y-coordinate within the map's height
        y = random.uniform(0.0, map_height)

        # Define a sage radius around the robot, combining the robot's own radius with and additional safety margin
        radius = self.ROBOT_RADIUS + 0.5

        # Loop until a valid position is found (i.e., the position is free of obstacles)
        while not self._is_pos_valid(x, y, radius, map):
            # If the position is not valid, generate new random coordinates
            x = random.uniform(0.0, map_width)
            y = random.uniform(0.0, map_height)

        # Generate a random orientation angle (theta) between -pi and pi
        theta = random.uniform(-math.pi, math.pi)

        # Return the valid random position and orientation as a tuple
        return x, y, theta
    
    # Function to check if a position (x, y) is valid (free of obstacles) on the map
    def _is_pos_valid(self, x, y, radius, map):
        """
        Checks if the position (x, y) is valid on the map, meaning it is free of obstacles and within bounds.

        Args:
            x (float): The x-coordinate of the position to check.
            y (float): The y-coordinate of the position to check.
            radius (float): The radius around the position to check for obstacles.
            map (OccupancyGrid): The map data, which includes information about obstacles.

        Returns:
            bool: True if the position is valid (free of obstacles and within bounds), False otherwise
        """

        # Calculate the number of cells that correspond to the given safety radius
        cell_radius = int(radius / map.info.resolution)

        # Convert the global x, y coordinates to map grid indices
        y_index = int((y - map.info.origin.position.y) / map.info.resolution)
        x_index = int((x - map.info.origin.position.x) / map.info.resolution)

        # Loop through the map cells within the square defined by the radius around the position
        for i in range(x_index - cell_radius, x_index + cell_radius, 1):
            for j in range(y_index - cell_radius, y_index + cell_radius, 1):
                # Calculate the linear index of the cell in the map's data array
                index = j * map.info.width + i

                # Check if the index is within the bounds of the map data
                if index >= len(map.data):
                    return False # The index is out of bounds, so the position is invalid 
                
                try:
                    # Get the value of the cell from the map's data
                    val = map.data[index]
                except IndexError:
                    return False # An IndexError indicates and out-of-bounds access, so the position is invalid
            
                # Check if the cell is occupied by an obstacle (non-zero value means occupied )
                if val != 0:
                    return False # The cell is occupied, so the position is invalid
                
        
        # If all cells in the checked area are free and within bounds, the position is valid
        return True
    

    # ========================================================================= #
    # Observation and action functions

    # Return the observation data as a vector
    def _get_observation(self):
        """
        Processes and normalizes sensor data (pedestrian positions, scan data, and goal position)
        into a single observation vector, with modifications fro the Oradar MS200 lidar specification.

        - The Lidar data is split into 20-22 sefments.
        - Interpolation is applied to handle non-uniform lidar data points.

        Returns:
            np.ndarray: A combined and normalized observation vector.
        """

        # Extract the pedestrian positions from the cnn_data structure
        self.ped_pos = self.cnn_data.ped_pos_map

        # Extract the lidar scan data from the cnn_data structure
        self.scan = self.cnn_data.scan

        # Extract the goal position from the cnn_data structure
        self.goal = self.cnn_data.goal_cart

        # Normalize the pedestrian position map (ped_pos) to the range [-1, 1]
        v_min = -2
        v_max = 2
        self.ped_pos = np.array(self.ped_pos, dtype = np.float32)
        self.ped_pos = 2 * (self.ped_pos - v_min) / (v_max - v_min) + (-1)

        # Process and normalize the lidar scan data (Oradar MS200)
        # Handling missing data (NaN or zero values) by interpolation
        lidar_resolution_deg = 0.8 # Angular resolution of the Oradar MS200 lidar
        fov = 360
        num_lidar_points = int(fov / lidar_resolution_deg) # Total number of lidar points (theoretically)
        num_segments = 18
        segment_size = num_lidar_points // num_segments # Number of points per segment

        # Handle missing values (NaNs or zeros) using interpolation
        lidar_data = np.array(self.scan, dtype=np.float32)
        angles = np.linspace(0, fov, num=num_lidar_points, endpoint=False) # generate an array of equally spaced values between 0 and fov (not including fov)

        # Replace zeros and NaNs with interpolated values
        # Valid range: 0.03 to 12 meters
        valid_mask = np.logical_and(lidar_data > 0.03, lidar_data <= 12) # return a boolean mask indicating whether the values satisfy the condition
        
        if not np.all(valid_mask): # check if all elements in the valid_mask array evaluate to True
            # Interpolate the missing values (zeros or NaNs)
            valid_angles = angles[valid_mask]
            valid_data = lidar_data[valid_mask]
            interpolation_function = interp1d(valid_angles, valid_data, kind='linear', fill_value="extrapolate") # takes in two arrays. Then perform linear interpolation and extrapolation
            lidar_data = interpolation_function(angles)
             
        # Split the lidar data into 18 segments and compute the minimum and mean values
        scan_avg = np.zeros((2, num_segments))
        for n in range(num_segments):
            segment_data = lidar_data[n * segment_size: (n + 1) * segment_size]
            scan_avg[0, n] = np.min(segment_data) # Minimum value in the segment
            scan_avg[1, n] = np.mean(segment_data) # Mean value in the segment

        # Flatten the scan data for processing
        scan_avg_flat = scan_avg.flatten()
        s_min = 0.03 # Minimum lidar range
        s_max = 12.0 # Maximum lidar range
        self.scan = 2 * (scan_avg_flat - s_min) / (s_max - s_min) + (-1) # Normalize to [-1, 1]

        # Normalize the goal position to the range [-1, 1] to ensuere all input data is on the same scale.
        g_min = -2
        g_max = 2
        self.goal = np.array(self.goal, dtype=np.float32)
        self.goal = 2 * (self.goal - g_min) / (g_max - g_min) + (-1)

        # Combine the normalized pedestrian positions, scan data, and goal position into a single observation vector
        self.observation = np.concatenate((self.ped_pos, self.scan, self.goal), axis = None)

        # Return the combined observation vector
        return self.observation
    
    # collect and return key pieces of information about the robot's state
    # including initial pose, current pose, and goal position
    def _post_information(self):
        self.info = {
            "initial_pose": self.init_pose,
            "goal_position": self.goal_position,
            "current_pose": self.curr_pose
        }
        return self.info
    
    # Translate a given action into velocity commands, then publishing these commands to the robot
    def _take_action(self, action):
        """
        Converts action values into linear and angular velocity commands and publishes them.
        
        Args:
            action (list): A list where action[0] is the linear velocity control input 
                        and action[1] is the angular velocity control input.
        """
        cmd_vel = Twist()
        
        # Set the new linear velocity range [-0.22, 0.22] m/s
        vx_min = -0.22
        vx_max = 0.22
        
        # Set the new angular velocity range [-2.84, 2.84] rad/s
        vz_min = -2.84
        vz_max = 2.84
        
        # Map the action[0] (assumed to be in the range [-1, 1]) to the linear velocity range
        cmd_vel.linear.x = (action[0] + 1) * (vx_max - vx_min) / 2 + vx_min
        
        # Map the action[1] (assumed to be in the range [-1, 1]) to the angular velocity range
        cmd_vel.angular.z = (action[1] + 1) * (vz_max - vz_min) / 2 + vz_min
        
        # Publish the velocity command
        self._cmd_vel_pub.publish(cmd_vel)


    # ========================================================================= #
    # Reward computation

    # compute goal reaching reward
    def _goal_reached_reward(self, r_arrival, r_waypoint):
        """
        Computes the reward based on how close the robot is to the goal.
        It rewards the robot for reaching the goal or making progress towards it 
        and penalizes the robot if it fails to reach the goal within the allowed iterations.

        Args:
            r_arrival (float): Reward for reaching the goal.
            r_waypoint (float): Reward for making progress towards the goal.

        Returns:
            float: The computed reward
        """

        # Calculate the Euclidean distance between the robot's current position and the goal position
        dist_to_goal = np.linalg.norm(
            np.array([
                self.curr_pose.position.x - self.goal_position.x, # X-distance to goal 
                self.curr_pose.position.y - self.goal_position.y, # Y-distance to goal
                self.curr_pose.position.z - self.goal_position.z # Z-distance to goal
            ])
        )

        # Use modulo operation to track progress every DIST_NUM iterations
        t_1 = self.num_iterations % self.DIST_NUM

        # Initialize the distance tracking array on the first iteration
        if self.num_iterations == 0:
            self.dist_to_goal_reg = np.ones(self.DIST_NUM) * dist_to_goal # Initialize the array with the current distance
        
        # Define the maximum number of iterations allowed for reaching the goal
        max_iteration = 512

        # Case 1: Robot reaches the goal (distance to goal is within the specified radius)
        if dist_to_goal < self.GOAL_RADIUS:
            reward = r_arrival # Assign maximum reward for reaching the goal

        # Case 2: Robot fails to reach teh goal within the maximum allowed iterations
        elif self.num_iterations >= max_iteration:
            reward = -r_arrival # Penalize the robot for not reaching the goal in time

        # Case 3: Robot is making progress but hasn't reached the goal_yet
        else:
            # Reward is proportional to the reduction in distance to the goal since the last check
            reward = r_waypoint * (self.dist_to_goal_reg[t_1] - dist_to_goal)

        # Update the distance reward for the current iteration (used for future comparisons)
        self.dist_to_goal_reg[t_1] = dist_to_goal

        # Return the calculated reward (positive for progress or goal reached, negative for failure)
        return reward

    # Punishment after collision with obstacles
    def _obstacle_collision_punish(self, scan, r_scan, r_collision):
        """
        Computes the penalty for being too close to or colliding with obstacles based on lidar scan data.

        Args:
            scan (np.array): Lidar scan data (distances to obstacles).
            r_scan (float): Penalty factor for proximity to obstacles.
            r_collision (float): Penalty factor for colliding with an obstacle (i.e. getting too close)

        Returns:
            float: The computed penalty based on proximity to obstacles.
        """

        # Find the minimum distance from the lidar scan, ignoring zaro values (which indicate no reading)
        min_scan_dist = np.amin(scan[scan != 0])

        # Case 1: The robot is very close to an obstacle, potentially colliding (distance less than or equal to its radius)
        if min_scan_dist <= self.ROBOT_RADIUS and min_scan_dist >= 0.03:
            reward = r_collision # Apply a significant penalty for a collision

        # Case 2: The robot is near an obstacle but not colliding (distance within 3 times its radius)
        elif min_scan_dist < 3 * self.ROBOT_RADIUS:
            reward = r_scan * (3 * self.ROBOT_RADIUS - min_scan_dist) # Apply a smaller penalty based on proximity

        # Case 3: The robot is far enough from obstacles (no penalty)
        else:
            reward = 0.0

        return reward

    # Punishment for high angular velocity (fast turns)
    def _angular_velocity_punish(self, w_z, r_rotation, w_thresh):
        """
        Computes the penalty for excessive angular velocity (rotational speed).

        Args:
            w_z (float): The robot's current angular velocity (rotational speed around the z-axis).
            r_rotation (float): Penalty factor for angular velocity.
            w_thresh (float): Threshold for penalizing excessive angular velocity.

        Returns:
            float: The computed penalty based on the angular velocity.
        """
        
        # Case 1: If the angular velocity exceeds the threshold, apply a penalty
        if abs(w_z) > w_thresh:
            reward = abs(w_z) * r_rotation # Penalty increases with higher angular velocity 
        
        # Case 2: If the angular velocity is within acceptable limits, no penalty is given
        else:
            reward = 0.0
        
        return reward


    # Reward for aligning with the goal
    def _theta_reward(self, goal, mht_peds, v_x, r_angle, angle_thresh):
        """
        Computes a reward for aligning the robot's heading with the goal direction,
        while considering the presence of pedestrians to avoid collisions.

        Args:
            goal (np.array): The goal's position relative to the robot.
            mht_peds (object): Pedestrian tracking data (positions and velocities of pedestrians).
            v_x (float): The robot's current linear velocity (forward motion).
            r_angle (float): Reward factor for maintaining alignment with the goal.
            angle_thresh (float): Threshold for the angular reward (maximum allowable deviation).

        Returns:
            float: The computed reward based on heading alignment and pedestrian avoidance.
        """

        # Calculate the preferred angle (theta) to the goal in counter clockwise direction (robot's heading relative to the goal position)
        # Read more about this formula at: https://en.wikipedia.org/wiki/Atan2
        theta_pre = np.arctan2(goal[1], goal[0])

        d_theta = theta_pre # Initialize the heading difference as the preferred angle 

        # Case 1: If there are pedestrians in the scene
        if len(mht_peds.tracks) != 0:
            d_theta = np.pi / 2 # Set initial delta theta to 90 degrees (default avoidance angle)
            N = 60 # Number of random angle samples to check for safe movement
            theta_min = 1000 # Large initial value for minimum angle difference

            # Sample random angles to find a safe direction to move
            for i in range(N):
                theta = random.uniform(-np.pi, np.pi) # Random ange in range [-pi, pi]
                free = True # Assume the angle is free from obstacles/pedestrians

                # Loop through the tracked pedestrians to check for potential collisions
                for ped in mht_peds.tracks:
                    p_x = ped.pose.pose.position.x # Pedestrian's x-position
                    p_y = ped.pose.pose.position.y
                    p_vx = ped.twist.twist.linear.x # Pedestrian's x-velocity
                    p_vy = ped.twist.twist.linear.y

                    # Calculate the distance to the pedestrian
                    ped_dis = np.linalg.norm([p_x, p_y])

                    # If the pedestrian is within a 7m radius, check for collision potential
                    if ped_dis <= 7:
                        ped_theta = np.arctan2(p_y, p_x) # Pedestrian's angle relative to the robot
                        vo_theta = np.arctan2(3 * self.ROBOT_RADIUS, np.sqrt(ped_dis ** 2 - (3 * self.ROBOT_RADIUS) ** 2))

                        # Calculate the relative angle between the robot and pedestrian to avoid collision
                        theta_rp = np.arctan2(v_x * np.sin(theta) - p_vy, v_x * np.cos(theta) - p_vx)

                        # If the angle overlaps with the pedestrian's path (collision cone), mark as unsafe
                        if theta_rp >= (ped_theta - vo_theta) and theta_rp <= (ped_theta + vo_theta):
                            free = False
                            break

                
        # Case 2: If there are no pedestrians, simply align with the goal
        else:
            d_theta = theta_pre

        # Calculate the reward based on the angular difference from the goal direction 
        reward = r_angle * (angle_thresh - abs(d_theta))

        return reward

    # Aggregate reward computation
    def _compute_reward(self):
        """
        Computes the total reward for the robot's action based on various criteria such as:
        - Reaching the goal
        - Avoiding obstacles
        - Minimizing angular velocity (smooth turning)
        - Keeping a good heading angle toward the goal

        Returns:
            float: The toal reward for the current action.
        """

        # Reward for reaching the goal
        r_arrival = 20 # Large positive reward for reaching the goal.
        r_waypoint = 3.2 # Smaller reward for moving closer to the goal.

        # Penalty for colliding with obstacles
        r_collision = -20 # Large negative penalty for collisions
        r_scan = -0.2  # Penalty for being too close to obstacles (based on scan data)

        # Rewards related to the robot's angle and smooth turning
        r_angle = 0.6 # Reward for maintaining the correct heading angle towards the goal
        r_rotation = -0.1 # Penalty for excessive rotation (fast turns)

        # Thresholds for angular velocity and angle deviation
        angle_thresh = np.pi / 6 # Angle threshold (30 degrees)
        w_thresh = 1 # Angular velocity threshold

        # 1. Compute the reward for reaching or moving toward the goal
        r_g = self._goal_reached_reward(r_arrival, r_waypoint)

        # 2. Compute the penalty for collisions or being too close to obstacles
        r_c = self._obstacle_collision_punish(self.cnn_data.scan[:450], r_scan, r_collision)

        # 3. Compute the penalty for high angular velocity (fast turns)
        r_w = self._angular_velocity_punish(self.curr_vel.angular.z, r_rotation, w_thresh)

        # 3. Compute the reward for aligning with the goal.
        r_t = self._theta_reward(self.goal, self.mht_peds, self.curr_vel.linear.x, r_angle, angle_thresh)

        # Total reward is a sum of all components.
        reward = r_g + r_c + r_t + r_w 

        return reward
    
    # ========================================================================= #
    # Done flag computation

    # Determine whether the current episode (i.e., the task or goal the robot is trying to accomplish) has ended.
    def _is_done(self, reward):
        """
        Determines whether the current episode should end based on several conditions:
        - The robot reaches the goal.
        - The robot collides with obstacles a certain number of times.
        - The maximum number of iterations is exceeded.

        Args:
            reward (float): Current reward (not used directly in this function).

        Returns:
            bool: True if the episode is done, False otherwise.
        """

        # Increment the number of iterations (i.e., time steps taken in the current episode)
        self.num_iterations += 1

        # Calculate the Euclidean distance between the robot's urrent position and the goal position.
        dist_to_goal = np.linalg.norm(
            np.array([
                self.curr_pose.position.x - self.goal_position.x, # Difference in x-coordinates
                self.curr_pose.position.y - self.goal_position.y, # Difference in y-coordinates
                self.curr_pose.position.z - self.goal_position.z # Difference in z-coordinates
            ])
        )

        # Condition 1: Check if the robot has reached the goal (within the defined goal radius)
        if dist_to_goal <= self.GOAL_RADIUS:
            self._cmd_vel_pub.publish(Twist()) # Stop the robot by sending a zero velocity command
            self._episode_done = True # Mark the episode as done
            return True # Return True indicating that the episode is finished
        
        # Fetch the latest scan data (e.g., from the lidar) to check the proximity of obstacles
        scan = self.cnn_data.scan[-450: ] # Use the last 450 scan points 

        # Handle missing or zero values in the lidar data using interpolation
        scan[scan == 0] = np.nan # Convert zero values (which indicate missing data) to NaN
        valid_indices = np.where(~np.isnan(scan))[0] # Indices of valid (non-NaN) data
        valid_scan = scan[valid_indices] # Extract valid scan data 

        if len(valid_scan) < 2: # If there are too few valid points to interpolate, set default safe distance
            min_scan_dist = 12.0 
        else: 
            # Interpolate missing values in the scan data
            interp_func = interp1d(valid_indices, valid_scan, bounds_error=False, fill_value="extrapolate")
            scan_filled = interp_func(np.arange(len(scan))) # Fill missing data with interpolation
            min_scan_dist = np.amin(scan_filled) # Find the closest obstacle distance

        # Ensure min_scan_dist falls within the lidar's valid range of [0.03, 12] meters
        min_scan_dist = np.clip(min_scan_dist, 0.03, 12.0)

        # Condition 2: Check if the robot is colliding with an obstacle
        # If the robot is too close to an obstacle (distance <= robot_radius), increment the bump counter
        if min_scan_dist <= self.ROBOT_RADIUS and min_scan_dist >= 0.03:
            self.bump_num += 1 # Increment the bump counter if the robot is too close

        # Condition 3: If the robot has collided with obstacles more than 3 times, end the episode
        if self.bump_num >= 3:
            self._cmd_vel_pub.publish(Twist()) # Stop the robot by sending a zero velocity command
            self.bump_num = 0 # Reset the bump counter for the next episode
            self._episode_done = True # Mark the episode as done
            self._reset = True # Flag the environment for a reset after the episode end
            return True # Return True indicating that the episode is finished
        
        # Condition 4: Check if the robot has exceeded the maximum number of allowed iteration
        max_iteration = 512 # Set the maximum number of iterations allowed for one episode
        if self.num_iterations > max_iteration:
            self._cmd_vel_pub.publish(Twist()) # Stop the robot by sending a zero velocity command
            self._episode_done = True # Mark the episode as done
            self._reset = True # Flag the environment for a reset after the episode end
            return True
        
        # If none of the termination conditions are met, the episode continues
        return False # Return False indicating that the episode is still ongoing