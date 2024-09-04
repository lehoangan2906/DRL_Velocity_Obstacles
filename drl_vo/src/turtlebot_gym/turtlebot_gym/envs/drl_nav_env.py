#!/usr/bin/python3

import numpy as np
import numply.matlib
import random
import math
import gym
from scipy.optimize import linprog, minimize
import threading

import rclpy
from rclpy.node import Node
from gym import spaces
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import GetModelState, SetModelState
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import Pose, Twist, Point, PoseStamped, PoseWithCovarianceStamped
import time
from sensor_msgs.msg import LaserScan
from action_msgs.msg import GoalStatusArray
from pedsim_msgs.msg import TrackedPersons
from cnn_msgs.msg import CNNData


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
        self.cnn_data = CNNData()
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
        self._cnn_data_sub = self.create_subscription(CNNData, "/cnn_data", self._cnn_data_callback, qos)
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
        self._check_subscriber_ready("/cnn_data", CNNData)
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
        self._check_publisher_ready(self._cmd_vel_pub))
        self._check_publisher_ready(self._initial_goal_pub)
        self._check_publisher_ready('/gazebo/set_model_state')
        self._check_publisher_ready(self._initial_pose_pub)

    def _check_publisher_ready(self, obj):
        # Check if a publisher is ready
        while obj.get_subscription_count() == 0:
            pass
    
    def _check_service_ready(self, name, timeout=5.0):
        try:
            self.gazebo_client.wait_for_service(timeout)
        except rclpy.ServiceException:
            pass

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
    