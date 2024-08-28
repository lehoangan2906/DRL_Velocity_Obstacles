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

    # Publisher functions to publish data
    def _pub_initial_model_state(self, x, y, theta):
        robot_state = ModelState()
        robot_state.model_name = "mobile_base"
        robot_state.pose.position.x = x
        robot_state.pose.position.y = y
        robot_state.pose.position.z = 0
        robot_state.pose.orientation.x = 0
        robot_state.pose.orientation.y = 0
        robot_state.pose.orientation.z = np.sin(theta/2)
        robot_state.pose.orientation.w = np.cos(theta/2)
        robot_state.reference_frame = "world"
        try:
            result = self.gazebo_client.call(robot_state)
        except rclpy.ServiceException:
            pass