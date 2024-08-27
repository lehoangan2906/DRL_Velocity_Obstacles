import numpy as np
import numpy.matlib
import random
import math
from scipy.optimize import linprog, minimize
import threading

import rclpy  # Use rclpy for ROS 2 instead of rospy
from rclpy.node import Node
import gym  # Import OpenAI Gym
from gym.utils import seeding
from gym import spaces
from .gazebo_connection import GazeboConnection  # Import custom Gazebo connection for ROS 2
from std_msgs.msg import Float64, Empty, Bool
from gazebo_msgs.msg import ModelStates, ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import Pose, Twist, Point, PoseStamped, PoseWithCovarianceStamped
import time
from sensor_msgs.msg import BumperEvent
from actionlib_msgs.msg import GoalStatusArray
from pedsim_msgs.msg import TrackedPersons, TrackedPerson
from cnn_msgs.msg import CNN_data

class DRLNavEnv(gym.Env):
    """
    Custom Gym environment for TurtleBot3 in Gazebo with ROS 2.
    Handles unpausing the simulation, resetting controllers, and interfacing with ROS 2.
    """

    def __init__(self):
        # Initialize the ROS 2 node
        rclpy.init()
        self.node = Node('drl_nav_env')
        self.seed()

        # Robot parameters
        self.ROBOT_RADIUS = 0.3
        self.GOAL_RADIUS = 0.3
        self.DIST_NUM = 10
        self.pos_valid_flag = True
        # Bumper
        self.bump_flag = False
        self.bump_num = 0
        # Reward
        self.dist_to_goal_reg = np.zeros(self.DIST_NUM)
        self.num_iterations = 0

        # Action limits
        self.max_linear_speed = 0.5
        self.max_angular_speed = 2
        # Action space
        self.high_action = np.array([1, 1])
        self.low_action = np.array([-1, -1])
        self.action_space = spaces.Box(low=self.low_action, high=self.high_action, dtype=np.float32)
        # Observation space
        self.cnn_data = CNN_data()
        self.ped_pos = []
        self.scan = []
        self.goal = []
        #self.vel = []

        self.observation_space = spaces.Box(low=-1, high=1, shape=(19202,), dtype=np.float32)
        # Info, initial position, and goal position
        self.init_pose = Pose()
        self.curr_pose = Pose()
        self.curr_vel = Twist()
        self.goal_position = Point()
        self.info = {}
        # Episode done flag
        self._episode_done = False
        # Goal reached flag
        self._goal_reached = False
        # Reset flag
        self._reset = True

        # Initialize Gazebo connection
        self.gazebo = GazeboConnection(
            start_init_physics_parameters=True,
            reset_world_or_sim="WORLD"  # Reset only the world in the simulation
        )

        # Unpause simulation to start data flow
        self.gazebo.unpauseSim()

        # Initialize ROS 2 subscribers and publishers
        self._map_sub = self.node.create_subscription(OccupancyGrid, "/map", self._map_callback, 10)
        self._cnn_data_sub = self.node.create_subscription(CNN_data, "/cnn_data", self._cnn_data_callback, 10)
        self._robot_pos_sub = self.node.create_subscription(PoseStamped, "/robot_pose", self._robot_pose_callback, 10)
        self._robot_vel_sub = self.node.create_subscription(Odometry, '/odom', self._robot_vel_callback, 10)
        self._final_goal_sub = self.node.create_subscription(PoseStamped, "/move_base/current_goal", self._final_goal_callback, 10)
        self._goal_status_sub = self.node.create_subscription(GoalStatusArray, "/move_base/status", self._goal_status_callback, 10)
        self._ped_sub = self.node.create_subscription(TrackedPersons, "/track_ped", self._ped_callback, 10)

        # Publishers
        self._cmd_vel_pub = self.node.create_publisher(Twist, '/drl_cmd_vel', 10)
        self._initial_goal_pub = self.node.create_publisher(PoseStamped, '/move_base_simple/goal', 10)
        self._set_robot_state_service = self.node.create_client(SetModelState, '/gazebo/set_model_state')
        self._initial_pose_pub = self.node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        # Check that all systems are ready before starting
        self._check_all_systems_ready()
        self.gazebo.pauseSim()

        self.node.get_logger().debug("Finished DRLNavEnv INIT...")

    # Env methods
    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        """
        Perform one step in the environment.
        Args:
            action: The action to take (e.g., linear and angular velocity).
        Returns:
            obs: The current observation.
            reward: The reward for the current step.
            done: Whether the episode has ended.
            info: Additional information about the step.
        """
        self.gazebo.unpauseSim()
        self._take_action(action)
        self.gazebo.pauseSim()
        obs = self._get_observation()
        reward = self._compute_reward()
        done = self._is_done(reward)
        info = self._post_information()
        return obs, reward, done, info

    def reset(self):
        """ 
        Reset the environment to an initial state.
        Returns:
            obs: The initial observation.
            info: Additional information about the reset state.
        """
        self.node.get_logger().debug("Resetting DRLNavEnv")
        self._reset_sim()
        obs = self._get_observation()
        info = self._post_information()
        self.node.get_logger().debug("DRLNavEnv reset complete")
        return obs

    def close(self):
        """
        Clean up when closing the environment.
        """
        self.node.get_logger().warn("Closing DRLNavEnv")
        rclpy.shutdown()

    def _reset_sim(self):
        """Resets the simulation."""
        self.node.get_logger().debug("Starting simulation reset")
        self.gazebo.unpauseSim()
        self._set_init()
        self.gazebo.pauseSim()
        self.node.get_logger().debug("Simulation reset complete")
        return True
