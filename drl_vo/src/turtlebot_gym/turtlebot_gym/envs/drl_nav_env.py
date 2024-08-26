import numpy as np
import numpy.matlib
import random
import math
from scipy.optimize import linprog, _minimize
import threading

import rclpy
from rclpy.node import Node
from gym.utils import seeding
from gym import spaces

from geometry_msgs.msg import Pose, Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64, Empty, Bool

from gazebo_msgs.msg import ModelStates, ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
from pedsim_msgs.msg import TrackedPersons, TrackedPerson
from cnn_msgs import CNN_data

class DRLNavEnv(Node):
    """
        Gazebo environment with ROS2, converting standard gym methods into Gazebo commands
    """

    def __init__(self):
        super().__init__('drl_nav_env')
        self.seed()

        # Robot parameters
        self.ROBOT_RADIUS = 0.3 
        self.GOAL_RADIUS = 0.3
        self.DIST_NUM = 10
        self.pos_valid_flag = True

        # bumper
        self.bump_flag = False
        self.bump_num = 0

        # reward
        self.dist_to_goal_reg = np.zeros(self.DIST_NUM)
        self.num_iterations = 0

        # action limits
        self.max_linear_speed = 0.4 
        self.max_angular_speed = 0.6

        # action space 
        self.high_action = np.array([1, 1])
        self.low_action = np.array([-1, -1])
        self.action_space = spaces.Box(low=self.low_action, high=self.high_action, dtype=np.float32)

        # observation space
        self.cnn_data = CNN_data()
        self.ped_pos = []
        self.scan = []
        self.goal = []

        # MaxAbsScaler: normalize to (-1, 1)
        self.observation_space = spaces.Box(low=-1, high=1, shape=(19202,), dtype=np.float32)

        # Info, initial position, and goal position
        