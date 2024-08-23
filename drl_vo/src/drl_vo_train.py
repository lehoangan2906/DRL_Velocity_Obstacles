#!/usr/bin/python3
# Call to custom_cnn_full.py

import rclpy
import os
import numpy as np
import gym
import gymnasium as gym
import turtlebot_gym # Custom gym environment
from rclpy.node import Node
from sensor_msgs.msg import Point   # Import Point for dynamic goal
from stable_baselines3.common.vec_env import DummyVecEnv