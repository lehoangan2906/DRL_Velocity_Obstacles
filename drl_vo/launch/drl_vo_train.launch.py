""" 
This file sets up the training environment for the DRL-VO model. It includes the 
following components:
 - The model file and log directory are specified as launch arguments.
 - The `turtlebot_teleop' pacakage is included for velocity smoothing during teleoperation.
 - The `drl_vo_cmd` node is the main training node, which tuns the training scripts.
 - The `mix_cmd_vel` node handles publishing mixed velocity commands to the robot.
"""


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    