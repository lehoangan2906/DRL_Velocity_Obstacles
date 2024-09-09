import numpy as np
import rclpy
from rclpy.node import Node
from cnn_msgs.msg import CnnData
from geometry_msgs.msg import Point, Twist
from pedsim_msgs.msg import TrackedPerson, TrackedPersons
from sensor_msgs.msg import LaserScan
from cnn_data_pub import CnnData  # Import the main class from your file

def create_dummy_lidar_data():
    # Generate a dummy lidar scan with some missing data (0s) and valid data
    scan_data = np.random.uniform(0.03, 12.0, 450)
    scan_data[::10] = 0  # Introduce missing data (every 10th value)
    laser_scan_msg = LaserScan()
    laser_scan_msg.ranges = scan_data.tolist()
    return laser_scan_msg

def create_dummy_pedestrian_data():
    # Generate dummy pedestrian tracking data
    tracked_person = TrackedPerson()
    tracked_person.pose.pose.position.x = np.random.uniform(0, 20)
    tracked_person.pose.pose.position.y = np.random.uniform(-10, 10)
    tracked_person.twist.twist.linear.x = np.random.uniform(-1, 1)
    tracked_person.twist.twist.linear.y = np.random.uniform(-1, 1)
    
    tracked_persons_msg = TrackedPersons()
    tracked_persons_msg.tracks = [tracked_person for _ in range(5)]  # Create 5 pedestrians
    return tracked_persons_msg

def create_dummy_goal_data():
    # Generate a dummy goal position
    goal_msg = Point()
    goal_msg.x = np.random.uniform(0, 10)
    goal_msg.y = np.random.uniform(0, 10)
    return goal_msg

def create_dummy_velocity_data():
    # Generate dummy velocity data
    vel_msg = Twist()
    vel_msg.linear.x = np.random.uniform(-0.5, 0.5)
    vel_msg.angular.z = np.random.uniform(-1.0, 1.0)
    return vel_msg

def test_cnn_data_pub():
    rclpy.init()
    
    node = CnnData()

    # Test Lidar Callback
    lidar_data = create_dummy_lidar_data()
    node.scan_callback(lidar_data)
    print("Lidar scan processed successfully")

    # Test Pedestrian Callback
    ped_data = create_dummy_pedestrian_data()
    node.ped_callback(ped_data)
    print("Pedestrian data processed successfully")

    # Test Goal Callback
    goal_data = create_dummy_goal_data()
    node.goal_callback(goal_data)
    print("Goal data processed successfully")

    # Test Velocity Callback
    vel_data = create_dummy_velocity_data()
    node.vel_callback(vel_data)
    print("Velocity data processed successfully")

    # Test Timer Callback (which will publish the CNN data)
    node.timer_callback()
    print("CNN data published successfully")

    rclpy.shutdown()

if __name__ == '__main__':
    test_cnn_data_pub()
