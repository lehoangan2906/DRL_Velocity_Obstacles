#!/usr/bin/python3
# This script is used to train the DRL-VO policy using the PPO algorithm.

import os
import gym
import rclpy
import numpy as np
import turtlebot_gym    # Custom gym environment
from rclpy.node import Node
from sensor_msgs.msg import Point  # Import Point for dynamic goal position
from stable_baselines3 import PPO
from custom_cnn_full import CustomCNN                   # Import custom CNN model
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.results_plotter import load_results, ts2xy



class SaveOnBestTrainingRewardCallback(BaseCallback):
    """
    Callback for saving a model based on the training reward.
    The check is done every ``check_freq`` steps.
    
    :param check_freq: (int) Frequency to check the reward.
    :param log_dir: (str) Path to the folder where the model will be saved.
    :param verbose: (int) Verbosity level.
    """
    def __init__(self, check_freq: int, log_dir: str, verbose=1):
        super(SaveOnBestTrainingRewardCallback, self).__init__(verbose)
        self.check_freq = check_freq
        self.log_dir = log_dir
        self.save_path = os.path.join(log_dir, 'best_model')
        self.best_mean_reward = -np.inf

    def _init_callback(self) -> None:
        # Create folder if needed
        if self.save_path is not None:
            os.makedirs(self.save_path, exist_ok=True)

    def _on_step(self) -> bool:
        if self.n_calls % self.check_freq == 0:
            # Retrieve training reward
            x, y = ts2xy(load_results(self.log_dir), 'timesteps')
            if len(x) > 0:
                # Mean training reward over the last 100 episodes
                mean_reward = np.mean(y[-100:])
                if self.verbose > 0:
                    print(f"Num timesteps: {self.num_timesteps}")
                    print(f"Best mean reward: {self.best_mean_reward:.2f} - Last mean reward per episode: {mean_reward:.2f}")

                # New best model, save it
                if mean_reward > self.best_mean_reward:
                    self.best_mean_reward = mean_reward
                    if self.verbose > 0:
                        print(f"Saving new best model to {self.save_path}")
                    self.model.save(self.save_path)
        
        # Save model every 20000 timesteps
        if self.n_calls % 20000 == 0:
            path = self.save_path + '_model' + str(self.n_calls)
            self.model.save(path)

        return True

class DRLVOTrain(Node):
    """
    ROS2 Node class for training the DRL-VO policy using the PPO algorithm.
    This class sets up the environment, loads the pre-trained model if available,
    and continues training it while logging progress and saving the best model.

    :param log_dir: (str) Directory to save logs and model checkpoints.
    :param model_file: (str) Path to the pre-trained model file to continue training from.
    """
    def __init__(self):
        super().__init__('drl_vo_train')

        # Initialize dynamic goal topic subscriber
        self.goal_sub = self.create_subscription(
            Point, '/dynamic_goal_position', self.goal_callback, 10)

        # Current goal position (initialize with a default)
        self.current_goal = Point(x=0.0, y=0.0, z=0.0)

        # Get ROS2 parameters with default values
        log_dir = self.declare_parameter('log_dir', "./runs/").get_parameter_value().string_value
        model_file = self.declare_parameter('model_file', "./model/drl_pre_train.zip").get_parameter_value().string_value

        # Create log directory if it doesn't exist
        os.makedirs(log_dir, exist_ok=True)

        # Create and wrap the environment
        env = gym.make('drl-nav-v0', dynamic_goal=self.current_goal) # Initialize the custom gym environment with dynamic goal
        env = Monitor(env, log_dir)  # Monitor the environment to log training data

        # Policy parameters for PPO - for training the model from scratch
        policy_kwargs = dict(
            features_extractor_class=CustomCNN,  # Use a custom CNN for feature extraction
            features_extractor_kwargs=dict(features_dim=256),  # Output dimension from the CNN
            net_arch=[dict(pi=[256], vf=[128])]  # Architecture for the policy (actor or pi) and value function (critic or vf)
        )

        # Raw training (uncomment if needed)
        self.model = PPO("CnnPolicy", env, policy_kwargs=policy_kwargs, learning_rate=1e-3, verbose=2, tensorboard_log=log_dir, n_steps=512, n_epochs=10, batch_size=128)

        # Policy parameters for PPO - for continuing training with pre-trained model
        kwargs = {'tensorboard_log': log_dir,  # Directory for TensorBoard logs
                'verbose': 2,  # Verbosity level
                'n_epochs': 10,  # Number of epochs per update
                'n_steps': 512,  # Number of steps per environment per update
                'batch_size': 128,  # Minibatch size for each gradient update
                'learning_rate': 5e-5}  # Learning rate for the optimizer
        
        # Continue training with pre-trained model
        # self.model = PPO.load(model_file, env=env, **kwargs)  # Load the pre-trained model with specified parameters

        # Create the callback to save the best model
        callback = SaveOnBestTrainingRewardCallback(check_freq=5000, log_dir=log_dir)

        # Start training the model
        self.model.learn(total_timesteps=2000000, log_interval=5, tb_log_name='drl_vo_policy', callback=callback, reset_num_timesteps=True)

        # Save the final model
        self.model.save("drl_vo_model")
        self.get_logger().info("Training completed. Model saved.")
        env.close()

    def goal_callback(self, goal_msg):
        """
        Callback function to update the goal position based on incoming messages.
        :param goal_msg: (Point) The new goal position received from the ROS topic.
        """
        self.current_goal.x = goal_msg.x
        self.current_goal.y = goal_msg.y
        self.current_goal.z = goal_msg.z  # In case you want to use 3D goals

def main(args=None):
    """
    Main function to initialize the ROS2 node and start the training process.
    This function initializes ROS2, creates an instance of the DRLVOTrain node,
    and keeps it running until training is complete.
    """
    rclpy.init(args=args)  # Initialize the ROS2 communication Node
    drl_vo_train = DRLVOTrain()  # Create an instance of the training node
    rclpy.spin(drl_vo_train)  # Keep the node alive and responsive
    drl_vo_train.destroy_node()  # Destroy the node explicitly (optional)
    rclpy.shutdown()  # Shutdown the node

if __name__ == '__main__':
    main()  # Run the main function