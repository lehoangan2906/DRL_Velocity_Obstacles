import os
import numpy as np
import gym
import turtlebot_gym  # Custom environment for TurtleBot navigation
import rclpy  # ROS2 Python library
from rclpy.node import Node  # ROS2 node
from stable_baselines3.common.env_checker import check_env  # Stable Baselines environment checker
from stable_baselines3.common.monitor import Monitor  # For monitoring the environment during training
from stable_baselines3 import PPO  # PPO algorithm from Stable Baselines3
from stable_baselines3.common.results_plotter import load_results, ts2xy  # For plotting results
from stable_baselines3.common.vec_env import DummyVecEnv  # Wrapper for vectorized environments
from stable_baselines3.common.callbacks import BaseCallback  # Base callback class for saving models
from custom_cnn_full import *  # Import custom CNN for feature extraction


# Define a callback class to save the best model during training
class SaveOnBestTrainingRewardCallback(BaseCallback):
    """
    Callback for saving a model based on the training reward.
    This callback saves the best-performing model every `check_freq` steps.
    """

    def __init__(self, check_freq: int, log_dir: str, verbose=1):
        super(SaveOnBestTrainingRewardCallback, self).__init__(verbose)
        self.check_freq = check_freq  # Frequency of checking the reward
        self.log_dir = log_dir  # Directory for saving the model
        self.save_path = os.path.join(log_dir, 'best_model')  # Path to save the best model
        self.best_mean_reward = -np.inf  # Initialize the best reward as negative infinity

    def _init_callback(self) -> None:
        """
        Initialize the callback (e.g., create the log directory if needed).
        This is called once when the training starts.
        """
        # Create folder to save the model if it doesn't exist
        if self.save_path is not None:
            os.makedirs(self.save_path, exist_ok=True)

    def _on_step(self) -> bool:
        """
        This method is called at every step of the training loop.
        It checks the mean reward and saves the model if the performance improves.
        """
        # Check the reward every `check_freq` steps
        if self.n_calls % self.check_freq == 0:
            # Load and extract results (x: timesteps, y: rewards)
            x, y = ts2xy(load_results(self.log_dir), 'timesteps')
            if len(x) > 0:
                # Calculate the mean reward over the last 100 episodes
                mean_reward = np.mean(y[-100:])
                if self.verbose > 0:
                    self.logger.info(f"Num timesteps: {self.num_timesteps}")
                    self.logger.info(f"Best mean reward: {self.best_mean_reward:.2f} - Last mean reward: {mean_reward:.2f}")

                # If the current mean reward is better than the best so far, save the model
                if mean_reward > self.best_mean_reward:
                    self.best_mean_reward = mean_reward  # Update best mean reward
                    if self.verbose > 0:
                        self.logger.info(f"Saving new best model to {self.save_path}")
                    self.model.save(self.save_path)  # Save the model
        
        # Save the model every 20000 timesteps (regardless of performance)
        if self.n_calls % 20000 == 0:
            path = self.save_path + '_model' + str(self.n_calls)
            self.model.save(path)  # Save the model at the specified checkpoint

        return True  # Return True to continue training


def main(args=None):
    # Initialize the ROS2 environment
    rclpy.init(args=args)
    node = rclpy.create_node('env_test')  # Create a ROS2 node named 'env_test'

    # Get the directory for logs (set by the ROS2 parameter, default is './runs/')
    log_dir = node.get_parameter_or('log_dir', './runs/')
    os.makedirs(log_dir, exist_ok=True)  # Create the log directory if it doesn't exist

    # Create and wrap the gym environment ('drl-nav-v0' is the custom environment for navigation)
    env = gym.make('drl-nav-v0')  # Load the custom navigation environment
    env = Monitor(env, log_dir)  # Monitor the environment to log performance metrics
    obs = env.reset()  # Reset the environment to start from an initial state

    # Define the policy's architecture (PPO with custom CNN for feature extraction)
    policy_kwargs = dict(
        features_extractor_class=CustomCNN,  # Use a custom CNN for feature extraction
        features_extractor_kwargs=dict(features_dim=256),  # CNN output feature dimension
        net_arch=[dict(pi=[256], vf=[128])]  # Neural network architecture: pi for policy, vf for value function
    )

    # **Raw Training Option**: If you want to train a model from scratch, uncomment the following lines:
    """
    model = PPO("CnnPolicy", env, policy_kwargs=policy_kwargs, 
                learning_rate=1e-3,  # Higher learning rate for scratch training
                verbose=2, 
                tensorboard_log=log_dir, 
                n_steps=512, 
                n_epochs=10, 
                batch_size=128)  # Raw training parameters
    """

    # **Continue Training Option**: Load a pre-trained model and continue training with custom parameters
    kwargs = {
        'tensorboard_log': log_dir,  # Log directory for TensorBoard
        'verbose': 2,  # Verbosity level (2 for detailed output)
        'n_epochs': 10,  # Number of epochs per update
        'n_steps': 512,  # Number of steps per environment per update
        'batch_size': 128,  # Batch size for each gradient update
        'learning_rate': 5e-5  # Lower learning rate for fine-tuning
    }
    model_file = node.get_parameter_or('model_file', './model/drl_pre_train.zip')  # Path to the pre-trained model
    model = PPO.load(model_file, env=env, **kwargs)  # Load the model with the specified parameters

    # Create the callback for saving the best model
    callback = SaveOnBestTrainingRewardCallback(check_freq=5000, log_dir=log_dir)

    # Start training the model (for 2 million timesteps, log every 5 steps)
    model.learn(
        total_timesteps=2000000,  # Total training steps
        log_interval=5,  # Interval to log the progress
        tb_log_name='drl_vo_policy',  # Name for TensorBoard logging
        callback=callback,  # Pass the callback for saving the best model
        reset_num_timesteps=True  # Reset the timesteps counter
    )

    # Save the final model after training is complete
    model.save('drl_vo_model')  # Save the trained model to a file
    node.get_logger().info('Training finished.')  # Log that the training is finished
    env.close()  # Close the environment

    # Shutdown the ROS2 node
    rclpy.shutdown()  # Cleanly shut down the ROS2 environment


if __name__ == '__main__':
    # If this script is run directly, call the main function
    main()