import numpy as np
from scipy.interpolate import interp1d # For preprocessing the lidar data


# Simulate the cnn_data structure with pedestrian positions, scan, and goal
class FakeCnnData:
    def __init__(self):
        # Simulate pedestrian positions (randomly generated for this test)
        self.ped_pos_map = np.random.uniform(-2, 2, size=(10,))
        
        # Simulate lidar scan data (450 points for 360° FOV at 0.8° resolution)
        # Include valid points, zeros, and NaNs to test the function's ability to handle missing data
        self.scan = np.random.uniform(0.1, 30, size=(450,))
        self.scan[np.random.choice(450, 50, replace=False)] = 0  # Set some values to 0
        self.scan[np.random.choice(450, 50, replace=False)] = np.nan  # Set some values to NaN

        # Simulate goal position (randomly generated for this test)
        self.goal_cart = np.random.uniform(-2, 2, size=(3,))

# Define the robot's observation function, assuming the previous modifications are made
def test_get_observation():
    # Initialize the fake cnn_data
    cnn_data = FakeCnnData()
    
    # Define a mock class to hold the function
    class MockRobot:
        def __init__(self):
            self.cnn_data = cnn_data
        
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

            # Normalize the goal position to the range [-1, 1]
            g_min = -2
            g_max = 2
            self.goal = np.array(self.goal, dtype=np.float32)
            self.goal = 2 * (self.goal - g_min) / (g_max - g_min) + (-1)

            # Combine the normalized pedestrian positions, scan data, and goal position into a single observation vector
            self.observation = np.concatenate((self.ped_pos, self.scan, self.goal), axis = None)

            # Return the combined observation vector
            return self.observation
    
    # Create a mock robot and run the function
    robot = MockRobot()
    
    # Get the observation vector
    observation = robot._get_observation()
    
    # Print the observation vector to inspect
    print(observation)

# Call the test function
test_get_observation()
