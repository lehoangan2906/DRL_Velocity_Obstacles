#!/usr/bin/python3

import torch
import torch.nn as nn
import numpy as np
import random
import os

import gym
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor

# GLobal variables and seed settings

SEED1 = 1337

# Helper functions
def set_seed(seed):
    """
    Set the seed for all the random number generators to ensure deterministic behavior

    Args:
        seed (int): The seed value to use for RNGs.
    """

    torch.manual_seed(seed) # Set seed for CPU computations
    torch.cuda.manual_seed_all(seed)    # Set seed for GPU computations
    torch.backends.cudnn.deterministics =True   #Make CuDNN deterministic
    torch.backends.cudnn.benchmark =False   # disable cuDNN benchmarking for deterministic results
    random.seed(seed)      # Set the seed for python's built-in RNG
    os.environ['PYTHONHASHSEED'] = str(seed)    # Set seed for Python hash functions


# ================================================================================
#
# ResNet blocks and utility functions
#
# ================================================================================

def conv3x3(in_planes, out_planes, stride=1, groups=1, dilation=1):
    """
    3x3 convolution with padding.

    Args:
        in_planes (int): Number of input channels.
        out_planes (int): Number of output channels.
        stride (int): Stride of the convolution.
        groups (int): Number of blocked connections from input channels to output channels
        dilation (int): Spacing between kernel elements.

    Returns:
        nn.Conv2d: Convolutional layer.
    """

    return nn.Conv2d(in_planes, out_planes, kernel_size=3, stride=stride,
                     padding=dilation, groups=groups, bias=False, dilation=dilation)


def conv1x1(in_planes, out_planes, stride=1):
    """
    1x1 convolution.

    Args:
        in_planes (int): Number of input channels.
        out_planes (int_): Number of output channels.
        stride (int): Stride of the convolution.

    Returns:
        nn.Conv2d: Convolutional layer.
    """

    return nn.Conv2d(in_planes, out_planes, kernel_size=1, stride=stride, bias=False)


class Bottleneck(nn.Module):
    """
    Bottleneck layer for ResNet, which is a building block of the CNN architecture.

    This implementation follows the ResNet V1.5 variant, which places the stride for 
    downsampling at the 3x3 convolution layer.

    Args:
        inplanes (int): Number of input channels.
        planes (int): Number of output channels.
        stride (int): Stride of the convolution.
        downsample (nn.Module): Downsampling layer.
        groups (int): Number of groups in the convolution.
        base_width (int): Base width of the convolution.
        dilation (int): Dilation rate for the convolution.
        norm_layer (nn.Module): Normalization layer.
    """

    expansion = 2   # Expansion factor for the number of output channels.


    def __init__ (self, inplanes, planes, stride=1, downsample=None, groups=1, 
                  base_width=64, dilation=1, norm_layer=None):
        super(Bottleneck, self).__init__()

        if norm_layer is None:
            norm_layer = nn.BatchNorm2d     # Use batch normalization by default

        width = int(planes * (base_width / 64.)) * groups  # Calculate the width of the layer


        # Convolutional layers with batch normalization and ReLU activation.
        self.conv1 = conv1x1(inplanes, width)   # First 1x1 convolution.
        self.bn1 = norm_layer(width)    # Batch normalization for the first conv layer
        self.conv2 = conv3x3(width, width, stride, groups, dilation)    # 3x3 convolutional layer 
        self.bn2 = norm_layer(width)    # Batch normalization for the second conv layer
        self.conv3 = conv1x1(width, planes * self.expansion)    # Second 1x1 convolutional layer
        self.bn3 = norm_layer(planes * self.expansion)  # Batch norlaization for the third convolutional layer
        self.relu = nn.ReLU(inplace=True)   # ReLU activation

        # Optional downsampling layer to match dimensions
        self.downsample = downsample
        self.stride = stride

    
    def forward(self, x): # It's fucking easy to understand the flow
        """
        Forward pass through the bottleneck layer

        Args:
            x (torch.Tensor): Input tensor.

        Returns:
            torch.Tensor: Output tensor after passing through the bottleneck block.
        """

        identity = x    # Store the input tensor for the residual connection.

        # Pass the input through the convolutional layers.
        out = self.conv1(x)
        out = self.bn1(out)
        out = self.relu(out)

        out = self.conv2(out)
        out = self.bn2(out)
        out = self.relu(out)

        out = self.conv3(out)
        out = self.bn3(out)

        # Downsample if necessary and add the residual connection
        if self.downsample is not None:
            identity = self.downsample(x)

        out += identity # Add the residual connection
        out = self.relu(out)    # Apply ReLU activation

        return out
    

# ----------------------------------------------------------------------------------
#
# Custom CNN Model Definition
#
# ----------------------------------------------------------------------------------

class CustomCNN(BaseFeaturesExtractor):
    """
    Custom CNN model used in the DRL-VO network for feature extraction.

    This model is based on a ResNet architecture with bottleneck layers and
    is designed to extract features from the input observations, including
    pedestriand positions, LiDAR scans, and the goal position.

    Args:
        observation_space (gym.Space): Observation space of the environment.
        features_dim (int): Number of features to be extracted  (output dimension).
    """

    def __init__(self, observation_space: gym.spaces.Box, features_dim: int = 256):
        super (CustomCNN, self).__init__(observation_space, features_dim)

        # Define network parameters
        block = Bottleneck  # Define the bottleneck block type
        layers = [2, 1, 1]  # Define the number of blocks in each layer.
        zero_init_residual = True   # Initialize the residual layers with zeros
        groups = 1  # Number of groups for grouped convolution
        width_per_group = 64    # Base width of the convolutional layers
        replace_stride_with_dilation = None     # Dilation settings for strides
        norm_layer = nn.BatchNorm2d     # Use batch normalization for layers.


        # Initialize network layers
        self.inplanes = 64  # Initial number of input channels
        self.dilation = 1   # Initial dilation rate
        
        if replace_stride_with_dilation is None:
            replace_stride_with_dilation = [False, False, False] # Default dilation configuration
        
        self.groups = groups  # Number of groups for grouped convolution
        self.base_width = width_per_group # Base width of the convolutional layers


        # Initial convolutional layer
        self.conv1 = nn.Conv2d(3, self.inplanes, kernel_size=3, stride=1, padding=1, bias = False)
        self.bn1 = norm_layer(self.inplanes)    # Batch normalization for the initial layer
        self.relu = nn.ReLU(inplace = True)    # ReLU activation after batch normalization
        self.maxpool = nn.MaxPool2d(kernel_size=3, stride=1, padding=1) # Max pooling layer


        # Residual layers (ResNet blocks)
        self.layer1 = self._make_layer(block, 64, layers[0])    # First residual layer
        self.layer2 = self._make_layer(block, 128, layers[1], stride=2, dilate=replace_stride_with_dilation[0])  # Second residual layer
        self.layer3 = self._make_layer(block, 256, layers[2], stride=2, dilate=replace_stride_with_dilation[1])  # Third residual layer


        # Additional convolutional layers for feature extraction
        self.conv2_2 = nn.Sequential(
            nn.Conv2d(in_channels=256, out_channels=128, kernel_size=(1, 1), stride = (1, 1), padding=(0, 0)),
            nn.BatchNorm2d(128),    # Batch normalization
            nn.ReLU(inplace=True),  # ReLU activation

            nn.Conv2d(in_channels=128, out_channels=128, kernel_size=(3, 3), stride = (1, 1), padding= (1, 1)),
            nn.BatchNorm2d(128),    # Batch normalization
            nn.ReLU(inplace=True),  # ReLU activation

            nn.Conv3d(in_channels=128, out_channels=256, kernel_size=(1, 1), stride = (1, 1), padding = (0, 0)),
            nn.BatchNorm2d(256)    # Batch normalization
        )


        self.downsample2 = nn.Sequential(
            nn.Conv2d(in_channels=128, out_channels=256, kernel_size=(1, 1), stride=(2, 2), padding=(0, 0)),
            nn.BatchNorm2d(256)   # Batch normalization
        )



        self.relu2 = nn.ReLU(inplace = True)    # ReLU activation



        self.conv3_2 = nn.Sequential(
            nn.COnv2d(in_channels=512, out_channels=256, kernel_Size=(1, 1), stride = (1, 1), padding = (0, 0)),
            nn.BatchNorm2d(256),    # Batch normalization
            nn.ReLU(inplace=True),  # ReLU activation

            nn.Conv2d(in_channels=256, out_channels=256, kernel_size=(3, 3), stride=(1,1), padding=(1, 1)),
            nn.BatchNorm2d(256),  # Batch normalization
            nn.ReLU(inplace=True),  # ReLU activation

            nn.Conv2d(in_channels=256, out_channels=512, kernel_size=(1, 1), stride=(1,1), padding=(0, 0)),
            nn.BatchNorm2d(512)  # Batch normalization
        )


        self.downsample3 = nn.Sequential(
            nn.Conv2d(in_channels=64, out_channels=512, kernel_size=(1, 1), stride=(4,4), padding=(0, 0)),
            nn.BatchNorm2d(512)  # Batch normalization
        )


        self.relu3 = nn.ReLU(inplace=True)  # ReLU activation

        # Adaptive pooling and final fully connected layer
        self.avgpool = nn.AdaptiveAvgPool2d((1, 1))  # Adaptive average pooling
        self.linear_fc = nn.Sequential(
            nn.Linear(256 * block.expansion + 2, features_dim),  # Combine extracted features and goal
            nn.ReLU()  # ReLU activation after fully connected layer
        )

        # Zero-initialize the last batch norm in each residual block, making each block behave like an identity initially.
        if zero_init_residual:
            for m in self.modules():
                if isinstance(m, Bottleneck):
                    nn.init.constant_(m.bn3.weight, 0)

    
    def _make_layer(self, block, planes, blocks, stride=1, dilate=False):
        """
        Create a layer composed of multiple bottleneck blocks.

        Args:
            block (Bottleneck): Block type to use for the layer.
            planes (int): Number of output channels.
            blocks (int): Number of blocks to stack.
            stride (int): Stride of the first block.
            dilate (bool): Whether to apply dilation in this layer.

        Returns:
            nn.Sequential: Layer composed of multiple bottleneck blocks.
        """

        norm_layer = self._norm_layer
        downsample = None
        previous_dilation = self.dilation

        if dilate:
            self.dilation *= stride
            stride = 1
        
        if stride != 1 or self.inplanes != planes * block.expansion:
            downsample = nn.Sequential(
                conv1x1(self.inplanes, planes * block.expansion, stride),
                norm_layer(planes * block.expansion),
            )

        layers = []
        layers.append(block(self.inplanes, planes, groups=self.groups,
                            base_width=self.base_width, dilation=self.dilation,
                            norm_layer=norm_layer))
        
        return nn.Sequential(*layers)
    

    def _forward_impl(self, ped_pos, scan, goal):
        """
        Internal forward pass implementation, which processes pedestrian positions, 
        LiDAR scans, and the goal position through the network.

        Args:
            ped_pos (torch.Tensor): Tensor containing pedestrian position maps.
            scan (torch.Tensor): Tensor containing processed LiDAR scan data.
            goal (torch.Tensor): Tensor containing the goal position.

        Returns:
            torch.Tensor: Output feature tensor.
        """

        # Reshape pedestrian position and scan data for input into the network.
        ped_in = ped_pos.reshape(-1, 2, 80, 80)
        scan_in = scan.reshape(-1, 1, 80, 80)
        fusion_in = torch.cat((scan_in, ped_in), dim = 1)   # Concatenate pedestrian and LiDAR data


        # Pass through intial convolutional and residual layers.
        x = self.conv1(fusion_in)   # Initial convolutional layer
        x = self.bn1(x) # Batch normalization
        x = self.relu(x)    # ReLU activation
        x = self.maxpool(x)     # Max pooling layer

        identity3 = self.downsample3(x)     # Downsample for residual connection
        x = self.layer1(x)  # First residual layer
        
        identity2 = self.downsample2(x)    # Downsample for residual connection
        x = self.layer2(x)  # Second residual layer
        x = self.conv2_2(x)     # Additional convolutional layer
        x += identity2 # Add residual connection
        x = self.relu2(x)   # ReLU activation

        x = self.layer3(x) # Third residual layer
        x = self.conv3_2(x)     # Additional convolutional layer
        x += identity3  # Add residual connection
        x = self.relu3(x)  # ReLU activation

        # Perform adaptive pooling and flatten the output
        x = self.avgpool(x)  # Adaptive average pooling
        fusion_out = torch.flatten(x, 1)  # Flatten the output

        # Process goal position
        goal_in = goal.reshape(-1, 2)
        goal_out = torch.flatten(goal_in, 1)  # Flatten the goal position

        # Combine feature maps and goal, then pass through the final fully connected layer
        fc_in = torch.cat((fusion_out, goal_out), dim=1)
        x = self.linear_fc(fc_in)  # Final fully connected layer

        return x
    
    def forward(self, observation: torch.Tensor) -> torch.Tensor:
        """
        Forward pass through the network that processes the entire observation vector.

        Args:
            observation (torch.Tensor): Input observation tensor.

        Returns:
            torch.Tensor: Output feature tensor.
        """

        # Extract pedestrian position, LiDAR scan, and goal position from the observation
        ped_pos = observation[:, :12800]     # First portion is pedestrian positions
        scan = observation[:, 12800:19200]    # LiDAR scan
        goal = observation[:, 19200:]    # Goal position

        # Process the input through the network
        return self._forward_impl(ped_pos, scan, goal) # Process the input through the network
        