#!/usr/bin/python3

import torch
import torch.nn as nn
import numpy as np
import numpy.matlib
import random
import os

from stable_baselines3.common.torch_layers import BaseFeaturesExtractor
import gym



# ------------------------------------------------------------------------------------------------
# Helper functions
# ------------------------------------------------------------------------------------------------


# Set the rng for reproducibility
SEED1 = 1337

def set_seed(seed: int):
    """
    Set the seed for all the random number generators to make the results deterministics.
    
    :param seed: (int) The seed for all RNGs.
    """
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)
    torch.backends.cudnn.deterministic = True
    torch.backends.cudnn.benchmark = False
    random.seed(seed)
    os.environ['PYTHONHASHSEED'] = str(seed)




# ------------------------------------------------------------------------------------------------
# ResNet blocks
# ------------------------------------------------------------------------------------------------

# First type layer in the feature extractor module of the model
def conv3x3(in_planes: int, out_planes: int, stride = 1, groups = 1, dilation = 1) -> nn.Conv2d:
    """
    3x3 convolution with padding.

    :param in_planes: (int) Number of input channels.
    :param out_planes: (int) Number of output channels.
    :param stride: (int) Stride of the convolution.
    :param groups: (int) Number of groups for the convolution.
    :return: (nn.Conv2d) Convolution layer.
    """
    return nn.Conv2d(in_planes, out_planes, kernel_size=3, stride=stride,
                     padding = dilation, groups=groups, bias=False, dilation=dilation)

def conv1x1(in_planes: int, out_planes: int, stride=1) -> nn.Conv2d:
    """
    1x1 convolution.
    
    :param in_planes: (int) Number of input channels.
    : param out_planes: (int) Number of output channels.
    :param stride: (int) Stride of the convolution.
    :return: (nn.Conv2d) Convolution layer.
    """
    return nn.Conv2d(in_planes, out_planes, kernel_size=1, stride=stride, bias=False)


class Bottleneck(nn.Module):
    """
    A Bottleneck block used in ResNet architectures, designed to reduce the computation
    while preserving or increasing the depth of the feature map.
    """

    expansion = 2

    def __init__(self, inplanes, planes, stride=1, downsample=None, groups=1,
                 base_width=64, dilation=1, norm_layer=None):
        super(Bottleneck, self).__init__()
        if norm_layer is None: # If the normalization layer is not provided, use the default one
            norm_layer = nn.BatchNorm2d 
        width = int(planes * (base_width / 64.)) * groups
        self.conv1 = conv1x1(inplanes, width) # 1x1 convolution
        self.bn1 = norm_layer(width) # Batch normalization
        self.conv2 = conv3x3(width, width, stride, groups, dilation) # 3x3 convolution
        self.bn2 = norm_layer(width)
        self.conv3 = conv1x1(width, planes * self.expansion) # 1x1 convolution
        self.bn3 = norm_layer(planes * self.expansion)
        self.relu = nn.ReLU(inplace=True)
        self.downsample = downsample
        self.stride = stride

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """
        Perform the forward pass of the Bottleneck block in a ResNet model.

        This method takes an input tensor `x`, applies a series of convolutional, 
        batch normalization, and ReLU activation layers, and adds the result to 
        the input tensor (identity) to form a residual connection. If a downsample 
        operation is specified, it is applied to the input tensor before adding it
        to the output.

        The sequence of operations is as follows:
        1. Apply the first convolutional layer (conv1), followed by batch normalization (bn1) and ReLU activation.
        2. Apply the second convolutional layer (conv2), followed by batch normalization (bn2) and ReLU activation.
        3. Apply the third convolutional layer (conv3), followed by batch normalization (bn3).
        4. If a downsample operation is specified, apply it to the input tensor (identity).
        5. Add the (possibly downsampled) input tensor (identity) to the output from step 3.
        6. Apply ReLU activation to the result of step 5.
         

        :param x: (torch.Tensor) The input tensor.
        :return: (torch.Tensor) The output tensor after applying the Bottleneck block.
        """

        identity = x

        out = self.conv1(x)
        out = self.bn1(out)
        out = self.relu(out)

        out = self.conv2(out)
        out = self.bn2(out)
        out = self.relu(out)

        out = self.conv3(out)
        out = self.bn3(out)

        if self.downsample is not None:
            identity = self.downsample(x)

        out += identity
        out = self.relu(out)

        return out

# ------------------------------------------------------------------------------------------------
# End of ResNet blocks
# ------------------------------------------------------------------------------------------------




# ------------------------------------------------------------------------------------------------
# The model is defined here
# ------------------------------------------------------------------------------------------------

# Define the PyTorch MLP model
class CustomCNN(BaseFeaturesExtractor):
    """
    Custom CNN architecture for DRL-VO, used to extract features from observations
    before feeding them into the DRL network.

    :param observation_space: (gym.Space) THe observation space of the environment.
    :param features_dim: (int) Number of features extracted by the CNN.
    :param block: block type (Bottleneck)
    :param layers: the number of block layers

    return: none

    This method is the main function.
    """

    def __init__ (self, observation_space: gym.spaces.Box, features_dim: int = 256):
        super(CustomCNN, self).__init__(observation_space, features_dim)

        # Network parameters
        block = Bottleneck        # The block type to use in the ResNet architecture
        layers = [2, 1, 1]        # Number of blocks in each layer
        zero_init_residual = True # Whether to initialize the residual connections to zero
        groups = 1               # Number of groups for the 3x3 convolution
        width_per_group = 64     # Number of channels per group for the 3x3 convolution
        replace_stride_with_dilation = None # Replace stride with dilation in the ResNet architecture
        norm_layer = None       # Normalization layer to use in the ResNet architecture

        ######################### ped_pos net model #########################

        if norm_layer is None: # If the normalization layer is not provided, use the default one
            norm_layer = nn.BatchNorm2d
        self._norm_layer = norm_layer

        self.inplanes = 64 # Number of input channels
        self.dilation = 1  # Dilation factor for the 3x3 convolution

        if replace_stride_with_dilation is None: # If the stride is not replaced with dilation
            replace_stride_with_dilation = [False, False, False]

        self.groups = groups                 # Number of groups for the 3x3 convolution
        self.base_width = width_per_group    # Number of channels per group for the 3x3 convolution
        self.conv1 = nn.Conv2d(3, self.inplanes, kernel_size=3, stride=1, padding=1,
                               bias=False)   # 3x3 convolution
        self.bn1 = norm_layer(self.inplanes) # Batch normalization
        self.relu = nn.ReLu(inplace = True)  # ReLU activation
        self.maxpool = nn.MaxPool2d(kernel_size=3, stride=2, padding=1)

        self.layer1 = self._make_layer(block, 64, layers[0]) # First layer of the ResNet architecture
        self.layer2 = self._make_layer(block, 128, layers[1], stride=2,
                                       dilate=replace_stride_with_dilation[0]) # Second layer of the ResNet architecture
        self.layer3 = self._make_layer(block, 256, layers[2], stride=2,
                                       dilate=replace_stride_with_dilation[1]) # Third layer of the ResNet architecture
        
        self.conv2_2 = nn.Sequential(
            nn.Conv2d(in_channels=256, out_channels=128, kernel_size=(1, 1), stride=(1, 1), padding=(0, 0)),
            nn.BatchNorm2d(128),
            nn.ReLU(inplace=True),

            nn.Conv2d(in_channels=128, out_channels=128, kernel_size=(3, 3), stride=(1, 1), padding=(1, 1)),
            nn.BatchNorm2d(128),
            nn.ReLU(inplace=True),

            nn.Conv2d(in_channels=128, out_channels=256, kernel_size=(1, 1), stride=(1, 1), padding=(0, 0)),
            nn.BatchNorm2d(256)
        )

        self.downsample2 = nn.Sequential(
            nn.Conv2d(in_channels=128, out_channels=256, kernel_size=(1, 1), stride=(2, 2), padding=(0, 0)),
            nn.BatchNorm2d(256)
        )

        self.relu2 = nn.ReLU(inplace=True)

        self.conv3_2 = nn.Sequential(
            nn.Conv2d(in_channels=512, out_channels=256, kernel_size=(1, 1), stride=(1,1), padding=(0, 0)),
            nn.BatchNorm2d(256),
            nn.ReLU(inplace=True),

            nn.Conv2d(in_channels=256, out_channels=256, kernel_size=(3, 3), stride=(1,1), padding=(1, 1)),
            nn.BatchNorm2d(256),
            nn.ReLU(inplace=True),

            nn.Conv2d(in_channels=256, out_channels=512, kernel_size=(1, 1), stride=(1,1), padding=(0, 0)),
            nn.BatchNorm2d(512)
        )

        self.downsample3 = nn.Sequential(
            nn.Conv2d(in_channels=64, out_channels=512, kernel_size=(1, 1), stride=(4,4), padding=(0, 0)),
            nn.BatchNorm2d(512)
        )
        self.relu3 = nn.ReLU(inplace=True)

        self.avgpool = nn.AdaptiveAvgPool2d((1, 1))
        self.linear_fc = nn.Sequential(
            nn.Linear(256 * block.expansion + 2, features_dim),
            nn.ReLU()
        )

        for m in self.modules():
            if isinstance(m, nn.Conv2d):
                nn.init.kaiming_normal_(m.weight, mode='fan_out', nonlinearity='relu')
            elif isinstance(m, (nn.BatchNorm2d, nn.GroupNorm)):
                nn.init.constant_(m.weight, 1)
                nn.init.constant_(m.bias, 0)
            elif isinstance(m, nn.BatchNorm1d):
                nn.init.constant_(m.weight, 1)
                nn.init.constant_(m.bias, 0)
            elif isinstance(m, nn.Linear):
                nn.init.xavier_normal_(m.weight)

        # Zero-initialize the last BN in each residual branch,
        # So that the residual branch starts with zeros, and each 
        # residual block behaves like an identity.
        # This improves the model by 0.2~0.3% according to https://arxiv.org/abs/1706.02677
        if zero_init_residual:
            for m in self.modules():
                if isinstance(m, Bottleneck):
                    nn.init.constant_(m.bn3.weight, 0)           

    def _make_layer(self, block, planes, blocks, stride=1, dilate=False) -> nn.Sequential:
        """
        Helper function to create a sequential layer of Bottleneck blocks.

        :param block: (Bottleneck) The block type to use.
        :param planes: (int) Number of planes in the output.
        :param blocks: (int) Number of blocks to create.
        :param stride: (int) Stride value.
        :param dilate: (bool) Whether to use dilated convolution.
        :return: (nn.Sequential) The created layer.
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
        layers.append(block(self.inplanes, planes, stride, downsample, self.groups,
                            self.base_width, previous_dilation, norm_layer))
        self.inplanes = planes * block.expansion
        for _ in range(1, blocks):
            layers.append(block(self.inplanes, planes, groups=self.groups,
                                base_width=self.base_width, dilation=self.dilation,
                                norm_layer=norm_layer))

        return nn.Sequential(*layers)

    def _forward_impl(self, ped_pos: torch.Tensor, scan: torch.Tensor, goal: torch.Tensor) -> torch.Tensor:
        """
        Forward pass implementation that processes pedestrian positions, Lidar scan data,
        and goal points through the network.
        
        :param ped_pos: (torch.Tensor) Pedestrian positions.
        :param scan: (torch.Tensor) Lidar scan data.
        :param goal: (torch.Tensor) Goal position.
        :return: (torch.Tensor) Extracted features.
        """

        ################ Start the fusion net ################
        # Combine pedestrian positions and Lidar scan data
        ped_in = ped_pos.reshape(-1, 2, 80, 80)
        scan_in = scan.reshape(-1, 1, 80, 80)
        fusion_in = torch.cat((scan_in, ped_in), dim=1)
        
        # Pass combined input through CNN layers
        x = self.conv1(fusion_in)
        x = self.bn1(x)
        x = self.relu(x)
        x = self.maxpool(x)

        identity3 = self.downsample3(x)

        x = self.layer1(x)

        identity2 = self.downsample2(x)

        x = self.layer2(x)

        x = self.conv2_2(x)
        x += identity2
        x = self.relu2(x)

        x = self.layer3(x)
        x = self.conv3_2(x)
        x += identity3
        x = self.relu3(x)

        x = self.avgpool(x)
        fusion_out = torch.flatten(x, 1)
        ################ End of the fusion net ################

        ################ Start the goal net ################
        # Process goal position
        goal_in = goal.reshape(-1, 2)
        goal_out = torch.flatten(goal_in, 1)
        ################ End of the goal net ################

        # Combine features and goal information
        fc_in = torch.cat((fusion_out, goal_out), dim=1)
        x = self.linear_fc(fc_in)  

        return x

    def forward(self, observations: torch.Tensor) -> torch.Tensor:
        """
        Forward method that splits the input observations into pedestrian positions,
        Lidar scans, and goal positions, then processes them through the network.
        
        :param observations: (torch.Tensor) Input observation data.
        :return: (torch.Tensor) Output features.
        """
        ped_pos = observations[:, :12800]
        scan = observations[:, 12800:19200]
        goal = observations[:, 19200:]
        return self._forward_impl(ped_pos, scan, goal)