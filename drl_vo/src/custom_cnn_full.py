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


