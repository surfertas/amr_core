# @author Tasuku Miura
# @brief Datasets using Pytorch Dataset API (Assumes CUDA enabled)

import os
import pickle

from skimage import io, transform
import pandas as pd
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.autograd import Variable

from torch.utils.data import Dataset, DataLoader
from torchvision import transforms, utils

# used for logging to TensorBoard
from tensorboard_logger import configure, log_value

from pilot_net import *

import math
import copy


class AMRControllerDataset(Dataset):

    """
    Custom dataset to handle amr controller.

    Input is an image taken from a monocular camera, with controller mapping
    image to steering and throttle commands.
    """

    def __init__(self, csv_file, root_dir, transform=None):
        self._csv_file = csv_file
        self._root_dir = root_dir
        self._transform = transform
        self._frames = self._get_frames()

    def __len__(self):
        return len(self._frames)

    def __getitem__(self, idx):
        pass

    def _get_frames(self):
        pickle_path = os.path.join(self._root_dir, self._csv_file))
        # TODO: convert pickle file to csv
        pass
