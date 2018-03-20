# @author Tasuku Miura
# @brief Datasets using Pytorch Dataset API (Assumes CUDA enabled)

import os
import pickle
import csv

from skimage import io, transform
import pandas as pd

from torch.utils.data import Dataset, DataLoader
from torchvision import transforms, utils


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
        img_path = self._frames['images'].iloc[idx]
        img = io.imread(img_path)
        if self._transform is not None:
            img = self._transform(img)

        return {
            'image': img,
            'commands': self._frames['control_commands'].iloc[idx]
        }

    def _get_frames(self):
        return pd.read_csv(os.path.join(self._root_dir, self._csv_file))
