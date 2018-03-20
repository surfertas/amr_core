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

    def __init__(self, pickle_file, root_dir, transform=None):
        self._pickle_file = pickle_file
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
            'commands': self._frames[['steer','throttle']].iloc[idx].as_matrix()
        }

    def _get_frames(self):
        pickle_path = os.path.join(self._root_dir, self._pickle_file)
        with open(pickle_path, 'rb') as f:
            pdict = pickle.load(f)
        
        img_df = pd.DataFrame(pdict['images'], columns=['images'])
        controls_df = pd.DataFrame(pdict['control_commands'], columns=['steer', 'throttle'])
        df = pd.concat([img_df, controls_df], axis=1)
        return df
