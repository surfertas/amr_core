# @author Tasuku Miura

import torch
import torch.nn as nn
import torch.nn.functional as F
from torchvision import models

import cv2

from PIL import Image

from model import build_model
from config import get_cfg_defaults

from net_transforms import basenet_transforms

class PilotNet(object):
    def __init__(self, model_path):
        self.cfg = get_cfg_defaults()
        self.model = build_model(self.cfg)
        self.device = torch.device(self.cfg.MODEL.DEVICE)
        self.model.to(self.device)
        self.model.train(False)
	self.state_dict = torch.load(model_path)
        self.model.load_state_dict(torch.load(model_path)['state_dict'])

    def forward(self, image):
	image = self._preprocess(image)
        transform = basenet_transforms(self.cfg)['eval_transformer']
	image = transform(Image.fromarray(image))
        image = image.unsqueeze(0)        
        image = image.to(self.device)
        prediction = self.model(image)
        return prediction.item()

    def _preprocess(self, image):
        # crop image (remove useless information)
        cropped = image[range(*self.cfg.IMAGE.CROP_HEIGHT), :, :]
        return cropped


