# @author Tasuku Miura

import torch
import torch.nn as nn
import torch.nn.functional as F
from torchvision import models

import cv2

from model import build_model
from config import get_cfg_defaults



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
        # NOTE: check if rgb or bgr
        #image = cv2.cvtColor(image, code=cv2.COLOR_RGB2BGR)
        
        image = torch.from_numpy(image)
        image = image.to(self.device)
        prediction = self.model(image)
        return prediction.item()




