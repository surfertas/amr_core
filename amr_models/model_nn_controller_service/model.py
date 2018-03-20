# @author Tasuku Miura

import torch
import torch.nn as nn
import torch.nn.functional as F
from torchvision import models

class ResNet18FT(nn.Module):

    """
    Fine tune ResNet18
    """

    def __init__(self):
        super(ResNet18FT, self).__init__()
        self.resnet18ft = models.resnet18(pretrained=True)
        num_ftrs = self.resnet18ft.fc.in_features
        self.resnet18ft.fc = nn.Linear(num_ftrs, 2)
        self.resnet18ft.cuda()

    def forward(self, x):
        x = self.resnet18ft(x)
        return x
