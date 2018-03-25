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


class ResNet18FE(nn.Module):

    """
    Feature Extract ResNet18
    """

    def __init__(self):
        super(ResNet18FE, self).__init__()
        self.resnet18fe = models.resnet18(pretrained=True)
        # Set grad to false to freeze.
        for param in self.resnet18fe.parameters():
            param.requires_grad = False

        # Default sets requires_grad to false,
        # so final fc can be optimized.
        num_ftrs = self.resnet18fe.fc.in_features
        self.resnet18fe.fc = nn.Linear(num_ftrs, 2)
        self.resnet18fe.cuda()

    def forward(self, x):
        x = self.resnet18fe(x)
        return x
