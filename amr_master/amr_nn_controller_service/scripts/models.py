# @author Tasuku Miura
# @brief PyTorch implementation of PilotNet (Assumes CUDA enabled)

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


class CNNController(nn.Module):

    """
    Inspired by PilotNet per Nvidia paper.
    
    """
    def __init__(self):
        super(PilotNet, self).__init__()
        self.conv1 = nn.Conv2d(3, 24, kernel_size=5, stride=2, padding=0)
        self.conv2 = nn.Conv2d(24, 36, kernel_size=5, stride=2, padding=0)
        self.conv3 = nn.Conv2d(36, 48, kernel_size=5, stride=2, padding=0)
        self.conv4 = nn.Conv2d(48, 64, kernel_size=3, stride=1, padding=0)
        self.conv5 = nn.Conv2d(64, 64, kernel_size=3, stride=1, padding=0)
        self.fc1 = nn.Linear(64 * 1 * 18, 100)
        self.fc2 = nn.Linear(100, 50)
        self.fc3 = nn.Linear(50, 10)
        self.fc4 = nn.Linear(10, 2)

    def forward(self, x):
        x = (x - 0.5) * 2.0
        x = F.relu(self.conv1(x))
        x = F.relu(self.conv2(x))
        x = F.relu(self.conv3(x))
        x = F.relu(self.conv4(x))
        x = F.relu(self.conv5(x))
        x = x.view(x.size(0), -1)
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = F.relu(self.fc3(x))
        x = self.fc4(x)
        return x

