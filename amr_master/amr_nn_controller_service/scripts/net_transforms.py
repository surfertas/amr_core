# @author Tasuku Miura
# @brief Transforms used with PilotNet model

import torch
from torchvision import transforms, utils

def pilotnet_transforms(cfg):
    eval_transformer = transforms.Compose([
        transforms.Resize((cfg.IMAGE.TARGET_HEIGHT,cfg.IMAGE.TARGET_WIDTH)),
        transforms.ToTensor()])  # transform it into a torch tensor

    return {
        'eval_transformer': eval_transformer,
    }


