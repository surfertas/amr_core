# @author Tasuku Miura
# @brief Transforms used with Net model

import torch
from torchvision import transforms, utils


def net_transforms():
    eval_transforms = transforms.Compose([
        transforms.ToPILImage(),
        transforms.Resize((124, 124)),
        transforms.ToTensor()])  # transform it into a

    return {
        'eval_transforms': eval_transforms
    }
