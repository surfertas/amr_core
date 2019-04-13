# @author Tasuku Miura
# @brief Transforms used with Net model

import torch
from torchvision import transforms, utils


def basenet_transforms(cfg):
    train_transformer = transforms.Compose([
        transforms.Resize((cfg.IMAGE.TARGET_HEIGHT,cfg.IMAGE.TARGET_WIDTH)),
        transforms.ToTensor()])  # transform it into a torch tensor

    # loader for evaluation, keep separate as transformer for train can be
    # different
    eval_transformer = transforms.Compose([
        transforms.Resize((cfg.IMAGE.TARGET_HEIGHT,cfg.IMAGE.TARGET_WIDTH)),
        transforms.ToTensor()])  # transform it into a torch tensor

    return {
        'train_transformer': train_transformer,
        'eval_transformer': eval_transformer,
    }


