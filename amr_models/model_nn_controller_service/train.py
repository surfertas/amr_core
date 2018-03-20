# @author Tasuku Miura
# @brief Training for controller to output steering and throttle commands given
# an image taken from a monocular camera. (Assumes CUDA enabled)
# python train.py --root-dir /home/ubuntu/ws/amr_core/amr_models/model_nn_controller_service/data
# put images and pickle file in ./data

import os
import pickle
import argparse

from skimage import io, transform
import numpy as np
import pandas as pd
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.autograd import Variable

from torch.utils.data import Dataset, DataLoader, ConcatDataset
from torchvision import transforms, utils

from data_loader import *
from transforms import *
from model import ResNet18FT
from utils import save_checkpoint, create_dir


# used for logging to TensorBoard
from tensorboard_logger import configure, log_value


def train_one_epoch(epoch, model, loss_fn, optimizer, train_loader):
    model.train()
    print("Epoch {} starting.".format(epoch))
    epoch_loss = 0
    for batch in train_loader:
        data, target = batch['image'].cuda(), batch['commands'].cuda()
        data = Variable(data).type(torch.cuda.FloatTensor)
        target = Variable(target).type(torch.cuda.FloatTensor)

        predict = model(data)
        loss = loss_fn(predict, target)
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        epoch_loss += loss.data[0]

    epoch_loss /= len(train_loader.dataset)
    print("Epoch {:.4f}: Train set: Average loss: {:.6f}\t".format(epoch, epoch_loss))
    log_value('train_loss', epoch_loss, epoch)


def validate(epoch, model, loss_fn, optimizer, valid_loader):
    model.eval()
    valid_loss = 0
    for batch in valid_loader:
        data, target = batch['image'].cuda(), batch['commands'].cuda()
        data = Variable(data, volatile=True).type(torch.cuda.FloatTensor)
        target = Variable(target).type(torch.cuda.FloatTensor)
        predict = model(data)
        valid_loss += loss_fn(predict, target).data[0]  # sum up batch loss

    valid_loss /= len(valid_loader.dataset)
    print('Valid set: Average loss: {:.6f}\n'.format(valid_loss))
    log_value('valid_loss', valid_loss, epoch)
    return valid_loss


def test(model, loss_fn, optimizer, test_loader):
    model.eval()
    images = []
    targets = []
    predicts = []
    test_loss = 0
    for batch in test_loader:
        data, target = batch['image'].cuda(), batch['commands'].cuda()
        data = Variable(data, volatile=True).type(torch.cuda.FloatTensor)
        target = Variable(target).type(torch.cuda.FloatTensor)
        output = model(data)
        test_loss += loss_fn(output, target).data[0]  # sum up batch loss

        # Store image path as raw image too large.
        images.append(batch['image_path'])
        targets.append(target.data.cpu().numpy())
        predicts.append(output.data.cpu().numpy())

    test_loss /= len(test_loader.dataset)
    print('Test set: Average loss: {:.4f}\n'.format(test_loss))

    data_dict = {
        "image": np.array(images),
        "steer_target": np.array(targets).astype('float'),
        "steer_pred": np.array(predicts).astype('float')
    }

    with open("pyt_predictions_lstm.pickle", 'wb') as f:
        pickle.dump(data_dict, f)
        print("Predictions pickled...")


def main(args):
    args.cuda = not args.no_cuda and torch.cuda.is_available()

    # Set random seed to 0
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)

    # Set file paths.
    ckpt_path = os.path.join(args.root_dir, 'output')  # checkpoint.pth.tar')
    log_path = os.path.join(args.root_dir, 'log')

    create_dir(ckpt_path)
    create_dir(log_path)

    # Configure tensorboard log dir
    configure(os.path.join(args.root_dir, 'log'))

    train_pickle_file = args.train_data
    valid_pickle_file = args.valid_data

    # Get transforms
    transforms = imagenet_transforms()
    train_transforms = transforms['train_transforms']
    pre_process = transforms['eval_transforms']

    # Set Up data
    train_data_aug = AMRControllerDataset(train_pickle_file, args.root_dir, train_transforms)
    train_data_orig = AMRControllerDataset(train_pickle_file, args.root_dir, pre_process)
    train_data = ConcatDataset([train_data_orig, train_data_aug])
    print("Train data size: {}".format(len(train_data)))

    # Create data loader
    train_loader = DataLoader(train_data, batch_size=args.batch_size, shuffle=True, num_workers=4)

    #valid_data = AMRControllerDataset(valid_pickle_file, args.root_dir, pre_process)
    #print("Valid data size: {}".format(len(valid_data)))
    #valid_loader = DataLoader(valid_data, batch_size=args.valid_batch_size, shuffle=False, num_workers=4)
    #print("Data loaded...")

    # Initiate model.
    model = ResNet18FT().cuda()

    resume = False  # set to false for now.
    if resume:
        state_dict = torch.load(ckpt_path)
        model.load_state_dict(state_dict)

    # Set up optimizer and define loss function.
    optimizer = torch.optim.Adam(model.parameters())
    loss_fn = nn.MSELoss()
    print("Model setup...")

    # Train and validate
    for epoch in range(args.epochs):
        train_one_epoch(epoch, model, loss_fn, optimizer, train_loader)
        #ave_valid_loss = validate(epoch, model, loss_fn, optimizer, valid_loader)

        is_best = True  # Save checkpoint every epoch for now.

        save_checkpoint({
            'epoch': epoch,
            'state_dict': model.state_dict(),
            'optimizer': optimizer.state_dict()
        }, is_best, os.path.join(ckpt_path, 'checkpoint.pth.tar'))

    # Test
    #test(model, loss_fn, optimizer, valid_loader)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='NN Controller')
    parser.add_argument('--root-dir', type=str, default='.',
                        help='path to root')
    parser.add_argument('--train-data', type=str, default='predictions.pickle',
                        help='filename containing train data')
    parser.add_argument('--valid-data', type=str, default='valid_data.pickle',
                        help='filename containing valid data')
    parser.add_argument('--batch-size', type=int, default=32, metavar='N',
                        help='input batch size for training (default: 32)')
    parser.add_argument('--valid-batch-size', type=int, default=32, metavar='N',
                        help='input batch size for validation (default: 32)')
    parser.add_argument('--epochs', type=int, default=10, metavar='N',
                        help='number of epochs to train (default: 10)')
    parser.add_argument('--no-cuda', action='store_true', default=False,
                        help='disables CUDA training')
    parser.add_argument('--seed', type=int, default=0, metavar='S',
                        help='random seed (default: 0)')
    parser.add_argument('--log-interval', type=int, default=10, metavar='N',
                        help='how many batches to wait before logging training status')

    args = parser.parse_args()
    main(args)
