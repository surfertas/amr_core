
import torch
import numpy as np
from torch.autograd import Variable

from transforms import imagenet_transforms

def run_inference(model, img, use_cuda=1):
    """ Runs inference given a PyTorch model.
    Args:
        model - pytorch model
        img - numpy darray
        use_cuda - 1 is True, 0 is False
    Returns:
        throttle - throttle command
        steer - steer command
    """
    model.eval()
    trans = imagenet_transforms()['eval_transforms']
    img = trans(torch.from_numpy(img.transpose(2,0,1))).unsqueeze(0)
    if use_cuda:
        img = img.cuda()
  
    img = torch.autograd.Variable(img)
    # Cuda tensor to numpy doesnt support GPU, use .cpu() to move to host mem. 
    throttle, steer = model(img).data.cpu().numpy()[0]
    print(throttle, steer)
    return throttle, steer
