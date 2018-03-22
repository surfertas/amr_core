
import torch
import numpy as np
from torch.autograd import Variable

def run_inference(model, img, use_cuda=1):
    """ Runs inference given a PyTorch model.
    Args:
        model - pytorch model
        img - numpy darray
        use_cuda - 1 is True, 0 is False
    Returns:
        output.data[0] - throttle command
        output.data[1] - steer command
    """
    model.eval()
    img = torch.from_numpy(img.transpose(2,0,1)).float().div(255.0).unsqueeze(0)
    if use_cuda:
        img = img.cuda()
  
    img = torch.autograd.Variable(img)
    output = model(img)
    return output.data[0], output.data[1]
