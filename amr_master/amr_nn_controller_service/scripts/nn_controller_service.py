#!/usr/bin/env python
# @author Tasuku Miura
# @license MIT

import rospy
import std_msgs.msg
import cv2
from cv_bridge import CvBridge, CvBridgeError
from amr_nn_controller_service.msg import Command2D
from amr_nn_controller_service.srv import PredictCommand

import numpy as np
import torch
from torch.autograd import Variable

from models import PilotNet
from net_transforms import basenet_transforms
from utils import dotdict


class NNController(object):

    def __init__(self, model_path, params):
        self._model_path = model_path
        self.params = params
        self._m = None
        self._bridge = CvBridge()
        self._init_model()
        self._init_nn_controller_service()

    def _init_model(self):
        print("here")
        self._m = PilotNet(self._model_path)

        #m_tmp = torch.load(self._model_path)
        # Need 'state_dict' key to get saved model.
#        self._m.load_state_dict(m_tmp['state_dict'])
#        rospy.loginfo('Loading weights... Done!')
#        self._m.cuda()
        rospy.loginfo('NNController model initialized...')

    def _init_nn_controller_service(self):
        """ Initialize nn controller service. """
        self._service_nn_controller = rospy.Service(
            'amr_nn_controller_service',
            PredictCommand,
            self._nn_controller_handler
        )
        rospy.loginfo("NN controller service initialized")

    def _nn_controller_handler(self, req):
        """ Handler for nn controller service.
        Args:
            req - request
        Returns:
            res - ROS response, commands
        """
        try:
            cv_img = cv2.imdecode(np.fromstring(req.image.data, np.uint8), 1)
        except CvBridgeError as e:
            rospy.logerr(e)

        output = self._run_inference(self._m, cv_img)

        cmd_msg = Command2D()
        cmd_msg.header = std_msgs.msg.Header()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.lazy_publishing = True
        cmd_msg.x = output[0]  # throttle
        cmd_msg.y = output[1]  # steer

        return cmd_msg

    def _run_inference(self, model, img, use_cuda=1):
        """ Runs inference given a PyTorch model.
        Args:
            model - pytorch model
            img - numpy darray
            use_cuda - 1 is True, 0 is False
        Returns:
            throttle - throttle command
            steer - steer command
        """
        trans = basenet_transforms()['eval_transforms']
        img = trans(torch.from_numpy(img.transpose(2, 0, 1))).unsqueeze(0)
        if use_cuda:
            img = img.cuda()

        img = torch.autograd.Variable(img)
        # Cuda tensor to numpy doesnt support GPU, use .cpu() to move to host mem.
        steer = model(img).data.cpu().numpy()[0]
        print(steer)
	throttle = 0.3
        return throttle, steer


def main():
    rospy.init_node('amr_nn_controller_service')
    if rospy.has_param('/nn_controller_path'):
        model_path = rospy.get_param('/nn_controller_path')

    if rospy.has_param('/params'):
        # convert dict to work with dot notation.
        params = dotdict(rospy.get_param('/params'))

    print(params.num_channels)
    controller = NNController(model_path, params)
    rospy.spin()


if __name__ == '__main__':
    main()
