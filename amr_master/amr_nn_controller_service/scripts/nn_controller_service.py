#!/usr/bin/env python
# @author Tasuku Miura

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from amr_nn_controller_service.msg import Command2D
from amr_nn_controller_service.srv import PredictCommand

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.autograd import Variable

from models import *
from utils import run_inference

class NNController(object):

    def __init__(self, model_path):
        self._model_path = model_path
        self._m = None
        self._bridge = CvBridge()
        self._init_model()
        self._init_nn_controller_service()

    def _init_model(self):
        print("here")
        self._m = ResNet18FT()
        m_tmp = torch.load(self._model_path)
        # Need 'state_dict' key to get saved model. 
        self._m.load_state_dict(m_tmp['state_dict'])
        rospy.loginfo('Loading weights... Done!')
        self._m.cuda()
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
            #img = self._bridge.imgmsg_to_cv2(
            #    req.image,
            #    desired_encoding="passthrough"
            #)
        except CvBridgeError as e:
            rospy.logerr(e)

        print(cv_img.shape)
        # TODO: Call predict function using img as input
        output = run_inference(self._m, cv_img)

        cmd_msg = Command2D()
        cmd_msg.header =  None # TODO: create header http://docs.ros.org/api/std_msgs/html/msg/Header.html
        cmd_msg.lazy_publishing = True
        cmd_msg.x = output[0]  # throttle
        cmd_msg.y = output[1]  # steer

        return PredictCommandResponse(cmd_msg)


def main():
    rospy.init_node('amr_nn_controller_service')
    if rospy.has_param('/nn_controller_path'):
        model_path = rospy.get_param('/nn_controller_path')

    controller = NNController(model_path)
    rospy.spin()

if __name__ == '__main__':
    main()
