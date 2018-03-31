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
        self._m = ResNet18FE()
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
        except CvBridgeError as e:
            rospy.logerr(e)

        output = run_inference(self._m, cv_img)

        cmd_msg = Command2D()
        cmd_msg.header =  std_msgs.msg.Header()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.lazy_publishing = True
        cmd_msg.x = output[0]  # throttle
        cmd_msg.y = output[1]  # steer

        return cmd_msg


def main():
    rospy.init_node('amr_nn_controller_service')
    if rospy.has_param('/nn_controller_path'):
        model_path = rospy.get_param('/nn_controller_path')

    controller = NNController(model_path)
    rospy.spin()

if __name__ == '__main__':
    main()
