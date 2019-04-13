#!/usr/bin/env python
# @author Tasuku Miura
# @license MIT

import rospy
import std_msgs.msg
from sensor_msgs.msg import Joy

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


	self.joy_sub = rospy.Subscriber('/joy', Joy, self._joy_stick_cb)

    def _init_model(self):
        self._m = PilotNet(self._model_path)
        rospy.loginfo('NNController model initialized...')

    def _init_nn_controller_service(self):
        """ Initialize nn controller service. """
        self._service_nn_controller = rospy.Service(
            'amr_nn_controller_service',
            PredictCommand,
            self._nn_controller_handler
        )
        rospy.loginfo("NN controller service initialized")


    def _joy_stick_cb(self, msg):
        """ Read control command from joystick message..
        Args:
             msg - joystick message (http://wiki.ros.org/joy)
        """
	# Make sure throttle axis set correctly, and consistent with joy configuration.
        self.throttle = msg.axes[3]
        
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

        steer = self._run_inference(self._m, cv_img)

        cmd_msg = Command2D()
        cmd_msg.header = std_msgs.msg.Header()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.lazy_publishing = True
        cmd_msg.x = self.throttle
        cmd_msg.y = steer

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
	steer = model.forward(img)
	rospy.loginfo(steer)
        return steer


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
