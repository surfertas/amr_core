#!/usr/bin/env python
# @author Tasuku Miura
# @license MIT

import rospy
import std_msgs.msg
import cv2
from cv_bridge import CvBridge, CvBridgeError
from amr_tf_controller_service.msg import Command2D
from amr_tf_controller_service.srv import PredictCommand

import numpy as np

import tensorflow as tf

from controller import Controller


class TFController(object):

    def __init__(self, model_path, model_name):
        self._bridge = CvBridge()
        self._init_tf_controller_service()

        # Create Controller class instance 
        self._controller = Controller(model_path, model_name)
       
    def _init_tf_controller_service(self):
        """ Initialize nn controller service. """
        self._service_tf_controller = rospy.Service(
            'amr_tf_controller_service',
            PredictCommand,
            self._tf_controller_handler
        )
        rospy.loginfo("TF controller service initialized")

    def _tf_controller_handler(self, req):
        """ Handler for tf controller service.
        Args:
            req - request
        Returns:
            res - ROS response, commands
        """
        try:
            cv_img = cv2.imdecode(np.fromstring(req.image.data, np.uint8), 1)
        except CvBridgeError as e:
            rospy.logerr(e)

        output = run_inference(self._graph, cv_img)

        cmd_msg = Command2D()
        cmd_msg.header =  std_msgs.msg.Header()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.lazy_publishing = True
        cmd_msg.x = output[0]  # throttle
        cmd_msg.y = output[1]  # steer

        return cmd_msg


def main():
    rospy.init_node('amr_tf_controller_service')
    print("initialized")    
    if rospy.has_param('/model_path'):
        model_path = rospy.get_param('/model_path')

    if rospy.has_param('/model_name'):
        model_name = rospy.get_param('/model_name')

    controller = TFController(model_path, model_name)
    rospy.spin()

if __name__ == '__main__':
    main()
