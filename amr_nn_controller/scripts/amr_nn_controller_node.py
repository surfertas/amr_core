#!/usr/bin/env python
# @author Tasuku Miura

import rospy

import std_msgs.msg
from amr_controller.msg import Command2D
from amr_nn_controller.srv import *


class NNController(object):

    """
    Given an image input, a dl model infers commands.
    Controller subscribes to raw image topic, and calls the
    'amr_nn_controller_service'.
    """

    def __init__(self, img_topic, cmd_topic):
        self._img_topic = img_topic
        self._cmd_topic = cmd_topic

        rospy.wait_for_service('amr_nn_controller_service')
        rospy.loginfo("Service 'amr_nn_controller_service' is available...")
        self._serve_get_prediction = rospy.ServiceProxy(
            'amr_nn_controller_service',
            PredictCommand,
            persistent=True
        )

        self._image_sub = rospy.Subscriber(self._img_topic, CompressedImage, self._sub_callback)
        self._cmd_pub = rospy.Publisher(self._cmd_topic, Command2D, queue_size=10)

    def _sub_callback(self, img_msg):
        """ Handler for image subscriber. """
        try:
            resp = self._serve_get_prediction(img_msg)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {}".format(e))
        self._cmd_pub.publish(resp.commands)


if __name__ == '__main__':
    rospy.init_node('amr_nn_controller_node')
    img_topic = rospy.get_param('img_topic')
    cmd_topic = rospy.get_param('cmd_topic')
    nn_controller = NNController(img_topic, cmd_topic)
    rospy.spin()
