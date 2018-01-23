#!/usr/bin/env python
# @author Tasuku Miura

import rospy

import std_msgs.msg
from amr_controller.msg import Command2D
from amr_service_hub.srv import *
from amr_nn_models import *


class NNControllerService(object):

    def __init__(self, joystick_params):
        self._x_gain = joystick_params['x_gain']
        self._y_gain = joystick_params['y_gain']
        self._init_predict_service()
    
    def _init_predict_service(self):
        # Service to handle calls for prediction, when auto driving.
        self._service_get_prediction = rospy.Service(
            'service_get_prediction',
            PredictCommand,
            self._get_prediction_handler
        )
        rospy.loginfo("Get prediction service initialized...")

    def _get_prediction_handler(self, req):
        """ Handler for service_get_prediction. """
        # https://www.tensorflow.org/programmers_guide/saved_model
        cv_image = cv2.imdecode(np.fromstring(req.image.data, np.uint8), 1)

        # TODO: Need to import model and integrate prediction API.
        cmd = self._sess.run([self._predict],
            feed_dict={self._inputs: np.array([cv_image])}
        )
        # Convert prediction to ROS message type.
        cmd_msg = Command2D()
        
        # Create header
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        cmd_msg.header = header

        cmd_msg.lazy_publishing = True
        cmd_msg.x = self._x_gain * cmd[0]
        cmd_msg.y = self._y_gain * cmd[1]
        return cmd_msg
    
 __name__=='__main__':
    rospy.init_node('amr_nn_controller_service')
    joystick_params = rospy.get_param('joy_stick')
    nn_controller_service = NNControllerService(joystick_params)
    rospy.spin()
