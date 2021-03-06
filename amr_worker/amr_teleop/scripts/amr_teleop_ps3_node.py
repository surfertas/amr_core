#!/usr/bin/env python
# @author Tasuk Miura

# Revised from:
# Copyright (C) 2017 Maeve Automation - All Rights Reserved

import rospy
from amr_controller.msg import Command2D
from sensor_msgs.msg import Joy


class JoyStick:

    """
    Joystick class
    """

    def __init__(self, joystick_params):
        """ Instantiate joystick """

        self.x_gain = joystick_params['x_gain']
        self.y_gain = joystick_params['y_gain']
        self.x_axis = joystick_params['x_axis']
        self.y_axis = joystick_params['y_axis']
        self._sub_topic = joystick_params['subscribe_topic']
        self._pub_topic = joystick_params['publish_topic']

        self.joy_sub = rospy.Subscriber(self._sub_topic, Joy, self._joy_stick_cb)

        self.cmd_pub = rospy.Publisher(self._pub_topic, Command2D, queue_size=10)
        rospy.loginfo("Joystick initiated...")
        

    def _joy_stick_cb(self, msg):
        """ Read control command from joystick message, send to controller.
        Args:
             msg - joystick message (http://wiki.ros.org/joy)
        """
        cmd_msg = Command2D()
        cmd_msg.header = msg.header
        cmd_msg.lazy_publishing = True
        cmd_msg.x = self.x_gain * msg.axes[self.x_axis]
        cmd_msg.y = self.y_gain * msg.axes[self.y_axis]

        self.cmd_pub.publish(cmd_msg)


if __name__ == '__main__':
    rospy.init_node('amr_teleop_ps3')
    joystick_params = rospy.get_param('joy_stick')
    joystick = JoyStick(joystick_params)
    rospy.spin()
