#!/usr/bin/env python
# @author Tasuk Miura

# Revised from:
# https://github.com/wroscoe/donkey/blob/master/donkeycar/parts/actuator.py
# Copyright (C) 2017 Maeve Automation - All Rights Reserved

import rospy
import Adafruit_PCA9685
from amr_controller.msg import Command2D


class PCA9685_Controller:

    '''
    Adafruit PWM controler.
    This is used for most RC Cars
    '''

    def __init__(self, channel, frequency=60):
        # Initialise the PCA9685 using the default address (0x40).
        self.pwm = Adafruit_PCA9685.PCA9685()

        self.pwm.set_pwm_freq(frequency)
        self.channel = channel

    def set_pulse(self, pulse):
        self.pwm.set_pwm(self.channel, 0, pulse)


class Controller:
    RANGE_MIN = -1
    RANGE_MAX = 1

    """
    Convenience wrapper for loading a servo controller and
    performing command scaling.
    """

    def __init__(self, p):
        """ Instantiate a servo controller and keep a local
            copy of the params.
        """
        self.controller = PCA9685_Controller(p['channel'])
        self.params = p

    def update(self, cmd):
        """ Map a command `cmd` \in [-1,1] onto [min_pulse,max_pulse]
            centered on `zero_pulse`.

            arg: cmd The input control command.
        """
        if cmd < 0:
            pulse = self.map_index(
                cmd * self.params['min_pulse_scale'],
                self.RANGE_MIN,
                0,
                self.params['min_pulse'],
                self.params['zero_pulse'])
        else:
            pulse = self.map_index(
                cmd * self.params['max_pulse_scale'],
                0,
                self.RANGE_MAX,
                self.params['zero_pulse'],
                self.params['max_pulse'])

        if self.params['verbose']:
            rospy.loginfo(
                'set ' + str(pulse) + ' to channel ' + str(self.params['channel']))

        if not self.params['debug_only']:
            self.controller.set_pulse(pulse)

    def map_index(self, idx, range1_min, range1_max, range2_min, range2_max):
        """ Map an index `idx` of range 1 onto range 2.

            arg: idx - An index into range 1.
            arg: range1_min - Minimum of range 1.
            arg: range1_max - Maximum of range 1.
            arg: range2_min - Minimum of range 2.
            arg: range2_max - Maximum of range 2.

            ret: The index into range 2 whose offset is proportional to that of idx into range 1.
        """
        alpha = (idx - range1_min) / (range1_max - range1_min)
        range2_offset = alpha * (range2_max - range2_min)
        return int(range2_min + range2_offset)


class Handler:

    """
    Handler class for control command callback.
    """

    def __init__(self, throttle_params, steering_params):
        """ Instantiate controllers for throttle and steering. """
        self.throttle_controller = Controller(throttle_params)
        self.steering_controller = Controller(steering_params)

    def callback(self, msg):
        """ Read control command from message, send to controller. """
        self.throttle_controller.update(msg.x)
        self.steering_controller.update(msg.y)


if __name__ == '__main__':
    rospy.init_node('amr_vehicle_controller')
    throttle_params = rospy.get_param('throttle_actuator')
    steering_params = rospy.get_param('steering_actuator')

    handler = Handler(throttle_params, steering_params)
    rospy.Subscriber(
        '/test/Command2D',
        Command2D,
        handler.callback)
    rospy.spin()
