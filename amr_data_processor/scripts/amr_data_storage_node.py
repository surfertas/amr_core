#!/usr/bin/env python
# @author Tasuku Miura
# @license: MIT

import os
import cv2
import numpy as np

# ROS related
import rospy
import message_filters
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist, TwistStamped
from amr_controller.msg import Command2D

class DataWriter(object):

    """
    Class to specify topics to save and write to disk.
    """

    def __init__(self, data_dir, img_topic, cmd_topic, capacity):
        self._data_dir = data_dir               # dir to save images
        self._img_topic = img_topic             # image topic
        self._cmd_topic = cmd_topic             # velocity commands
        self._capacity = capacity               # capacity threshold

        # Store data buffer information.
        self._img_array = []
        self._cmd_array = []

        self._setup()
        self._init_subscribers()

    def _setup(self):
        """ Setup process related to DataWriter object. """
        # Create the data directory if it doesn't exist
        # New directory is created for each initialization to keep
        # data organized.
        self._data_dir = os.path.join(self._data_dir, str(rospy.get_rostime()))
        if not os.path.isdir(self._data_dir):
            os.makedirs(self._data_dir)

    def _init_subscribers(self):
        """ Set up subscribers and sync. """
        # Initialize subscribers.
        img_sub = message_filters.Subscriber(self._img_topic, CompressedImage)
        cmd_sub = message_filters.Subscriber(self._cmd_topic, Command2D)
        subs = [img_sub, cmd_sub]

        # Sync subscribers
        self._sync = message_filters.ApproximateTimeSynchronizer(
            subs,
            queue_size=10,
            slop=0.2
        )
        self._sync.registerCallback(self._sync_sub_callback)
        rospy.loginfo("Synced subscribers initialized...")

    def _sync_sub_callback(self, img, cmd):
        """ Call back for synchronize image and command subscribers.
            arg: img - image message of type CompressedImage
            arg: cmd - velocity message of type TwistStamped
        """
        if len(self._img_array) < self._capacity:
            cv_img = cv2.imdecode(np.fromstring(img.data, np.uint8), 1)
            path = os.path.join(self._data_dir, '{}.png'.format(rospy.get_rostime()))
            cv2.imwrite(path, cv_img)

            self._img_array.append(path)
            self._cmd_array.append(cmd)

        #TODO: dump img_array _cmd_array to csv

def main():
    rospy.init_node('amr_data_storage_node')

    # get dir to store data
    if rospy.has_param('data_dir'):
        data_dir = rospy.get_param(
            'data_dir',
            '/tmp/amr_data'
        )

    if rospy.has_param('img_topic'):
        img_topic = rospy.get_param(
            'img_topic',
            '/webcam/image_raw/compressed'
        )

    if rospy.has_param('cmd_topic'):
        cmd_topic = rospy.get_param(
            'cmd_topic',
            '/amr_command_2d'
        )

    if rospy.has_param('capacity'):
        capacity = rospy.get_param(
            'capacity'
        )

    data_writer = DataWriter(
        data_dir,
        img_topic,
        cmd_topic,
        capacity
    )

    rospy.loginfo("---------Params loaded----------")
    rospy.loginfo("Data directory: {}".format(data_dir))
    rospy.loginfo("Storing image topic: {}".format(img_topic))
    rospy.loginfo("Storing command topic: {}".format(cmd_topic))
    rospy.loginfo("Store capacity: {}".format(capacity))
    rospy.spin()

if __name__ == "__main__":
    main()
