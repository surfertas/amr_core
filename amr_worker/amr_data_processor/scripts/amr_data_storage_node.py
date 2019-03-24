#!/usr/bin/env python
# @author Tasuku Miura
# @license: MIT

import os
import cv2
import numpy as np
import pandas as pd
import pickle

# ROS related
import rospy
import message_filters
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import Joy
from amr_controller.msg import Command2D


class DataWriter(object):

    """
    Class to specify topics to save and write to disk.
    """

    def __init__(self, data_dir, img_topic, cmd_topic, joy_topic, save_frequency, capacity):
        """ Initialize DataWriter object.
        Args:
            data_dir - dir to save images
            img_topic - image topic
            save_frequency - rate at which to save data to disk
            capacity - capacity of data store
        """
        self._data_dir = data_dir
        self._img_topic = img_topic
        self._cmd_topic = cmd_topic
        self._joy_topic = joy_topic

        self._capacity = capacity

        # Store data buffer information.
        self._save_frequency = save_frequency
        self._img_path_array = []
        self._cmd_array = []

        # If the data collection process is on or off.
        self._process_on = False
        self._setup_running = False

        self._setup_data_dir()
        self._init_subscribers()

    def _setup_data_dir(self):
        """ Setup process related to DataWriter object. """
        # Create the data directory if it doesn't exist
        # New directory is created for each initialization to keep
        # data organized.
        self._session_dir = os.path.join(self._data_dir, str(rospy.get_rostime()))
        if not os.path.isdir(self._session_dir):
            os.makedirs(self._session_dir)

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
        
        # Setup joy stick subscriber
        self.joy_sub = rospy.Subscriber(self._joy_topic, Joy, self._controller_callback)

    def _save_data_info(self):
        """ Call periodically to save as input (path) and label to be used for
            training models.
        """
        data = {
            "images": np.array(self._img_path_array),
            "control_commands": np.array(self._cmd_array)
        }
        with open(os.path.join(self._session_dir, "predictions.pickle"), 'w') as f:
            pickle.dump(data, f)
    
        rospy.loginfo("Predictions saved to {}...".format(self._session_dir))

    def _sync_sub_callback(self, img, cmd):
        """ Call back for synchronize image and command subscribers.
        Args:
            img - image message of type CompressedImage
            cmd - velocity message of type TwistStamped
        """
        if len(self._img_path_array) < self._capacity and self._process_on:
            cv_img = cv2.imdecode(np.fromstring(img.data, np.uint8), 1)
            path = os.path.join(self._session_dir, '{}.png'.format(rospy.get_rostime()))
            cv2.imwrite(path, cv_img)

            self._img_path_array.append(path)
            self._cmd_array.append([cmd.x, cmd.y])

            if len(self._cmd_array) % self._save_frequency == 0:
                self._save_data_info()

    def _controller_callback(self, joy_msg):
        # http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Joy.html
        # http://wiki.ros.org/ps3joy
        # https://answers.ros.org/question/260212/start-launchfile-from-service/
            
        if joy_msg.buttons[9] == 1 and self._setup_running == False:
            if self._process_on:
                rospy.loginfo(
                    "Stop the current data collection process first before initializing new a process")
            else:
                self._setup_running = True
                self._setup_data_dir()

                # Reinitialize arrays
                self._img_path_array = []
                self._cmd_array = []
                rospy.loginfo("Reinitialized the data arrays.")
                self._process_on = True
                self._setup_running = False
                rospy.loginfo(
                    "Starting new data collection process at {}".format(self._session_dir))
            
        if joy_msg.buttons[11] == 1 and self._process_on == False:
            self._process_on = True
            rospy.loginfo(
                "Starting data collection and storage at {}".format(self._session_dir))

        if joy_msg.buttons[10] == 1 and self._process_on == True:
            self._process_on = False

            # Pickle with latest samples
            self._save_data_info()
            rospy.loginfo(
                "Stopping data collection and storage.")


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

    if rospy.has_param('joy_topic'):
        joy_topic = rospy.get_param(
            'joy_topic',
            '/joy'
        )

    if rospy.has_param('capacity'):
        capacity = rospy.get_param(
            'capacity',
            1000000
        )

    if rospy.has_param('save_frequency'):
        save_frequency = rospy.get_param(
            'save_frequency',
            100
        )

    data_writer = DataWriter(
        data_dir,
        img_topic,
        cmd_topic,
        joy_topic,
        save_frequency,
        capacity
    )

    rospy.loginfo("---------Params loaded----------")
    rospy.loginfo("Data directory: {}".format(data_dir))
    rospy.loginfo("Storing image topic: {}".format(img_topic))
    rospy.loginfo("Storing command topic: {}".format(cmd_topic))
    rospy.loginfo("Data pickle frequency: {}".format(save_frequency))
    rospy.loginfo("Store capacity: {}".format(capacity))
    rospy.spin()

if __name__ == "__main__":
    main()
