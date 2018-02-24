#!/usr/bin/env python
# @author Tasuku Miura

from object_detection.srv import *
import rospy
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError

from darknet import Darknet
from utils import load_class_names, do_detect, plot_boxes_cv2


class ObjectDetector(object):
    """
    Uses Yolo2 to detect objects for a given image.
    """
    def __init__(self, configs_path, weights_path, voc_names, coco_names, use_cuda):
        self._configs_path = configs_path
        self._weights_path = weights_path
        self._voc_names = voc_names
        self._coco_names = coco_names
        self._use_cuda = use_cuda

        self._bridge = CvBridge()

        # Initialize model used for inference.
        self._model, self._class_names = self._init_model()
        # Initialize service.
        self._init_detection_service()

    def _init_detection_service(self):
        """ Initialize object detection service. """
        self._service_detect_objects = rospy.Service(
            'object_detection_service',
            Yolo2,
            self._detect_objects_handler
        )
        rospy.loginfo("Object detection service initialized")


    def _init_model(self):
        """ Initialize yolo model.
        Returns:
            m - model used for detection.
            class_names - name of classes.
        """
        m = Darknet(self._configs_path)
        m.print_network()
        m.load_weights(self._weights_path)
        rospy.loginfo('Loading weights... Done!')

        if m.num_classes == 20:
            file_name = self._voc_names
        elif m.num_classes == 80:
            file_name = self._coco_names
        else:
            file_name = self._voc_names
        rospy.loginfo('Number of classes: {}'.format(m.num_classes))

        class_names = load_class_names(file_name)

        if self._use_cuda:
            m.cuda()

        return m, class_names
        
    def _detect_objects_handler(self, req):
        """ Handler for object detection service.
        Args:
            req - request in the form of ROS image type.
        Returns:
            res = ROS response, image with detected objects.
        """
        try:
            cv_img = self._bridge.imgmsg_to_cv2(req.img_req, desired_encoding="passthrough")
        except CvBridgeError as e:
            rospy.logerr(e)
        bboxes = do_detect(self._model, cv_img, 0.5, 0.4, self._use_cuda)
        draw_img = plot_boxes_cv2(cv_img, bboxes, None, self._class_names)
        try:
            img = self._bridge.cv2_to_imgmsg(draw_img, "rgb8")
        except CvBridgeError as e:
            rospy.logerr(e)
        return Yolo2Response(img)
         
def main():
    rospy.init_node('amr_object_detection_service')
    if rospy.has_param('/yolo2_cfg_path'):
        configs_path = rospy.get_param('/yolo2_cfg_path')
    else:
        sys.exit("Cant get path to yolo2 configuration file.")
    
    if rospy.has_param('/yolo2_weights_path'):
        weights_path = rospy.get_param('/yolo2_weights_path')
    else:
        sys.exit("Cant get path to yolo2 weights path.")

    if rospy.has_param('/voc_names'):
        voc_names = rospy.get_param('/voc_names')
    else:
        sys.exit("Cant get voc names.")

    if rospy.has_param('/coco_names'):
        coco_names = rospy.get_param('/coco_names')
    else:
        sys.exit("Cant get coco names.")

    detector = ObjectDetector(
        configs_path,
        weights_path,
        voc_names,
        coco_names,
        use_cuda=1
    )
    rospy.spin()

if __name__=='__main__':
    main()
