#!/usr/bin/python
import rospy
import cv2
from cv_bridge import CvBridge
import numpy as np
from std_msgs.msg import Int32, Int32MultiArray, Float32
from sensor_msgs.msg import Image
from decoder import decodeImage
import time


LANE_DETECTION_NODE_NAME = 'test'
CAMERA_TOPIC_NAME = 'camera_rgb'
CENTROID_TOPIC_NAME = '/centroid'

class Testing :
    def __init__(self):
        self.init_node = rospy.init_node(LANE_DETECTION_NODE_NAME)
        self.camera_subscriber = rospy.Subscriber(CAMERA_TOPIC_NAME, Image, self.test)
        self.bridge = CvBridge()
    def test(self, data) :
        img = self.bridge.imgmsg_to_cv2(data)
        cv2.imshow('img', img)


def main():
    lane_detector = Testing()
    rate = rospy.Rate(15)
    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()


if __name__ == '__main__':
    main()
