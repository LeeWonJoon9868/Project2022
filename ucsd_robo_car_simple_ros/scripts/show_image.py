#!/usr/bin/python
import rospy
from sensor_msgs.msg import Image
import cv2,cv_bridge
#from cv_bridge import CvBridge

class Follower:
	def __init__ (self):
		self.bridge = cv_bridge.CvBridge()
		cv2.namedWindow("window",1)
		self.image_sub = rospy.Subscriber('/orb_slam2_rgbd/debug_image',Image, self.image_callback)
	def image_callback(self,msg):
		image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
		cv2.imshow("window",image)
		cv2.waitKey(3)

rospy.init_node("follower")
follower=Follower()
rospy.spin()
