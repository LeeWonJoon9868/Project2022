#!/usr/bin/python
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

CAMERA_NODE_NAME = 'camera_server'
CAMERA_TOPIC_NAME = 'camera_rgb'


cv2_video_capture = cv2.VideoCapture(1)

width = cv2_video_capture.get(cv2.CAP_PROP_FRAME_WIDTH)
height = cv2_video_capture.get(cv2.CAP_PROP_FRAME_HEIGHT)
#print(width, height)
#cv2_video_capture.set(cv2.CAP_PROP_FPS,30)

cv2_video_capture.set(cv2.CAP_PROP_FRAME_WIDTH,1920)
cv2_video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT,1080)

CAMERA_FREQUENCY = 10  # Hz

## this is a test and another

def talker():
    pub = rospy.Publisher(CAMERA_TOPIC_NAME, Image, queue_size=10)
    rospy.init_node(CAMERA_NODE_NAME, anonymous=True)
    rate = rospy.Rate(CAMERA_FREQUENCY)

    while not rospy.is_shutdown():
        ret, frame = cv2_video_capture.read()
        frame = cv2.resize(frame, dsize=(320,240), interpolation=cv2.INTER_AREA)#dsize=(320,240)
        #print("camera"+str(frame.shape))

        # construct msg
        try: 
            bridge = CvBridge()
            rgb = bridge.cv2_to_imgmsg(frame,'bgr8')
            rgb.header.stamp = rospy.Time.now()
            pub.publish(rgb)
        except TypeError:
            pass
        rate.sleep()



if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
