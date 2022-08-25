#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#https://github.com/juano2310/CarND-Behavioral-Cloning-P3-Juan/blob/master/drive.py
import rospy
import time
import sys
import base64
from datetime import datetime
import os
import shutil
import numpy as np
import cv2

from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import Image,  CompressedImage
from ackermann_msgs.msg import AckermannDriveStamped
from cv_bridge import CvBridge, CvBridgeError

import json
from keras.models import model_from_json, load_model
import h5py
from keras import __version__ as keras_version

ANALYSIS_FILE ="FUCK.txt"
frame_get=0
frame_out=0
end_time=0
recognition_time=0
calculation_time=0
start = 0

h5_name="model_Black_Basic_All_Laps.h5"
json_name = "model_Black_Basic_All_Laps.json"

class cmd_vel_node(object):
    def __init__(self):
            
      """ROS Subscriptions """
      #self.joy_sub = rospy.Subscriber("/joy_teleop/cmd_vel_stamped",TwistStamped,self.debug_img)
      self.cmd_sub = rospy.Subscriber("/cmd_vel",Twist,self.debug_img)
      self.debug_pub = rospy.Publisher("/image_converter/debug_video",Image, queue_size=10)

      self.image_sub = rospy.Subscriber("/camera_rgb",Image,self.cvt_image)
      self.image_pub = rospy.Publisher("/image_converter/output_video",Image, queue_size=10)
      #self.cmdVel_pub = rospy.Publisher("/platform_control/cmd_vel", Twist, queue_size=10)
      #self.cmdVelStamped_pub = rospy.Publisher('/platform_control/cmd_vel_stamped', TwistStamped, queue_size=10)
      self.cmdVel_pub = rospy.Publisher("/ackermann_cmd", AckermannDriveStamped, queue_size=10)
      self.cmdVelStamped_pub = rospy.Publisher('/ackermann_cmd_stamped', AckermannDriveStamped, queue_size=10)

      """ Variables """
      self.model_path = '/home/hanjokim/catkin_ws/src/model/' + h5_name
      self.cmdvel = AckermannDriveStamped()
      self.baseVelocity = AckermannDriveStamped()
      self.input_cmd = AckermannDriveStamped()
      self.bridge = CvBridge()
      self.latestImage = np.zeros((320, 180))
      self.outputImage = None
      self.resized_image = None
      self.debugImage = None
      self.imgRcvd = False
      self.start_time = rospy.Time.now()


    def debug_img(self, cmd):
      self.input_cmd = cmd
      throttle = self.input_cmd.linear.x
      steering =self.input_cmd.angular.z

      #print("CMD: {} {}").format(throttle,steering)

      if self.imgRcvd:

        # Get latest image
        self.debugImage = cv2.resize(self.latestImage, (320,180)) 
        height, width, channels = self.debugImage.shape

        # Text settings
        font = cv2.FONT_HERSHEY_SIMPLEX
        location = (50,50) #10,20
        fontScale = .5
        fontColor = (255,0,0)
        lineType = 2
        throttle_str = "Throttle: " + "{0:.2f}".format(throttle)
        steering_str = "Steering: " + "{0:.2f}".format(steering)

        # Print text
        cv2.putText(self.debugImage, throttle_str, location, font, fontScale, fontColor, lineType)
        cv2.putText(self.debugImage, steering_str, (10,35), font, fontScale, fontColor, lineType)

        # Draw markers
        throttle_center = int(50 + (120 - (120*(throttle/.15))))
        
        radius = 3
        circleColor = (0,0,255)
        thickness = -1

        #cv2.circle(self.debugImage, (20, throttle_center), radius, circleColor, thickness, lineType, shift=0)


        steering_center = int(160 + (140 * (steering/1.6)))
        
        #cv2.circle(self.debugImage, (steering_center, 160), radius, circleColor, thickness, lineType, shift=0)


        # Publish debug image
        cv2.imshow("debug",self.debugImage)
        self.publish(self.debugImage, self.bridge,  self.debug_pub)



    def cvt_image(self,data):  
      try:
        self.latestImage = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.start_time = rospy.Time.now()
        global frame_get, recognition_time
        recognition_time += self.start_time.to_sec() - data.header.stamp.to_sec()
        frame_get+=1	
      except CvBridgeError as e:
        print(e)
      if self.imgRcvd != True:
          self.imgRcvd = True    
        
    def publish(self, image,  bridge,  publisher):
        try:
            #Determine Encoding
            if np.size(image.shape) == 3: 
                imgmsg = bridge.cv2_to_imgmsg(image, "bgr8") 
            else:
                imgmsg = bridge.cv2_to_imgmsg(image, "mono8") 
            publisher.publish(imgmsg)  
        except CvBridgeError as e:
            print(e)
            
    def cmdVel_publish(self, cmdVelocity):
        
         # Publish Twist
         self.cmdVel_pub.publish(cmdVelocity)

         # Publish TwistStamped 
         self.baseVelocity.drive = cmdVelocity
         
         baseVelocity = AckermannDriveStamped()

         baseVelocity.drive = cmdVelocity
     
         now = rospy.get_rostime()
         baseVelocity.header.stamp.secs = now.secs
         baseVelocity.header.stamp.nsecs = now.nsecs
         self.cmdVelStamped_pub.publish(baseVelocity)
            
    
    def run(self):
         global start
         # check that model Keras version is same as local Keras version
         f = h5py.File('/home/hanjokim/catkin_ws/src/model/' + h5_name, mode='r')
         model_version = f.attrs.get('keras_version')
         keras_version_installed = None
         keras_version_installed = str(keras_version).encode('utf8')

         if model_version != keras_version_installed:
             print('You are using Keras version ', keras_version_installed, ', but the model was built using ', model_version)

         # Model reconstruction from JSON file

         with open('/home/hanjokim/catkin_ws/src/model/' + json_name, 'r') as f:
             model = model_from_json(f.read())

         model = load_model('/home/hanjokim/catkin_ws/src/model/' + h5_name)
         
         # Load weights into the new model
         print("Model loaded.")

         while np.sum(self.latestImage) == 0 :
             pass
     
         while not rospy.is_shutdown():
             # Only run loop if we have an image
             if self.imgRcvd:
                 temp_start_time = rospy.Time.now()
                 
                 # step 1: 
                 self.resized_image = cv2.resize(self.latestImage, (320,180)) 
                 
                 # step 2: 
                 image_array = np.asarray(self.resized_image)
                 
                 # step 3:
                 
                 self.cmdvel.drive.speed = 0.8
                 self.angle = float((model.predict(image_array[None, :, :, :], batch_size=1))/10)
                 self.angle = -1.57 if self.angle < -1.57 else 1.57 if self.angle > 1.57 else self.angle
                 self.cmdvel.drive.steering_angle = self.angle
                 global calculation_time, frame_out
                 frame_out+=1
                 calculation_time +=(rospy.Time.now() - temp_start_time).to_sec()

                 
                 #print(self.cmdvel.angular.z)
                 
                 self.cmdVel_publish(self.cmdvel)
                 
                 # Publish Processed Image
                 self.outputImage = self.latestImage
                 #cv2.imshow("result",self.outputImage)
                 self.publish(self.outputImage, self.bridge,  self.image_pub)
         f = open(os.path.abspath(os.path.join(os.path.realpath(__file__),'..')) + '/config/' + ANALYSIS_FILE,'a')
         f.write(
"total frame : " + str (frame_get) +
"\ndetected frame : " + str(frame_out) + 
"\nmissed frame :" + str(frame_get - frame_out) + 
"\nrecognition time: " + str(recognition_time / frame_get) + 
"\ncalculation time: " + str(calculation_time / frame_out)+ "\n\n"

)
         f.close()


    
def main(args):
  global start_time

  rospy.init_node('model_control_node', anonymous=True)

  cmd = cmd_vel_node() 
  
  cmd.run() 

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()
  
if __name__ == '__main__':
    main(sys.argv)
    
