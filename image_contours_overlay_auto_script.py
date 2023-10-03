#!/usr/bin/env python

# Sample NEPI Automation Script. 
# Uses onboard ROS python library to
# 1. run sensor imagery through an image enhancement algorithm and republish to a new topic
# 2. Run until Stopped

import time
import sys
import rospy
import numpy as np
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import UInt8, Empty, String, Bool

#####################################################################################
# SETUP - Edit as Necessary ##################################
##########################################

# ROS namespace setup
BASE_NAMESPACE = "/nepi/s2x/"

###!!!!!!!! Set Camera topics and parameters !!!!!!!!
CAMERA_NAME = "nexigo_n60_fhd_webcam_audio/"
#CAMERA_NAME = "sidus_ss400/"
#CAMERA_NAME = "onwote_hd_poe/"
#CAMERA_NAME = "see3cam_cu81/"
CAMERA_NAMESPACE = BASE_NAMESPACE + CAMERA_NAME

RESOLUTION_ADJ_TOPIC = CAMERA_NAMESPACE + "idx/set_resolution_mode"
IMAGE_INPUT_TOPIC = CAMERA_NAMESPACE + "idx/color_2d_image"
IMAGE_OUTPUT_TOPIC = CAMERA_NAMESPACE + "idx/contour_2d_image"

LOW_RES_VALUE = 0
MED_RES_VALUE = 1
HIGH_RES_VALUE = 2
ULTRA_RES_VALUE = 3

###!!!!!!!! Set Enhancment topics and parameters !!!!!!!!
ENHANCED_IMAGE_TOPIC = CAMERA_NAMESPACE + "idx/enhanced_2d_image"


#####################################################################################
# Globals
#####################################################################################

contour_image_pub = rospy.Publisher(IMAGE_OUTPUT_TOPIC, Image, queue_size=10)


#####################################################################################
# Methods
#####################################################################################


### callback to get image and apply contour
def image_contour_callback(img_msg):
  #Convert image from ros to cv2
  bridge = CvBridge()
  cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
  # Get contours
  cv_image.setflags(write=1)
  cv_image_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
  ret, thresh2 = cv2.threshold(cv_image_gray, 150, 255, cv2.THRESH_BINARY)
  contours3, hierarchy3 = cv2.findContours(thresh2, cv2.RETR_LIST, 
                                       cv2.CHAIN_APPROX_NONE)

  # Add contours as overlay
  cv2.drawContours(cv_image, contours3, -1, (0, 255, 0), 2, 
                 cv2.LINE_AA)
  # Add text overlay
  font                   = cv2.FONT_HERSHEY_SIMPLEX
  bottomLeftCornerOfText = (10,10)
  fontScale              = 0.5
  fontColor              = (0, 255, 0)
  thickness              = 1
  lineType               = 1

  cv2.putText(cv_image,'Image with Contours', 
    bottomLeftCornerOfText, 
    font, 
    fontScale,
    fontColor,
    thickness,
    lineType)
  
  #Convert image from cv2 to ros
  img_out_msg = bridge.cv2_to_imgmsg(cv_image,"bgr8")#desired_encoding='passthrough')
  
  # Publish new image to ros
  contour_image_pub.publish(img_out_msg) # You can view the enhanced_2D_image topic at //192.168.179.103:9091/ in a connected web browser

### Cleanup processes on node shutdown
def cleanup_actions():
    print("Shutting down: Executing script cleanup actions")


### Script Entrypoint
def startNode():
  rospy.loginfo("Starting Image Enhance automation script", disable_signals=True) # Disable signals so we can force a shutdown
  rospy.init_node
  rospy.init_node(name="image_enhance_auto_script")

  # Camera initialization
  rospy.loginfo("Initializing " + CAMERA_NAMESPACE + " to LOW resolution")
  time.sleep(1)

  # Start Image Enhance Process and Pubslisher
  rospy.Subscriber(IMAGE_INPUT_TOPIC, Image, image_contour_callback)

  rospy.on_shutdown(cleanup_actions)
  
  # Spin forever (until object is detected)
  rospy.spin()


#####################################################################################
# Main
#####################################################################################

if __name__ == '__main__':
  startNode()

