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


#LOW_RES_VALUE = 0
#MED_RES_VALUE = 1
#HIGH_RES_VALUE = 2
#ULTRA_RES_VALUE = 3

###!!!!!!!! Set Enhancment topics and parameters !!!!!!!!
ENHANCED_IMAGE_TOPIC = CAMERA_NAMESPACE + "idx/enhanced_2d_image"
ENHANCE_SENSITIVITY_RATIO=0.5

#####################################################################################
# Globals
#####################################################################################

ehnanced_image_pub = rospy.Publisher(ENHANCED_IMAGE_TOPIC, Image, queue_size=10)


#####################################################################################
# Methods
#####################################################################################


### callback to get image and apply enhancement algorithm
def image_enhancement_callback(img_msg):
  bridge = CvBridge()
  cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")#desired_encoding='passthrough')
  cv_image.setflags(write=1)
  # Color Correction optimization
  Max=[0,0,0]
  for k in range(0, 3):
    Max[k] = np.max(cv_image[:,:,k])
  Min_Max_channel  = np.min(Max)
  for k in range(0, 3):
    Max_channel  = np.max(cv_image[:,:,k])
    Min_channel  = np.min(cv_image[:,:,k])
    Mean_channel = np.mean(cv_image[:,:,k])
    Chan_scale = (255 - Mean_channel) / (Max_channel - Min_channel)
    if Chan_scale < 1:
      Chan_scale = 1 - (1-Chan_scale)*(255-Min_Max_channel)/170
    elif Chan_scale > 1:
      Chan_scale = 1 + (Chan_scale-1)*(255-Min_Max_channel)/170
    if Chan_scale > 1*(1+ENHANCE_SENSITIVITY_RATIO):
      Chan_scale = 1 *(1+ENHANCE_SENSITIVITY_RATIO)
    if Chan_scale < -1*(1+ENHANCE_SENSITIVITY_RATIO):
      Chan_scale = -1 *(1+ENHANCE_SENSITIVITY_RATIO)
    Chan_offset = -1*Min_channel
    if Chan_offset < -10 * (1+9*ENHANCE_SENSITIVITY_RATIO):
      Chan_offset = -10 * (1+9*ENHANCE_SENSITIVITY_RATIO)
      
  cv_image[:,:,k] = (cv_image[:,:,k] + Chan_offset) * Chan_scale

  # Contrast and Brightness optimization
  gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

  # Calculate grayscale histogram
  hist = cv2.calcHist([gray],[0],None,[256],[0,256])
  hist_size = len(hist)
  
  # Calculate cumulative distribution from the histogram
  accumulator = []
  accumulator.append(float(hist[0]))
  for index in range(1, hist_size):
    accumulator.append(accumulator[index -1] + float(hist[index]))
  
  # Locate points to clip
  maximum = accumulator[-1]
  clip_hist_percent = (maximum/100.0)
  clip_hist_percent /= 2.0
  
  # Locate left cut
  minimum_gray = 0
  while accumulator[minimum_gray] < clip_hist_percent:
    minimum_gray += 5
  
  # Locate right cut
  maximum_gray = hist_size -1
  while accumulator[maximum_gray] >= (maximum - clip_hist_percent):
    maximum_gray -= 1
  
  # Calculate alpha and beta values
  alpha = 255 / (maximum_gray - minimum_gray) * (0.5+ENHANCE_SENSITIVITY_RATIO)
  if alpha>2: ##
    alpha=2 ##
  beta = (-minimum_gray * alpha + 10) * (0.5+ENHANCE_SENSITIVITY_RATIO)
  if beta<-50: ##
    beta=-50 ##
  cv_image = cv2.convertScaleAbs(cv_image, alpha=alpha, beta=beta)
  img_enhc_msg = bridge.cv2_to_imgmsg(cv_image,"bgr8")#desired_encoding='passthrough')
  
  # Publish Enahaced Image Topic
  ehnanced_image_pub.publish(img_enhc_msg) # You can view the enhanced_2D_image topic at //192.168.179.103:9091/ in a connected web browser

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
  rospy.Subscriber(IMAGE_INPUT_TOPIC, Image, image_enhancement_callback)

  rospy.on_shutdown(cleanup_actions)
  
  # Spin forever (until object is detected)
  rospy.spin()


#####################################################################################
# Main
#####################################################################################

if __name__ == '__main__':
  startNode()

