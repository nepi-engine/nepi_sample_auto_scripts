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

CAM_RES=1 # Number 0-3, 0 Low, 1 Med, 2 High, 3 Ultra
ENHANCE_SENSITIVITY_RATIO=0.5

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
IMAGE_OUTPUT_TOPIC = CAMERA_NAMESPACE + "idx/enhanced_2d_image"

#####################################################################################
# Globals
#####################################################################################
res_adj_pub = rospy.Publisher(RESOLUTION_ADJ_TOPIC, UInt8, queue_size=10)
enhanced_image_pub = rospy.Publisher(IMAGE_OUTPUT_TOPIC, Image, queue_size=10)

#####################################################################################
# Methods
#####################################################################################

### System Initialization processes
def initialize_actions():
  print("")
  rospy.loginfo("Initializing " + CAMERA_NAMESPACE )
  rospy.loginfo("Connecting to ROS Topic " + IMAGE_INPUT_TOPIC )
  # Check if camera topic is publishing
  topic_list=rospy.get_published_topics(namespace='/')
  topic_to_connect=[IMAGE_INPUT_TOPIC, 'sensor_msgs/Image']
  if topic_to_connect in topic_list: 
    print("Camera topic found, starting initializing process")
    res_adj_pub.publish(CAM_RES)
    time.sleep(1)
    print("Initialization Complete")
  else: 
    print("!!!!! Camera topic not found, shutting down")
    time.sleep(1)
    rospy.signal_shutdown("Camera topic not found")


### callback to get image, enahance image, and publish new image on new topic
def image_enhance_callback(img_msg):
  global ehnanced_image_pub
  #Convert image from ros to cv2
  bridge = CvBridge()
  cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
  # Get contours
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
  if not rospy.is_shutdown():
    enhanced_image_pub.publish(img_enhc_msg) # You can view the enhanced_2D_image topic at //192.168.179.103:9091/ in a connected web browser


### Cleanup processes on node shutdown
def cleanup_actions():
  global enhanced_image_pub
  print("Shutting down: Executing script cleanup actions")
  # Unregister publishing topics
  enhanced_image_pub.unregister()
  time.sleep(2)

### Script Entrypoint
def startNode():
  rospy.loginfo("Starting Image Enhance automation script", disable_signals=True) # Disable signals so we can force a shutdown
  rospy.init_node
  rospy.init_node(name="image_enhance_auto_script")
  # Run Initialization processes
  initialize_actions()
  # Start image enhance process and pubslisher
  rospy.Subscriber(IMAGE_INPUT_TOPIC, Image, image_enhance_callback, queue_size = 1)
  #Set up cleanup on node shutdown
  rospy.on_shutdown(cleanup_actions)
  # Spin forever (until object is detected)
  rospy.spin()


#####################################################################################
# Main
#####################################################################################

if __name__ == '__main__':
  startNode()

