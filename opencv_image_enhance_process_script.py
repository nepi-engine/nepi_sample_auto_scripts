#!/usr/bin/env python
#
# NEPI Dual-Use License
# Project: nepi_sample_auto_scripts
#
# This license applies to any user of NEPI Engine software
#
# Copyright (C) 2023 Numurus, LLC <https://www.numurus.com>
# see https://github.com/numurus-nepi/nepi_edge_sdk_base
#
# This software is dual-licensed under the terms of either a NEPI software developer license
# or a NEPI software commercial license.
#
# The terms of both the NEPI software developer and commercial licenses
# can be found at: www.numurus.com/licensing-nepi-engine
#
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - https://www.numurus.com/licensing-nepi-engine
# - mailto:nepi@numurus.com
#
#


# Sample NEPI Process Script.
# 1. run sensor imagery through an image enhancement algorithm and republish to a new topic
# 2. Run until Stopped

import time
import sys
import rospy
import numpy as np
import cv2
from resources import nepi

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import UInt8, Empty, String, Bool

#########################################
# USER SETTINGS - Edit as Necessary 
#########################################

ENHANCE_SENSITIVITY_RATIO = 0.5

## Set ROS Image Topic Name to Use
IMAGE_INPUT_TOPIC_NAME = "color_2d_image"


#########################################
# ROS NAMESPACE SETUP
#########################################

# ROS namespace setup
NEPI_BASE_NAMESPACE = "/nepi/s2x/"

#########################################
# Node Class
#########################################

class opencv_image_enhance_process(object):

  #######################
  ### Node Initialization
  def __init__(self):
    rospy.loginfo("Starting Initialization Processes")
    ## Initialize Class Variables
    ## Define Class Namespaces
    IMAGE_OUTPUT_TOPIC = NEPI_BASE_NAMESPACE + "image_enahnaced"
    ## Define Class Services Calls
    ## Create Class Sevices    
    ## Create Class Publishers
    # A custom image topic
    self.enhanced_image_pub = rospy.Publisher(IMAGE_OUTPUT_TOPIC, Image, queue_size=10)
    ## Start Class Subscribers
    # Wait for topic
    rospy.loginfo("Waiting for topic: " + IMAGE_INPUT_TOPIC_NAME)
    image_topic = nepi.wait_for_topic(IMAGE_INPUT_TOPIC_NAME)
    # Start image contours overlay process and pubslisher
    rospy.Subscriber(image_topic, Image, self.image_custom_callback, queue_size = 1)
    ## Start Node Processes
    ## Initiation Complete
    rospy.loginfo("Initialization Complete")

  #######################
  ### Node Methods

  ### Add your CV2 image customization code here
  def image_custom_callback(self,img_msg):
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
      self.enhanced_image_pub.publish(img_enhc_msg) # You can view the enhanced_2D_image topic at //192.168.179.103:9091/ in a connected web browser


  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    rospy.loginfo("Shutting down: Executing script cleanup actions")


#########################################
# Main
#########################################
if __name__ == '__main__':
  current_filename = sys.argv[0].split('/')[-1]
  current_filename = current_filename.split('.')[0]
  rospy.loginfo(("Starting " + current_filename), disable_signals=True) # Disable signals so we can force a shutdown
  rospy.init_node(name=current_filename)
  #Launch the node
  node_name = current_filename.rpartition("_")[0]
  rospy.loginfo("Launching node named: " + node_name)
  node_class = eval(node_name)
  node = node_class()
  #Set up node shutdown
  rospy.on_shutdown(node.cleanup_actions)
  # Spin forever (until object is detected)
  rospy.spin()


