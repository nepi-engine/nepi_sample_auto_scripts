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


# Sample NEPI Automation Script. 
# Uses onboard ROS python library to
# 1. Add contrours and text overlay to image and republish to a new topic
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

## Set Image ROS Topic Name to Use
IMAGE_INPUT_TOPIC_NAME = "color_2d_image"

# ROS namespace setup
NEPI_BASE_NAMESPACE = "/nepi/s2x/"

IMAGE_OUTPUT_TOPIC = NEPI_BASE_NAMESPACE + "image_contours"

#####################################################################################
# Globals
#####################################################################################
contour_image_pub = rospy.Publisher(IMAGE_OUTPUT_TOPIC, Image, queue_size=10)

#####################################################################################
# Methods
#####################################################################################

### System Initialization processes
def initialize_actions():
  print("")
  print("Starting Initialization")  
  # Wait for topic
  print("Waiting for topic: " + IMAGE_INPUT_TOPIC_NAME)
  image_topic = wait_for_topic(IMAGE_INPUT_TOPIC_NAME)
  # Start image contours overlay process and pubslisher
  rospy.Subscriber(image_topic, Image, image_contour_callback, queue_size = 1)
  print("Initialization Complete")


### callback to get image, apply contour, and publish new image on new topic
def image_contour_callback(img_msg):
  global contour_image_pub
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
  if not rospy.is_shutdown():
    contour_image_pub.publish(img_out_msg) # You can view the enhanced_2D_image topic at //192.168.179.103:9091/ in a connected web browser


#######################
# Initialization Functions

### Function to find a topic
def find_topic(topic_name):
  topic = ""
  topic_list=rospy.get_published_topics(namespace='/')
  for topic_entry in topic_list:
    if topic_entry[0].find(topic_name) != -1:
      topic = topic_entry[0]
  return topic

### Function to check for a topic 
def check_for_topic(topic_name):
  topic_exists = True
  topic=find_topic(topic_name)
  if topic == "":
    topic_exists = False
  return topic_exists

### Function to wait for a topic
def wait_for_topic(topic_name):
  topic = ""
  while topic == "" and not rospy.is_shutdown():
    topic=find_topic(topic_name)
    time.sleep(.1)
  return topic

#######################
# StartNode and Cleanup Functions

### Cleanup processes on node shutdown
def cleanup_actions():
  global contour_image_pub
  print("Shutting down: Executing script cleanup actions")
  # Unregister publishing topics
  custom_image_pub.unregister()


### Script Entrypoint
def startNode():
  rospy.loginfo("Starting OpenCV Image Contours Process Script", disable_signals=True) # Disable signals so we can force a shutdown
  rospy.init_node
  rospy.init_node(name="opencv_image_your_custom_process_script")
  # Run Initialization processes
  initialize_actions()
  #Set up cleanup on node shutdown
  rospy.on_shutdown(cleanup_actions)
  # Spin forever (until object is detected)
  rospy.spin()


#####################################################################################
# Main
#####################################################################################

if __name__ == '__main__':
  startNode()

