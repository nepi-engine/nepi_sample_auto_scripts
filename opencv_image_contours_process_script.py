#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#


# Sample NEPI Process Script.
# 1. Add contrours and text overlay to image and republish to a new topic
# 2. Run until Stopped


import time
import sys
import rospy
import numpy as np
import cv2
from nepi_edge_sdk_base import nepi_ros 
from nepi_edge_sdk_base import nepi_msg
from nepi_edge_sdk_base import nepi_img

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import UInt8, Empty, String, Bool

#########################################
# USER SETTINGS - Edit as Necessary 
#########################################

## Set ROS Image Topic Name to Use
IMAGE_INPUT_TOPIC_NAME = "color_2d_image"



#########################################
# Node Class
#########################################

class opencv_image_contours(object):

  #######################
  ### Node Initialization
  DEFAULT_NODE_NAME = "opencv_image_contours" # Can be overwitten by luanch command
  def __init__(self):
    #### APP NODE INIT SETUP ####
    nepi_ros.init_node(name= self.DEFAULT_NODE_NAME)
    self.node_name = nepi_ros.get_node_name()
    self.base_namespace = nepi_ros.get_base_namespace()
    nepi_msg.createMsgPublishers(self)
    nepi_msg.publishMsgInfo(self,"Starting Initialization Processes")
    ##############################
    ## Initialize Class Variables
    ## Define Class Namespaces
    IMAGE_OUTPUT_TOPIC = self.base_namespace + "image_contours"
    ## Define Class Services Calls
    ## Create Class Sevices    
    ## Create Class Publishers
    # A custom image topic
    self.contour_image_pub = rospy.Publisher(IMAGE_OUTPUT_TOPIC, Image, queue_size=10)
    ## Start Class Subscribers
    # Wait for topic
    nepi_msg.publishMsgInfo(self,"Waiting for topic: " + IMAGE_INPUT_TOPIC_NAME)
    image_topic = nepi_ros.wait_for_topic(IMAGE_INPUT_TOPIC_NAME)
    # Start image contours overlay process and pubslisher
    rospy.Subscriber(image_topic, Image, self.image_custom_callback, queue_size = 1)
    ## Start Node Processes

    ##############################
    ## Initiation Complete
    nepi_msg.publishMsgInfo(self," Initialization Complete")
    # Spin forever (until object is detected)
    rospy.spin()
    ##############################

  #######################
  ### Node Methods

  ### Add your CV2 image customization code here
  def image_custom_callback(self,ros_img_msg):
    #Convert image from ros to cv2
    cv2_image = nepi_img.rosimg_to_cv2img(ros_img_msg)

    # Get and overlay image contours
    [contours, hierarchy] = nepi_img.get_contours(cv2_image)
    cv2_image = nepi_img.overlay_contours(cv2_image,contours)
 
    #Convert image from cv2 to ros
    ros_img_out_msg = nepi_img.cv2img_to_rosimg(cv2_image, encoding = "bgr8")
    # Publish new image to ros
    if not rospy.is_shutdown():
      self.contour_image_pub.publish(ros_img_out_msg) # You can view the enhanced_2D_image

  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    nepi_msg.publishMsgInfo(self,"Shutting down: Executing script cleanup actions")


#########################################
# Main
#########################################
if __name__ == '__main__':
  opencv_image_contours()

