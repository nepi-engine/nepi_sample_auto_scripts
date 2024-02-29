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
# 1. Connects to camera image
# 2. Estimates image brightness with OpenCV 
# 3. Adjust LED brightness based on image brightness


import time
import sys
import rospy
import numpy as np
from numpy.linalg import norm
import cv2
from resources import nepi

from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import UInt8, Empty, String, Bool

#########################################
# USER SETTINGS - Edit as Necessary 
#########################################

LED_LEVEL_MAX = 0.3 # Ratio 0-1
SENSITIVITY_RATIO = 1.0
AVG_LENGTH = 40

## Set ROS Image Topic Name to Use
IMAGE_INPUT_TOPIC_NAME = "color_2d_image"
#IMAGE_INPUT_TOPIC_NAME = "bw_2d_image"

#Set LED Control ROS Topic Name (or partial name)
LED_CONTROL_TOPIC_NAME = "lsx/set_intensity"


#########################################
# ROS NAMESPACE SETUP
#########################################

# ROS namespace setup
NEPI_BASE_NAMESPACE = "/nepi/s2x/"

#########################################
# Node Class
#########################################

class led_auto_level_process(object):

  #######################
  ### Node Initialization
  def __init__(self):
    rospy.loginfo("Starting Initialization Processes")
    ## Initialize Class Variables
    self.led_level_max = LED_LEVEL_MAX
    self.intensity_history = np.zeros(AVG_LENGTH)
    self.avg_intensity = 0
    self.img_brightness_ratio =0
    ## Define Class Namespaces
    IMAGE_OUTPUT_TOPIC = NEPI_BASE_NAMESPACE + "image_custom"
    ## Define Class Services Calls
    ## Create Class Sevices    
    ## Create Class Publishers
    ## Create Class Publishers
    led_control_topic_name = LED_CONTROL_TOPIC_NAME
    rospy.loginfo("Waiting for topic name: " + led_control_topic_name)
    led_control_topic=nepi.wait_for_topic(led_control_topic_name)

    rospy.loginfo("Found topic: " + led_control_topic)
    self.led_intensity_pub = rospy.Publisher(led_control_topic, Float32, queue_size = 1)
    ## Start Class Subscribers
    # Wait for topic
    rospy.loginfo("Waiting for topic: " + IMAGE_INPUT_TOPIC_NAME)
    image_topic = nepi.wait_for_topic(IMAGE_INPUT_TOPIC_NAME)
    # Start image contours overlay process and pubslisher
    rospy.Subscriber(image_topic, Image, self.image_brightness_callback, queue_size = 1)
    # Start regular print callback
    rospy.Timer(rospy.Duration(1), self.lxs_print_callback)
    ## Start Node Processes
    ## Initiation Complete
    rospy.loginfo("Initialization Complete")

  #######################
  ### Node Methods

  def lxs_print_callback(self,timer):
    #print(self.intensity_history)
    print("Image brightness estimated at: " + "%.2f" % (self.img_brightness_ratio))
    print("Intensity level set to: " + "%.2f" % (self.avg_intensity))

  ### Add your CV2 image customization code here
  def image_brightness_callback(self,img_msg):
    #Convert image from ros to cv2
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    # Get brightness estimate
    self.img_brightness_ratio=self.brightness_ratio(cv_image)
    #Adjust LED level
    intensity = self.led_level_max *  (1-self.img_brightness_ratio)
    self.intensity_history = np.roll(self.intensity_history,1)
    self.intensity_history[0]=intensity
    self.avg_intensity = np.mean(self.intensity_history)
    
    self.object_detected = True
    if not rospy.is_shutdown():
      self.led_intensity_pub.publish(data = self.avg_intensity)

  ### image brightness estimator
  def brightness_ratio(self,img):
      if len(img.shape) == 3:
          # Colored RGB or BGR (*Do Not* use HSV images with this function)
          # create brightness with euclidean norm
          b_ratio = np.average(norm(img, axis=2)) / np.sqrt(3) / 255 * 2 * SENSITIVITY_RATIO
      else:
          # Grayscale
          return np.average(img) / 255 * 2 * SENSITIVITY_RATIO
      if b_ratio > 1:
        b_ratio = 1
      elif b_ratio < 0:
        b_ratio = 0
      return b_ratio


  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    rospy.loginfo("Shutting down: Executing script cleanup actions")
    self.led_intensity_pub.publish(data = 0)

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






