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
# 1. Waits for ai detection topic
# 2. Adjust LED level based on target location in image

# Requires the following additional scripts are running
# a)ai_detector_config_script.py
# These scripts are available for download at:
# [link text](https://github.com/numurus-nepi/nepi_sample_auto_scripts)

import time
import sys
import rospy
import statistics
import numpy as np
from nepi_edge_sdk_base import nepi_ros 
from nepi_edge_sdk_base import nepi_msg

from std_msgs.msg import Empty, Float32
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBoxes, ObjectCount


#########################################
# USER SETTINGS - Edit as Necessary 
#########################################

OBJECT_LABEL_OF_INTEREST = "person"
LED_LEVEL_MAX = 0.3
WD_TIMEOUT_SEC = 4
AVG_LENGTH = 2

#Set LED Control ROS Topic Name (or partial name)
LED_CONTROL_TOPIC_NAME = "lsx/set_intensity"



#########################################
# Node Class
#########################################

class led_adjust_on_object_detect(object):

  #######################
  ### Node Initialization
  DEFAULT_NODE_NAME = "led_adjust_on_object_detect" # Can be overwitten by luanch command
  def __init__(self):
    #### APP NODE INIT SETUP ####
    nepi_ros.init_node(name= self.DEFAULT_NODE_NAME)
    self.node_name = nepi_ros.get_node_name()
    self.base_namespace = nepi_ros.get_base_namespace()
    nepi_msg.createMsgPublishers(self)
    nepi_msg.publishMsgInfo(self,"Starting Initialization Processes")
    ##############################
    ## Initialize Class Variables
    self.led_intensity_pub = None
    self.object_label_of_interest = OBJECT_LABEL_OF_INTEREST
    self.led_level_max = LED_LEVEL_MAX
    self.wd_timeout_sec = WD_TIMEOUT_SEC
    self.wd_check_interval_sec = .1
    self.wd_timer = 0
    self.intensity_history = np.zeros(AVG_LENGTH)
    self.img_width = 0 # Updated on receipt of first image
    self.img_height = 0 # Updated on receipt of first image
    self.object_detected = False
    ## Define Class Namespaces
    # AI Detector Subscriber Topics
    AI_BOUNDING_BOXES_TOPIC = self.base_namespace + "classifier/bounding_boxes"
    AI_DETECTION_IMAGE_TOPIC = self.base_namespace + "classifier/detection_image"
    AI_FOUND_OBJECT_TOPIC = self.base_namespace + "classifier/found_object"
    ## Class subscribers

    # Wait for AI detector image topic to publish
    nepi_msg.publishMsgInfo(self,"Connecting to NEPI Detector Image Topic")
    nepi_msg.publishMsgInfo(self,AI_DETECTION_IMAGE_TOPIC )
    nepi_msg.publishMsgInfo(self,"Waiting for topic: " + AI_DETECTION_IMAGE_TOPIC)
    nepi_ros.wait_for_topic(AI_DETECTION_IMAGE_TOPIC)
    img_sub = rospy.Subscriber(AI_DETECTION_IMAGE_TOPIC, Image, self.image_callback)
    while self.img_width == 0 and self.img_height == 0 and not rospy.is_shutdown():
      nepi_msg.publishMsgInfo(self,"Waiting for Classifier Detection Image")
      nepi_ros.sleep(1,100)
    img_sub.unregister() # Don't need it anymore
    ## Create Class Publishers
    led_control_topic_name = LED_CONTROL_TOPIC_NAME
    nepi_msg.publishMsgInfo(self,"Waiting for topic name: " + led_control_topic_name)
    led_control_topic=nepi_ros.wait_for_topic(led_control_topic_name)
    nepi_msg.publishMsgInfo(self,"Found topic: " + led_control_topic)
    self.led_intensity_pub = rospy.Publisher(led_control_topic, Float32, queue_size = 1)
    ## Start Class Subscribers
    # Set up object detector subscriber
    nepi_msg.publishMsgInfo(self,"Starting object detection subscriber: Object of interest = " + self.object_label_of_interest + "...")
    rospy.Subscriber(AI_BOUNDING_BOXES_TOPIC, BoundingBoxes, self.object_detected_callback, queue_size = 1)
    #Set up found object subscriber which monitors all AI outputs
    nepi_msg.publishMsgInfo(self,"Starting found object subscriber")
    rospy.Subscriber(AI_FOUND_OBJECT_TOPIC, ObjectCount, self.found_object_callback, queue_size = 1)
    ## Start Node Processes
    # Setup watchdog process
    nepi_msg.publishMsgInfo(self,"Setting up watchdog timer")
    rospy.Timer(rospy.Duration(self.wd_check_interval_sec), self.watchdog_timer_callback)

    ##############################
    ## Initiation Complete
    nepi_msg.publishMsgInfo(self," Initialization Complete")
    # Spin forever (until object is detected)
    rospy.spin()
    ##############################



  #######################
  ### Node Methods

  ### Simple callback to get image height and width
  def image_callback(self,img_msg):
    # This is just to get the image size for ratio purposes
    if (self.img_height == 0 and self.img_width == 0):
      nepi_msg.publishMsgInfo(self,"Initial input image received. Size = " + str(img_msg.width) + "x" + str(img_msg.height))
      self.img_height = img_msg.height
      self.img_width = img_msg.width

  # Action upon detection of object of interest
  def object_detected_callback(self,bounding_box_msg):
    self.object_detected = False
    # Iterate over all of the objects reported by the detector
    for box in bounding_box_msg.bounding_boxes:
      # Check for the object of interest and take appropriate actions
      if box.Class == self.object_label_of_interest:
        box_of_interest=box
        print(box_of_interest.Class)
        # Calculate the box center in image ratio terms
        object_loc_y_pix = box_of_interest.ymin + ((box_of_interest.ymax - box_of_interest.ymin)  / 2) 
        object_loc_x_pix = box_of_interest.xmin + ((box_of_interest.xmax - box_of_interest.xmin)  / 2)
        object_loc_y_ratio = float(object_loc_y_pix) / self.img_height
        object_loc_x_ratio = float(object_loc_x_pix) / self.img_width
        print("Object Detected " + self.object_label_of_interest + " with box center (" + str(object_loc_x_ratio) + ", " + str(object_loc_y_ratio) + ")")
        # check if we are AIose enough to center in either dimension to stop motion: Hysteresis band
        box_abs_error_x_ratio = 2.0 * abs(object_loc_x_ratio - 0.5)
        box_abs_error_y_ratio = 2.0 * abs(object_loc_y_ratio - 0.5)
        print("Object Detection Error Ratios Horz: " "%.2f" % (box_abs_error_x_ratio) + " Vert: " + "%.2f" % (box_abs_error_y_ratio))
        # Sending LED level update
        center_ratios = [1-box_abs_error_x_ratio] # ignore vertical
        mean_center_ratio = statistics.mean(center_ratios)
        print("Target center ratio: " + "%.2f" % (mean_center_ratio))
        intensity = self.led_level_max *  mean_center_ratio**4
        self.intensity_history = np.roll(self.intensity_history,1)
        self.intensity_history[0]=intensity
        avg_intensity = np.mean(self.intensity_history)
        print("Setting intensity level to: " + "%.2f" % (avg_intensity))
        self.object_detected = True
        if not rospy.is_shutdown():
          self.led_intensity_pub.publish(data = avg_intensity)
      else:
        #led_intensity_pub.publish(data = 0.0)
        #print("No " + object_label_OF_INTEREST + " type for target data")
        time.sleep(0.01)
      if self.object_detected == False and not rospy.is_shutdown():
        self.led_intensity_pub.publish(data = 0)
    
  ### Check the number of objects detected on last detection process
  def found_object_callback(self,found_obj_msg):
    self.wd_timer = 0
    if found_obj_msg.count == 0:
      nepi_msg.publishMsgInfo(self,"No objects found")
      self.object_detected=False
      if not rospy.is_shutdown():
        self.led_intensity_pub.publish(data = 0)


  ### Setup a regular background scan process based on timer callback
  def watchdog_timer_callback(self,timer):
    # Called periodically no matter what as a Timer object callback
    nepi_msg.publishMsgInfo(self,"Watchdog timer: " + str(self.wd_timer))
    if self.wd_timer > self.wd_timeout_sec:
      nepi_msg.publishMsgInfo(self,"Past timeout time, turning lights off")
      if not rospy.is_shutdown():
        self.led_intensity_pub.publish(data = 0)
    else:
      self.wd_timer = self.wd_timer + self.wd_check_interval_sec



  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    global led_intensity_pub
    nepi_msg.publishMsgInfo(self,"Shutting down: Executing script cleanup actions")
    self.led_intensity_pub.publish(data = 0)



#########################################
# Main
#########################################
if __name__ == '__main__':
  led_adjust_on_object_detect()


