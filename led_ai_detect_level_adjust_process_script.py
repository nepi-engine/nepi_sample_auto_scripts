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

from std_msgs.msg import Empty, Float32
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBoxes, ObjectCount


#########################################
# USER SETTINGS - Edit as Necessary 
#########################################

OBJ_LABEL_OF_INTEREST = "person"
MAX_LED_LEVEL = 0.1
WD_TIMEOUT_SEC = 4
AVG_LENGTH = 5

#Set LED Control ROS Topic Name (or partial name)
LED_CONTROL_TOPIC_NAME = "lsx/set_intensity"

#########################################
# ROS NAMESPACE SETUP
#########################################

# ROS namespace setup
NEPI_BASE_NAMESPACE = "/nepi/s2x/"

# AI Detector Subscriber Topics
AI_BOUNDING_BOXES_TOPIC = NEPI_BASE_NAMESPACE + "classifier/bounding_boxes"
AI_DETECTION_IMAGE_TOPIC = NEPI_BASE_NAMESPACE + "classifier/detection_image"
AI_FOUND_OBJECT_TOPIC = NEPI_BASE_NAMESPACE + "classifier/found_object"

# Snapshot Publish Topic


#########################################
# Globals
#########################################

led_intensity_pub = None
img_width = 0 # Updated on receipt of first image
img_height = 0 # Updated on receipt of first image
object_detected = False
intensity_last = 0
intensity_history = np.zeros(AVG_LENGTH)


#########################################
# Methods
#########################################

### System Initialization processes
def initialize_actions():
  global img_height
  global img_width
  global led_intensity_pub
  print("")
  print("Starting Initialization Processes")
  # Wait for topic by name
  print("Waiting for topic name: " + LED_CONTROL_TOPIC_NAME)
  led_control_topic=find_topic(LED_CONTROL_TOPIC_NAME)
  print("Found topic: " + led_control_topic)
  led_intensity_pub = rospy.Publisher(led_control_topic, Float32, queue_size = 1)
  # Wait for topic by name
  print("Connecting to NEPI Detector Image Topic")
  print(AI_DETECTION_IMAGE_TOPIC )
  # Wait for topic
  print("Waiting for topic: " + AI_DETECTION_IMAGE_TOPIC)
  wait_for_topic(AI_DETECTION_IMAGE_TOPIC)
  img_sub = rospy.Subscriber(AI_DETECTION_IMAGE_TOPIC, Image, image_callback)
  while img_width == 0 and img_height == 0:
    print("Waiting for Classifier Detection Image")
    time.sleep(1)
  img_sub.unregister() # Don't need it anymore
  # Set up object detector subscriber
  print("Starting object detection subscriber: Object of interest = " + OBJ_LABEL_OF_INTEREST + "...")
  rospy.Subscriber(AI_BOUNDING_BOXES_TOPIC, BoundingBoxes, object_detected_callback, queue_size = 1)
  #Set up found object subscriber which monitors all AI outputs
  print("Starting found object subscriber")
  rospy.Subscriber(AI_FOUND_OBJECT_TOPIC, ObjectCount, found_object_callback, queue_size = 1)
  print("Setting up watchdog timer")
  rospy.Timer(rospy.Duration(1), watchdog_timer_callback)
  print("Initialization Complete")
 
### Simple callback to get image height and width
def image_callback(img_msg):
  # This is just to get the image size for ratio purposes
  global img_height
  global img_width
  if (img_height == 0 and img_width == 0):
    print("Initial input image received. Size = " + str(img_msg.width) + "x" + str(img_msg.height))
    img_height = img_msg.height
    img_width = img_msg.width

# Action upon detection of object of interest
def object_detected_callback(bounding_box_msg):
  global img_height
  global img_width
  global led_intensity_pub
  global intensity_last
  global intensity_history
  global object_detected
  global wd_timer
  wd_timer = 0
  object_detected = False
  # Iterate over all of the objects reported by the detector
  for box in bounding_box_msg.bounding_boxes:
    # Check for the object of interest and take appropriate actions
    if box.Class == OBJ_LABEL_OF_INTEREST:
      box_of_interest=box
      print(box_of_interest.Class)
      # Calculate the box center in image ratio terms
      object_loc_y_pix = box_of_interest.ymin + ((box_of_interest.ymax - box_of_interest.ymin)  / 2) 
      object_loc_x_pix = box_of_interest.xmin + ((box_of_interest.xmax - box_of_interest.xmin)  / 2)
      object_loc_y_ratio = float(object_loc_y_pix) / img_height
      object_loc_x_ratio = float(object_loc_x_pix) / img_width
      print("Object Detected " + OBJ_LABEL_OF_INTEREST + " with box center (" + str(object_loc_x_ratio) + ", " + str(object_loc_y_ratio) + ")")
      # check if we are AIose enough to center in either dimension to stop motion: Hysteresis band
      box_abs_error_x_ratio = 1-2.0 * abs(object_loc_x_ratio - 0.5)
      box_abs_error_y_ratio = 1-2.0 * abs(object_loc_y_ratio - 0.5)
      print("Object Detection Error Ratios Horz: " "%.2f" % (box_abs_error_x_ratio) + " Vert: " + "%.2f" % (box_abs_error_y_ratio))
      # Sending LED level update
      error_ratios = [box_abs_error_x_ratio] # ignore vertical
      mean_error_ratio = statistics.mean(error_ratios)
      print("Target center ratio: " + "%.2f" % (mean_error_ratio))
      intensity = MAX_LED_LEVEL *  mean_error_ratio*4
      intensity_last = intensity
      intensity_history = np.roll(intensity_history,1)
      intensity_history[0]=intensity
      avg_inensity = np.mean(intensity_history)
      print("Setting intensity level to: " + "%.2f" % (avg_inensity))
      object_detected = True
      if not rospy.is_shutdown():
        led_intensity_pub.publish(data = avg_inensity)
    else:
      #led_intensity_pub.publish(data = 0.0)
      #print("No " + OBJ_LABEL_OF_INTEREST + " type for target data")
      time.sleep(0.01)
    if object_detected == False and not rospy.is_shutdown():
      led_intensity_pub.publish(data = 0)

def found_object_callback(found_obj_msg):
  # Must reset object_detected in the event of no objects to restart scan mode
  global object_detected
  global watchdog_timer
  wd_timer = 0
  if found_obj_msg.count == 0:
    print("No objects found")
    object_detected=False
    if not rospy.is_shutdown():
      led_intensity_pub.publish(data = 0)

### Setup a regular background scan process based on timer callback
def watchdog_timer_callback(timer):
  # Called periodically no matter what as a Timer object callback
  global wd_timer
  print("Watchdog timer: " + str(wd_timer))
  if wd_timer > WD_TIMEOUT_SEC:
    print("Past timeout time, turning lights off")
    if not rospy.is_shutdown():
      led_intensity_pub.publish(data = 0)
  else:
    wd_timer = wd_timer + 1

#######################
# Initialization Functions

### Function to find a topic
def find_topic(topic_name):
  topic = ""
  topic_list=rospy.get_published_topics(namespace='/')
  #print(topic_list)
  for topic_entry in topic_list:
    #print(topic_entry[0])
    if topic_entry[0].find(topic_name) != -1:
      topic = topic_entry[0]
  return topic

### Function to check for a topic 
def wait_for_topic(topic_name):
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

def cleanup_actions():
  global led_intensity_pub
  print("Shutting down: Executing script cleanup actions")
  led_intensity_pub.publish(data = 0)


### Script Entrypoint
def startNode():
  rospy.loginfo("Starting AI Detect and Snapshot Process Script", disable_signals=True) # Disable signals so we can force a shutdown
  rospy.init_node
  rospy.init_node(name="ai_detect_and_snapshot_process_script")
  # Run Initialization processes
  initialize_actions()
  #Set up Anode shutdown
  rospy.on_shutdown(cleanup_actions)
  # Spin forever (until object is detected)
  rospy.spin()


#########################################
# Main
#########################################

if __name__ == '__main__':
  startNode()

