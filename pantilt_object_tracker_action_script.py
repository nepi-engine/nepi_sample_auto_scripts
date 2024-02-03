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

# Sample NEPI Action Script. 
### Expects Classifier to be running ###
# 1. Configures and starts pan and tilt
# 2. Sets and start pan and tilt search scan
# 3. Waits for specific objects to be detected and starts tracking largest detection box
# 4. Returns to search/scan mode if no detected objects are being detected

# Requires the following additional scripts are running
# a)ai_detector_config_script.py
# These scripts are available for download at:
# [link text](https://github.com/numurus-nepi/nepi_sample_auto_scripts)

import time
import sys
import rospy
import numpy as np
import cv2
##import subprocess
##import os

from std_msgs.msg import UInt8, Empty, Float32
from sensor_msgs.msg import Image
from nepi_ros_interfaces.msg import PanTiltLimits, PanTiltPosition, PanTiltStatus, StringArray
from darknet_ros_msgs.msg import BoundingBoxes, ObjectCount


#########################################
# USER SETTINGS - Edit as Necessary 
#########################################

# Minimum detection box area as a ration of image size
MIN_DETECT_BOX_AREA_RATIO = 0.15 # Filters background targets.
OBJ_LABEL_OF_INTEREST = "person"

# Pan and Tilt setup parameters
PTX_REVERSE_PAN = False # Relative to image pixel location reporting
PTX_REVERSE_TILT = True # Relative to image pixel location reporting
PTX_SCAN_PAN_LIMITS = 40 # +- Pan Angle Limits
PTX_SCAN_TILT_RATIO = 0.55 # Tilt Angle During Scanning

# Parameters for pan and tilt tracking settings
PTX_SCAN_CHECK_INTERVAL = 0.25
PTX_SCAN_SPEED = 0.6
PTX_MAX_TRACK_SPEED = 1.0
PTX_MIN_TRACK_SPEED = 0.05
PTX_OBJECT_TILT_OFFSET_RATIO = 0.7 # Adjust tilt tracking ratio location within box
PTX_OBJ_CENTERED_BUFFER_RATIO = 0.3 # Hysteresis band about center of image for tracking purposes

#########################################
# ROS NAMESPACE SETUP
#########################################

NEPI_BASE_NAMESPACE = "/nepi/s2x/"
PTX_NAMESPACE = NEPI_BASE_NAMESPACE + "iqr_pan_tilt/ptx/"

# PanTilt Subscribe Topics
PTX_GET_STATUS_TOPIC = PTX_NAMESPACE + "status"
# PanTilt Control Publish Topics
PTX_SET_SPEED_RATIO_TOPIC = PTX_NAMESPACE + "set_speed_ratio"
PTX_GOHOME_TOPIC = PTX_NAMESPACE + "go_home"
PTX_STOP_TOPIC = PTX_NAMESPACE + "stop_moving"
PTX_GOTO_PAN_RATIO_TOPIC = PTX_NAMESPACE + "jog_to_yaw_ratio"
PTX_GOTO_TILT_RATIO_TOPIC = PTX_NAMESPACE + "jog_to_pitch_ratio"
PTX_SET_SOFT_LIMITS_TOPIC = PTX_NAMESPACE + "set_soft_limits"

# Classifier topics and parameters
AI_BOUNDING_BOXES_TOPIC = NEPI_BASE_NAMESPACE + "classifier/bounding_boxes"
AI_FOUND_OBJECT_TOPIC = NEPI_BASE_NAMESPACE + "classifier/found_object"
AI_DETECTION_IMAGE_TOPIC = NEPI_BASE_NAMESPACE + "classifier/detection_image"


#########################################
# Globals
#########################################

send_pt_home_pub = rospy.Publisher(PTX_GOHOME_TOPIC, Empty, queue_size=10)
set_pt_speed_ratio_pub = rospy.Publisher(PTX_SET_SPEED_RATIO_TOPIC, Float32, queue_size=10)
set_pt_pan_ratio_pub = rospy.Publisher(PTX_GOTO_PAN_RATIO_TOPIC, Float32, queue_size=10)
set_pt_tilt_ratio_pub = rospy.Publisher(PTX_GOTO_TILT_RATIO_TOPIC, Float32, queue_size=10)
set_pt_soft_limits_pub = rospy.Publisher(PTX_SET_SOFT_LIMITS_TOPIC, PanTiltLimits, queue_size=10)

img_width = 0 # Updated on receipt of first image
img_height = 0 # Updated on receipt of first image
img_area = 0 # Updated on receipt of first image

pt_stop_motion_pub = rospy.Publisher(PTX_STOP_TOPIC, Empty, queue_size=10)
pan_scan_direction = 1 # Keep track of current scan direction (1: Positive Limit, -1: Negative Limit)
img_width = 0 # Updated on receipt of first image
img_height = 0 # Updated on receipt of first image
pt_forward_pan_limit_ratio = 1.0 if PTX_REVERSE_PAN is False else 0.0
pt_backward_pan_limit_ratio = 1.0 - pt_forward_pan_limit_ratio
pt_forward_tilt_limit_ratio = 1.0 if PTX_REVERSE_TILT is False else 0.0
pt_backward_tilt_limit_ratio = 1.0 - pt_forward_tilt_limit_ratio
pt_tilt_scan_ratio = PTX_SCAN_TILT_RATIO if PTX_REVERSE_TILT is False else 1-PTX_SCAN_TILT_RATIO
object_detected=False
last_object_pan_ratio=0
pt_yaw_now_deg=0
pt_pitch_now_deg=0
pt_yaw_now_ratio=0
pt_pitch_now_ratio=0
pt_speed_now_ratio=0


#########################################
# Methods
#########################################

### System Initialization processes
def initialize_actions():
  global set_pt_speed_ratio_pub
  global set_pt_pan_ratio_pub
  global set_pt_tilt_ratio_pub
  global set_pt_soft_limits_pub
  global pt_tilt_scan_ratio
  print("")
  print("Starting Initialization")
  # Wait for AI detection topic
  print("Waiting for topic: " + AI_DETECTION_IMAGE_TOPIC)
  wait_for_topic(AI_DETECTION_IMAGE_TOPIC)
  img_sub = rospy.Subscriber(AI_DETECTION_IMAGE_TOPIC, Image, image_callback)
  while img_width == 0 and img_height == 0 and not rospy.is_shutdown():
    print("Waiting for Classifier Detection Image")
    time.sleep(1)
  img_sub.unregister() # Don't need it anymore
  # Start PT Status Callback
  print("Starting Pan Tilt Stutus callback")
  rospy.Subscriber(PTX_GET_STATUS_TOPIC, PanTiltStatus, pt_status_callback)  
  # Pan/Tilt initialization: Center both axes
  print("Setting pan/tilt to initial position and speed")
  print("Scan tilt ratio set to: " + "%.2f" % (pt_tilt_scan_ratio))
  set_pt_tilt_ratio_pub.publish(pt_tilt_scan_ratio)
  set_pt_pan_ratio_pub.publish(0.5)
  set_pt_speed_ratio_pub.publish(PTX_SCAN_SPEED)
  time.sleep(2) # Give time to center
  # Set up the timer that start scanning when no objects are detected
  print("Setting up pan/tilt scan check timer")
  rospy.Timer(rospy.Duration(PTX_SCAN_CHECK_INTERVAL), pt_scan_timer_callback)
  #Set up object detector subscriber which only updates on AI detection
  print("Starting object detection subscriber")
  rospy.Subscriber(AI_BOUNDING_BOXES_TOPIC, BoundingBoxes, objects_detected_callback, queue_size = 1)
  #Set up found object subscriber which monitors all AI outputs
  print("Starting found object subscriber")
  rospy.Subscriber(AI_FOUND_OBJECT_TOPIC, ObjectCount, found_object_callback, queue_size = 1)
  print("Initialization Complete")


### Simple callback to get image height and width
def image_callback(img_msg):
  # This is just to get the image size for ratio purposes
  global img_height
  global img_width
  global img_area
  if (img_height == 0 and img_width == 0):
    #print("Initial input image received. Size = " + str(img_msg.width) + "x" + str(img_msg.height))
    img_height = img_msg.height
    img_width = img_msg.width
    img_area = img_height*img_width



### Simple callback to get pt status info
def pt_status_callback(PanTiltStatus):
  # This is just to get the current pt positions
  global pt_yaw_now_deg
  global pt_pitch_now_deg
  global pt_yaw_now_ratio
  global pt_pitch_now_ratio
  global pt_speed_now_ratio
  pt_yaw_now_deg=PanTiltStatus.yaw_now_deg
  pt_pitch_now_deg=PanTiltStatus.pitch_now_deg
  yaw_limit_min=PanTiltStatus.yaw_min_softstop_deg
  yaw_limit_max=PanTiltStatus.yaw_max_softstop_deg
  pitch_limit_min=PanTiltStatus.pitch_min_softstop_deg
  pitch_limit_max =PanTiltStatus.pitch_max_softstop_deg
  pt_yaw_now_ratio=(pt_yaw_now_deg-yaw_limit_min)/(yaw_limit_max-yaw_limit_min)
  if PTX_REVERSE_PAN:
    pt_yaw_now_ratio=1-pt_yaw_now_ratio
  pt_pitch_now_ratio=(pt_pitch_now_deg-pitch_limit_min)/(pitch_limit_max-pitch_limit_min)
  if PTX_REVERSE_TILT:
    pt_pitch_now_ratio=1-pt_pitch_now_ratio
  pt_speed_now_ratio=PanTiltStatus.speed_ratio



### Setup a regular background scan process based on timer callback
def pt_scan_timer_callback(timer):
  # Called periodically no matter what as a Timer object callback
  global set_pt_speed_ratio_pub
  global set_pt_pan_ratio_pub
  global set_pt_tilt_ratio_pub
  global pan_scan_direction
  global pt_tilt_scan_ratio
  global pt_yaw_now_deg
  global pt_pitch_now_deg
  global pt_pitch_now_ratio
  global pt_speed_now_ratio
  global object_detected
  #print("Entering Scan Callback, Object Detection Value: " + str(object_detected))
  #print("Current pan_ratio: " + "%.2f" % (pt_yaw_now_ratio))
  #print("Current tilt_ratio: " + "%.2f" % (pt_pitch_now_ratio))
  #print("Current speed_ratio: " + "%.2f" % (pt_speed_now_ratio))
  if not object_detected: # if not tracking, return to scan mode
    set_pt_speed_ratio_pub.publish(PTX_SCAN_SPEED)
    #print("No Targets Found, Entering Scan Mode")
    if pt_yaw_now_deg > PTX_SCAN_PAN_LIMITS:
      #print("Soft Pan Limit Reached, Reversing Scan Direction")
      pan_scan_direction = -1
    elif pt_yaw_now_deg < (-1 * PTX_SCAN_PAN_LIMITS):
      #print("Soft Pan Limit Reached, Reversing Scan Direction")
      pan_scan_direction = 1
    pt_scan_pan_ratio = pan_scan_direction
    if pt_scan_pan_ratio < 0:
      pt_scan_pan_ratio=0
    #print("Current pan_scan_to_ratio: " + "%.2f" % (pt_scan_pan_ratio))
    #print("Current tilt_scan_to_ratio: " + "%.2f" % (pt_tilt_scan_ratio))
    set_pt_pan_ratio_pub.publish(pt_scan_pan_ratio)
    set_pt_tilt_ratio_pub.publish(pt_tilt_scan_ratio)

 
### Detection and localization of object of interest relative to FOV center in degrees
def objects_detected_callback(bounding_box_msg):
  global pan_scan_direction
  global last_object_pan_ratio
  global object_detected
  global img_area
  box_of_interest = None
  #print("Entering Detection Callback")
  # Iterate over all of the objects reported by the detector and return center of largest box in degrees relative to img center
  largest_box_area_ratio=0 # Initialize largest box area
  for box in bounding_box_msg.bounding_boxes:
    # Check for the object of interest and take appropriate actions
    if box.Class == OBJ_LABEL_OF_INTEREST:
      # Check if largest box
      box_area=(box.xmax-box.xmin)*(box.ymax-box.ymin)
      box_area_ratio = float(box_area) / img_area
      if box_area_ratio > largest_box_area_ratio:
        largest_box_area_ratio=box_area_ratio
        largest_box=box
  if largest_box_area_ratio > MIN_DETECT_BOX_AREA_RATIO:
    box_of_interest = largest_box
    object_detected=True
    # Calculate the box center in image ratio terms
    object_tilt_ratio=PTX_OBJECT_TILT_OFFSET_RATIO
    if PTX_REVERSE_TILT:
      object_tilt_ratio=1-object_tilt_ratio
    object_loc_y_pix = box_of_interest.ymin + ((box_of_interest.ymax - box_of_interest.ymin)  * object_tilt_ratio) 
    object_loc_x_pix = box_of_interest.xmin + ((box_of_interest.xmax - box_of_interest.xmin)  / 2)
    object_loc_y_ratio = float(object_loc_y_pix) / img_height
    object_loc_x_ratio = float(object_loc_x_pix) / img_width
    #print("Object Detected " + OBJ_LABEL_OF_INTEREST + " with box center (" + str(object_loc_x_ratio) + ", " + str(object_loc_y_ratio) + ")")
    # Call the tracking algorithm
    pt_track_box(object_loc_y_ratio, object_loc_x_ratio)
    # Set next scan direction in direction of target
    pan_scan_direction = np.sign(last_object_pan_ratio-object_loc_x_ratio) # direction used if object lost an scan mode start up
    last_object_pan_ratio=object_loc_x_ratio
  else:
    # Object of interest not detected, so reset object_detected
    object_detected=False  # will start scan mode on next timer event
  
def found_object_callback(found_obj_msg):
  # Must reset object_detected in the event of no objects to restart scan mode
  global object_detected
  if found_obj_msg.count == 0:
    #print("No objects found")
    object_detected=False


### Track box process based on current box center relative ratio of image
def pt_track_box(object_loc_y_ratio, object_loc_x_ratio):
  global object_detected
  global set_pt_speed_ratio_pub
  global set_pt_pan_ratio_pub
  global set_pt_tilt_ratio_pub
  #print("Entering Track Callback, Object Detection Value: " + str(object_detected)) 
  if object_detected:
    #print("Target Found, Entering Track Mode")
    # Simple bang/bang positional control with hysteresis band and error-proportional speed control
    # First check if we are close enough to center in either dimension to stop motion: Hysteresis band
    box_abs_error_x_ratio = 2.0 * abs(object_loc_x_ratio - 0.5)
    box_abs_error_y_ratio = 2.0 * abs(object_loc_y_ratio - 0.5)
    #print("Object Detection Error Ratios pan: " "%.2f" % (box_abs_error_x_ratio) + " tilt: " + "%.2f" % (box_abs_error_y_ratio))
    if (box_abs_error_y_ratio <= PTX_OBJ_CENTERED_BUFFER_RATIO ) or \
       (box_abs_error_x_ratio <= PTX_OBJ_CENTERED_BUFFER_RATIO ):
      #print_throttle(1.0, "Object is centered in frame in at least one axis: Stopping any p/t motion") 
      pt_stop_motion_pub.publish()
    # Now set the speed proportional to average error
    speed_control_value = PTX_MIN_TRACK_SPEED + (PTX_MAX_TRACK_SPEED-PTX_MIN_TRACK_SPEED) * box_abs_error_x_ratio
    #print("Current track speed ratio: " + "%.2f" % (speed_control_value))
    set_pt_speed_ratio_pub.publish(speed_control_value)
    # Per-axis bang/bang based on direction
    if box_abs_error_y_ratio > PTX_OBJ_CENTERED_BUFFER_RATIO:
      #print("Tilt Object Error To High, Adjusitng TilT") 
      tilt_axis_ratio_target = pt_forward_tilt_limit_ratio if object_loc_y_ratio < 0.5 else pt_backward_tilt_limit_ratio
      set_pt_tilt_ratio_pub.publish(tilt_axis_ratio_target)
      #print("Current tilt_track_to_ratio: " + "%.2f" % (tilt_axis_ratio_target))
    if box_abs_error_x_ratio > PTX_OBJ_CENTERED_BUFFER_RATIO:
      #print("Pan Object Error To High, Adjusitng Pan")     
      pan_axis_ratio_target = pt_forward_pan_limit_ratio if object_loc_x_ratio < 0.5 else pt_backward_pan_limit_ratio
      set_pt_pan_ratio_pub.publish(pan_axis_ratio_target)
      #print("Current pan_track_to_ratio: " + "%.2f" % (pan_axis_ratio_target))


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
  global send_pt_home_pub
  print("Shutting down: Executing script cleanup actions")
  print("Send pan and tilt to home position")
  send_pt_home_pub.publish()


### Script Entrypoint
def startNode():
  rospy.loginfo("Starting Pan Tilt Object Tracker Action Script", disable_signals=True) # Disable signals so we can force a shutdown
  rospy.init_node(name="pantilt_object_tracker_action_script")
  #initialize system including pan scan process
  initialize_actions()
  #Set up cleanup on node shutdown
  rospy.on_shutdown(cleanup_actions)
  # Spin forever (until object is detected)
  rospy.spin()


#########################################
# Main
#########################################

if __name__ == '__main__':
  startNode()   #initialize system


