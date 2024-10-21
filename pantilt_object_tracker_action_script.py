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

from std_msgs.msg import UInt8, Empty, Float32
from sensor_msgs.msg import Image
from nepi_ros_interfaces.msg import PanTiltLimits, PanTiltPosition, PanTiltStatus, StringArray
from darknet_ros_msgs.msg import BoundingBoxes, ObjectCount


#########################################
# USER SETTINGS - Edit as Necessary 
#########################################



OBJECT_LABEL_OF_INTEREST = "person"

# AI Detector Image ROS Topic Name or Partial Name
IMAGE_INPUT_TOPIC_NAME = "color_2d_image"
MIN_DETECT_BOX_AREA_RATIO = 0.01 # Filters background targets.


# Pan and Tilt scan settings
PTX_SCAN_PAN_LIMIT_DEG = 40 # +- Pan Angle Limits
PTX_SCAN_TILT_RATIO = 0.15 # Tilt Angle During Scanning + Up
PTX_SCAN_SPEED_RATIO = 0.6
PTX_SCAN_CHECK_INTERVAL = 0.25

# Pan and Tilt tracking settings
PTX_MAX_TRACK_SPEED_RATIO = 1.0
PTX_MIN_TRACK_SPEED_RATIO = 0.1
PTX_OBJECT_TILT_OFFSET_RATIO = 0.15 # Adjust tilt center to lower or raise the calculated object center
PTX_OBJ_CENTERED_BUFFER_RATIO = 0.15 # Hysteresis band about center of image for tracking purposes



#########################################
# Node Class
#########################################

class pantilt_object_tracker(object):

  #######################
  ### Node Initialization
  DEFAULT_NODE_NAME = "pantilt_object_tracker" # Can be overwitten by luanch command
  def __init__(self):
    #### APP NODE INIT SETUP ####
    nepi_ros.init_node(name= self.DEFAULT_NODE_NAME)
    self.node_name = nepi_ros.get_node_name()
    self.base_namespace = nepi_ros.get_base_namespace()
    nepi_msg.createMsgPublishers(self)
    nepi_msg.publishMsgInfo(self,"Starting Initialization Processes")
    ##############################
    ## Initialize Class Variables
    self.object_label_of_interest = OBJECT_LABEL_OF_INTEREST
    self.img_width = 0 # Updated on receipt of first image
    self.img_height = 0 # Updated on receipt of first image
    self.img_area = 0 # Updated on receipt of first image
    self.object_detected = False

    self.pan_scan_direction = 1 # Keep track of current scan direction (1: Positive Limit, -1: Negative Limit)
    self.img_width = 0 # Updated on receipt of first image
    self.img_height = 0 # Updated on receipt of first image
    
    self.status = None

    self.last_object_pan_ratio=0
    self.total_tilt_degs = 90
    self.current_tilt_ratio = 0.5
    self.total_pan_degs = 90
    self.current_pan_ratio = 0.5

    ## Define Class Namespaces
    # Wait for ptx status topic to publish
    ptx_status_topic = "/ptx/status"
    nepi_msg.publishMsgInfo(self,"Waiting for topic name: " + ptx_status_topic)
    ptx_topic=nepi_ros.wait_for_topic(ptx_status_topic)
    PTX_NAMESPACE = (ptx_topic.rpartition("ptx")[0] + "ptx/")
    nepi_msg.publishMsgInfo(self,"Found ptx namespace: " + PTX_NAMESPACE)
    # PanTilt Status Topics
    PTX_GET_STATUS_TOPIC = PTX_NAMESPACE + "status"
    # PanTilt Control Publish Topics
    PTX_SET_SPEED_RATIO_TOPIC = PTX_NAMESPACE + "set_speed_ratio"
    PTX_GOHOME_TOPIC = PTX_NAMESPACE + "go_home"
    PTX_STOP_TOPIC = PTX_NAMESPACE + "stop_moving"
    PTX_GOTO_PAN_RATIO_TOPIC = PTX_NAMESPACE + "jog_to_yaw_ratio"
    PTX_GOTO_TILT_RATIO_TOPIC = PTX_NAMESPACE + "jog_to_pitch_ratio"
    PTX_SET_SOFT_LIMITS_TOPIC = PTX_NAMESPACE + "set_soft_limits"

    # AI Detector Subscriber Topics
    AI_BOUNDING_BOXES_TOPIC = self.base_namespace + "ai_detector_mgr/bounding_boxes"
    AI_DETECTION_IMAGE_TOPIC = self.base_namespace + "ai_detector_mgr/detection_image"
    AI_FOUND_OBJECT_TOPIC = self.base_namespace + "ai_detector_mgr/found_object"
    ## Class subscribers

    # Wait for AI detector image topic to publish
    nepi_msg.publishMsgInfo(self,"Connecting to NEPI Detector Image Topic")
    nepi_msg.publishMsgInfo(self,AI_DETECTION_IMAGE_TOPIC )
    nepi_msg.publishMsgInfo(self,"Waiting for topic: " + AI_DETECTION_IMAGE_TOPIC)
    nepi_ros.wait_for_topic(AI_DETECTION_IMAGE_TOPIC)
    img_sub = rospy.Subscriber(AI_DETECTION_IMAGE_TOPIC, Image, self.image_callback)
    while self.img_width == 0 and self.img_height == 0 and not rospy.is_shutdown():
      nepi_msg.publishMsgInfo(self,"Waiting for Detection Image")
      nepi_ros.sleep(1,100)
    img_sub.unregister() # Don't need it anymore
    ## Create Class Publishers
    self.send_pt_home_pub = rospy.Publisher(PTX_GOHOME_TOPIC, Empty, queue_size=10)
    self.set_pt_speed_ratio_pub = rospy.Publisher(PTX_SET_SPEED_RATIO_TOPIC, Float32, queue_size=10)
    self.set_pt_pan_ratio_pub = rospy.Publisher(PTX_GOTO_PAN_RATIO_TOPIC, Float32, queue_size=10)
    self.set_pt_tilt_ratio_pub = rospy.Publisher(PTX_GOTO_TILT_RATIO_TOPIC, Float32, queue_size=10)
    self.set_pt_soft_limits_pub = rospy.Publisher(PTX_SET_SOFT_LIMITS_TOPIC, PanTiltLimits, queue_size=10)
    self.pt_stop_motion_pub = rospy.Publisher(PTX_STOP_TOPIC, Empty, queue_size=10)
    ## Start Class Subscribers
    # Start PT Status Callback
    print("Starting Pan Tilt Stutus callback")
    rospy.Subscriber(PTX_GET_STATUS_TOPIC, PanTiltStatus, self.pt_status_callback)  
    # Set up object detector subscriber
    nepi_msg.publishMsgInfo(self,"Starting object detection subscriber: Object of interest = " + self.object_label_of_interest + "...")
    rospy.Subscriber(AI_BOUNDING_BOXES_TOPIC, BoundingBoxes, self.object_detected_callback, queue_size = 1)
    #Set up found object subscriber which monitors all AI outputs
    nepi_msg.publishMsgInfo(self,"Starting found object subscriber")
    rospy.Subscriber(AI_FOUND_OBJECT_TOPIC, ObjectCount, self.found_object_callback, queue_size = 1)
    ## Start Node Processes
    # Set up the timer that start scanning when no objects are detected
    print("Setting up pan/tilt scan check timer")
    rospy.Timer(rospy.Duration(PTX_SCAN_CHECK_INTERVAL), self.pt_scan_timer_callback)

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
      self.img_area = self.img_height*self.img_width

  ### Simple callback to get pt status info
  def pt_status_callback(self,PanTiltStatus):
    # This is just to get the current pt positions
    self.status = PanTiltStatus

    self.total_tilt_degs = self.status.pitch_max_softstop_deg - self.status.pitch_min_softstop_deg
    tilt_ratio = 0.5 + self.status.pitch_now_deg / (self.total_tilt_degs)
    if self.status.reverse_pitch_control:
      self.current_tilt_ratio = 1 - tilt_ratio
    self.total_pan_degs = self.status.yaw_max_softstop_deg - self.status.yaw_min_softstop_deg
    pan_ratio = 0.5 + self.status.yaw_now_deg / (self.total_pan_degs)
    if self.status.reverse_yaw_control:
      self.current_pan_ratio = 1 - pan_ratio
    
  ### Setup a regular background scan process based on timer callback
  def pt_scan_timer_callback(self,timer):
    # Called periodically no matter what as a Timer object callback
    if not self.object_detected: # if not tracking, return to scan mode
      #print("No Targets Found, Entering Scan Mode")
      if self.status.yaw_now_deg > PTX_SCAN_PAN_LIMIT_DEG:
        print("Soft Pan Limit Reached, Reversing Scan Direction")
        if self.status.reverse_yaw_control == False:
          self.pan_scan_direction = -1
        else:
          self.pan_scan_direction = 1
      elif self.status.yaw_now_deg < (-1 * PTX_SCAN_PAN_LIMIT_DEG):
        print("Soft Pan Limit Reached, Reversing Scan Direction")
        if self.status.reverse_yaw_control == False:
          self.pan_scan_direction = 1
        else:
          self.pan_scan_direction = -1
      pan_ratio = (self.pan_scan_direction + 1) /2

      tilt_offset_ratio =  PTX_SCAN_TILT_RATIO
      if self.status.reverse_pitch_control == False:
        tilt_offset_ratio = - tilt_offset_ratio
      tilt_ratio = 0.5 + tilt_offset_ratio
      speed_ratio = PTX_SCAN_SPEED_RATIO
      print("Current pan_scan_to_dir: " + "%.2f" % (pan_ratio))
      print("Current tilt_scan_to_dir: " + "%.2f" % (tilt_ratio))
      self.set_pt_speed_ratio_pub.publish(speed_ratio)
      self.set_pt_pan_ratio_pub.publish(pan_ratio)
      self.set_pt_tilt_ratio_pub.publish(tilt_ratio)



  # Action upon detection of object of interest
  def object_detected_callback(self,bounding_box_msg):
    box_of_interest = None
    #print("Entering Detection Callback")
    # Iterate over all of the objects reported by the detector and return center of largest box in degrees relative to img center
    largest_box_area_ratio=0 # Initialize largest box area
    for box in bounding_box_msg.bounding_boxes:
      # Check for the object of interest and take appropriate actions
      if box.Class == OBJECT_LABEL_OF_INTEREST:
        # Check if largest box
        box_area=(box.xmax-box.xmin)*(box.ymax-box.ymin)
        box_area_ratio = float(box_area) / self.img_area
        if box_area_ratio > largest_box_area_ratio:
          largest_box_area_ratio=box_area_ratio
          largest_box=box
    if largest_box_area_ratio > MIN_DETECT_BOX_AREA_RATIO:
      box_of_interest = largest_box
      self.object_detected = True
      # Calculate the box center in image ratio terms
      object_loc_y_pix = box_of_interest.ymin + ((box_of_interest.ymax - box_of_interest.ymin)  / 2) 
      object_loc_x_pix = box_of_interest.xmin + ((box_of_interest.xmax - box_of_interest.xmin)  / 2)
      object_loc_y_ratio = float(object_loc_y_pix) / self.img_height - PTX_OBJECT_TILT_OFFSET_RATIO
      object_loc_x_ratio = float(object_loc_x_pix) / self.img_width
      object_error_y_ratio = (object_loc_y_ratio - 0.5)  
      object_error_x_ratio = (object_loc_x_ratio - 0.5) 
      #print("Object Detection Center Error Ratios  x: " + "%.2f" % (object_error_x_ratio) + " y: " + "%.2f" % (object_error_y_ratio))
      # Call the tracking algorithm
      self.pt_track_box(object_error_x_ratio, object_error_y_ratio)
    else:
      # Object of interest not detected, so reset object_detected
      self.object_detected=False  # will start scan mode on next timer event
  
  def found_object_callback(self,found_obj_msg):
    # Must reset object_detected in the event of no objects to restart scan mode
    if found_obj_msg.count == 0:
      #print("No objects found")
      self.object_detected=False

  ### Track box process based on current box center relative ratio of image
  def pt_track_box(self,object_error_x_ratio, object_error_y_ratio):
    #print("Entering Track Callback, Object Detection Value: " + str(self.object_detected)) 
    if self.object_detected:
      #print("Target Found, Entering Track Mode")
      # Simple bang/bang positional control with hysteresis band and error-proportional speed control
      # First check if we are close enough to center in either dimension to stop motion: Hysteresis band
      # Adjust the vertical box error to better center on target
      print("Object Detection Error Ratios pan: " + "%.2f" % (object_error_x_ratio) + " tilt: " + "%.2f" % (object_error_y_ratio))
      if (abs(object_error_y_ratio) <= PTX_OBJ_CENTERED_BUFFER_RATIO ) and \
         (abs(object_error_x_ratio) <= PTX_OBJ_CENTERED_BUFFER_RATIO ):
        #print("Object is centered in frame: Stopping any p/t motion") 
        self.pt_stop_motion_pub.publish()
      else:
        #print("Object not centered in frame")
        # Now set the speed proportional to average error
        speed_control_value = PTX_MIN_TRACK_SPEED_RATIO + \
                              (PTX_MAX_TRACK_SPEED_RATIO-PTX_MIN_TRACK_SPEED_RATIO) * max(abs(object_error_x_ratio),abs(object_error_y_ratio))
        #print("Current track speed ratio: " + "%.2f" % (speed_control_value))
        self.set_pt_speed_ratio_pub.publish(speed_control_value)
        # Per-axis adjustment
        self.move_pan_rel_ratio(object_error_x_ratio)
        self.move_tilt_rel_ratio(object_error_y_ratio)
        # set next scan in direction of last detection
        if self.status.reverse_yaw_control == False:
          self.pan_scan_direction = - np.sign(object_error_x_ratio)
        else:
          self.pan_scan_direction = np.sign(object_error_x_ratio)
        #print("X Error: " + "%.2f" % (object_error_x_ratio))
        #print("New Scan Dir: " + str(self.pan_scan_direction))



  def move_pan_rel_ratio(self,pan_rel_ratio):
    print("Pan Track Info")
    print(self.current_pan_ratio)
    print(pan_rel_ratio)
    if self.status.reverse_yaw_control == False:
      pan_ratio = self.current_pan_ratio - pan_rel_ratio
    else:
      pan_ratio = self.current_pan_ratio + pan_rel_ratio
    print(pan_ratio)
    if pan_ratio < 0.0:
      pan_ratio = 0
    elif pan_ratio > 1.0:
      pan_ratio = 1
    if not rospy.is_shutdown():
      print("Current pan_to_ratio: " + "%.2f" % (pan_ratio))
      self.set_pt_pan_ratio_pub.publish(pan_ratio)

  def move_tilt_rel_ratio(self,tilt_rel_ratio):
    #print("Tilt Track Info")
    #print(self.current_tilt_ratio)
    #print(tilt_rel_ratio)
    if self.status.reverse_pitch_control == True:
      tilt_ratio = self.current_tilt_ratio - tilt_rel_ratio
    else:
      tilt_ratio = self.current_tilt_ratio + tilt_rel_ratio
    #print(tilt_ratio)
    if tilt_ratio < 0:
      tilt_ratio = 0
    elif tilt_ratio > 1:
      tilt_ratio = 1
    if not rospy.is_shutdown():
      print("Current tilt_to_ratio: " + "%.2f" % (tilt_ratio))
      self.set_pt_tilt_ratio_pub.publish(tilt_ratio)


  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    global led_intensity_pub
    nepi_msg.publishMsgInfo(self,"Shutting down: Executing script cleanup actions")



#########################################
# Main
#########################################
if __name__ == '__main__':
  pantilt_object_tracker()


