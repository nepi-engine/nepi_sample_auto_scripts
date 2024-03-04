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

# Sample NEPI Mission Script.
### Expects Classifier to be running ###
# 1) Monitors AI detector output for specfic target class 
# 3) Changes system to Loiter mode on detection
# 4) Sends NEPI snapshot event trigger
# 5) Waits to achieve waits set time to complete snapshot events
# 6) Sets system back to original mode
# 6) Delays, then waits for next detection

# Requires the following additional scripts are running
# a) ardupilot_rbx_driver_script.py
# (Optional) Some Snapshot Action Automation Script like the following
#   b)snapshot_event_save_to_disk_action_script.py
#   c)snapshot_event_send_to_cloud_action_script.py
# d) (Optional) ardupilot_rbx_fake_gps_process_script.py if a real GPS fix is not available
# These scripts are available for download at:
# [link text](https://github.com/numurus-nepi/nepi_sample_auto_scripts)

import rospy
import sys
import time
from resources import nepi
from resources import nepi_rbx

from std_msgs.msg import Empty, UInt8, Int8, Float32, Float64, Float64MultiArray
from sensor_msgs.msg import NavSatFix, Image
from nepi_ros_interfaces.msg import TargetLocalization
from darknet_ros_msgs.msg import BoundingBoxes

#########################################
# USER SETTINGS - Edit as Necessary 
#########################################

###!!!!!!!! Set Automation action parameters !!!!!!!!
OBJ_LABEL_OF_INTEREST = "person"
OBJ_CENTERED_BUFFER_RATIO = 0.5 # acceptable band about center of image for saving purposes

SNAPSHOT_TIME_S = 5 # Time to wait after detection and trigger sent
TRIGGER_RESET_DELAY_S = 10 # Min delay between triggers


#########################################
# ROS NAMESPACE SETUP
#########################################

# ROS namespace setup
NEPI_BASE_NAMESPACE = "/nepi/s2x/"

#########################################
# Node Class
#########################################

class drone_detect_loiter_snapshot_mission(object):

  # RBX State and Mode Dictionaries
  RBX_STATES = ["DISARM","ARM"]
  RBX_MODES = ["STABILIZE","LAND","RTL","LOITER","GUIDED","RESUME"]
  RBX_ACTIONS = ["TAKEOFF"] 

  #######################
  ### Node Initialization
  def __init__(self):
    rospy.loginfo("Starting Initialization Processes")
    nepi_rbx.rbx_initialize(self, NEPI_BASE_NAMESPACE)
    ## Start Node Processes
    # AI Detection Subscribers
    AI_BOUNDING_BOXES_TOPIC = NEPI_BASE_NAMESPACE + "classifier/bounding_boxes"
    AI_DETECTION_IMAGE_TOPIC = NEPI_BASE_NAMESPACE + "classifier/detection_image"
    # Wait for AI detection topic and subscribe to bounding box topic
    self.img_width = 0
    self.img_height = 0
    rospy.loginfo("Connecting to NEPI Detector Image Topic")
    rospy.loginfo(AI_DETECTION_IMAGE_TOPIC )
    rospy.loginfo("Waiting for topic: " + AI_DETECTION_IMAGE_TOPIC)
    ai_image_topic_name = nepi.wait_for_topic(AI_DETECTION_IMAGE_TOPIC)
    img_sub = rospy.Subscriber(ai_image_topic_name, Image, self.ai_image_callback)
    while self.img_width == 0 and self.img_height == 0:
      rospy.loginfo("Waiting for Classifier Detection Image")
      time.sleep(1)
    img_sub.unregister() # Don't need it anymore
    # Set up object detector subscriber
    rospy.loginfo("Starting object detection subscriber: Object of interest = " + OBJ_LABEL_OF_INTEREST + "...")
    rospy.Subscriber(AI_BOUNDING_BOXES_TOPIC, BoundingBoxes, self.object_detected_callback, queue_size = 1)
    # Mission Action Topics (If Required)
    SNAPSHOT_TRIGGER_TOPIC = NEPI_BASE_NAMESPACE + "snapshot_event"
    self.snapshot_trigger_pub = rospy.Publisher(SNAPSHOT_TRIGGER_TOPIC, Empty, queue_size = 1)
    # Setup snapshot processes
    self.reset_delay_timer = 10000
    self.last_reset_time = time.time()
    ## Initiation Complete
    rospy.loginfo("Initialization Complete")
    rospy.loginfo("Waiting for AI Object Detection")    

  #######################
  ### Node Methods    

  ## Function for custom mission actions
  def mission_actions(self):
    ###########################
    # Start Your Custom Actions
    ###########################
    success = True

    #########################################
    rospy.loginfo("Sending snapshot event trigger")
    self.snapshot()
    rospy.loginfo("Waiting for " + str(SNAPSHOT_TIME_S) + " secs after trigger")
    nepi.sleep(SNAPSHOT_TIME_S,100)
    ###########################
    # Stop Your Custom Actions
    ###########################
    rospy.loginfo("Mission Actions Complete")
    return success

  #######################
  # RBX Status Callbacks
  ### Callback to update rbx current state value
  def rbx_status_callback(self,status_msg):
    self.rbx_status = status_msg

  #######################
  # AI Detection Functions
  
    ### Simple callback to get image height and width
  def ai_image_callback(self,img_msg):
    # This is just to get the image size for ratio purposes
    if (self.img_height == 0 and self.img_width == 0):
      rospy.loginfo("Initial input image received. Size = " + str(img_msg.width) + "x" + str(img_msg.height))
      self.img_height = img_msg.height
      self.img_width = img_msg.width

  # Action upon detection of object of interest
  def object_detected_callback(self,bounding_box_msg):
    global snapshot_trigger_pub
    # Iterate over all of the objects reported by the detector
    if self.reset_delay_timer > TRIGGER_RESET_DELAY_S:
      for box in bounding_box_msg.bounding_boxes:
        rospy.loginfo("Target type " + box.Class + " detected")
        # Check for the object of interest and take appropriate actions
        if box.Class == OBJ_LABEL_OF_INTEREST:
          box_of_interest=box
          rospy.loginfo(box_of_interest.Class)
          # Calculate the box center in image ratio terms
          object_loc_y_pix = box_of_interest.ymin + ((box_of_interest.ymax - box_of_interest.ymin)  / 2) 
          object_loc_x_pix = box_of_interest.xmin + ((box_of_interest.xmax - box_of_interest.xmin)  / 2)
          object_loc_y_ratio = float(object_loc_y_pix) / self.img_height
          object_loc_x_ratio = float(object_loc_x_pix) / self.img_width
          rospy.loginfo("Object Detected " + OBJ_LABEL_OF_INTEREST + " with box center (" + str(object_loc_x_ratio) + ", " + str(object_loc_y_ratio) + ")")
          # check if we are AIose enough to center in either dimension to stop motion: Hysteresis band
          box_abs_error_x_ratio = 2.0 * abs(object_loc_x_ratio - 0.5)
          box_abs_error_y_ratio = 2.0 * abs(object_loc_y_ratio - 0.5)
          rospy.loginfo("Object Detection Error Ratios Horz: " "%.2f" % (box_abs_error_x_ratio) + " Vert: " + "%.2f" % (box_abs_error_y_ratio))
          if (box_abs_error_y_ratio <= OBJ_CENTERED_BUFFER_RATIO ) and \
             (box_abs_error_x_ratio <= OBJ_CENTERED_BUFFER_RATIO ):
            rospy.loginfo("Detected a " + OBJ_LABEL_OF_INTEREST + " close to image center")
            ##########################################
            # Switch to Loiter Mode and Send Snapshot Event Trigger
            rospy.loginfo("Switching to Loiter mode")
            success = nepi_rbx.set_rbx_mode(self,"LOITER") # Change mode to Loiter
            #########################################
            # Run Mission Actions
            rospy.loginfo("Starting Mission Actions")
            success = self.mission_actions()
            #########################################
            rospy.loginfo("Switching back to original mode")
            success = nepi_rbx.set_rbx_mode(self,"RESUME")
            #########################################
            rospy.loginfo("Delaying next trigger for " + str(TRIGGER_RESET_DELAY_S) + " secs")
            nepi.sleep(TRIGGER_RESET_DELAY_S,100)
            self.reset_delay_timer = 0
            self.last_reset_time = time.time()
            rospy.loginfo("Waiting for AI Object Detection")
        else:
          rospy.loginfo("Target not type " + OBJ_LABEL_OF_INTEREST)
    else:
      self.reset_delay_timer = time.time() - self.last_reset_time


  #######################
  # Mission Action Functions

  ### Function to send snapshot event trigger and wait for completion
  def snapshot(self):
    self.snapshot_trigger_pub.publish(Empty())
    rospy.loginfo("Snapshot trigger sent")

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


  



