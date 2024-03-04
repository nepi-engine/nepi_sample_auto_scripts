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
import math
from resources import nepi
from resources import nepi_rbx

from std_msgs.msg import Empty, UInt8, Int8, Float32, Float64, Float64MultiArray
from sensor_msgs.msg import NavSatFix, Image
from nepi_ros_interfaces.msg import TargetLocalization

#########################################
# USER SETTINGS - Edit as Necessary 
#########################################

###!!!!!!!! Set Automation action parameters !!!!!!!!
OBJ_LABEL_OF_INTEREST = "person"
TARGET_OFFSET_GOAL_M = 0.5 # How close to set setpoint to target

SNAPSHOT_TIME_S = 10 # Time to wait after detection and trigger sent
TRIGGER_RESET_DELAY_S = 3 # Min delay between triggers


#########################################
# ROS NAMESPACE SETUP
#########################################

# ROS namespace setup
NEPI_BASE_NAMESPACE = "/nepi/s2x/"

#########################################
# Node Class
#########################################

class drone_follow_object_mission(object):

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
    # AI 3D Targeting Subscribers
    # Wait for AI targeting detection topic and subscribe to it
    AI_TARGETING_TOPIC = "targeting/targeting_data"
    rospy.loginfo("Waiting for topic: " + AI_TARGETING_TOPIC)
    ai_targeting_topic_name = nepi.wait_for_topic(AI_TARGETING_TOPIC)
    rospy.loginfo("Starting move to object callback")
    rospy.Subscriber(ai_targeting_topic_name, TargetLocalization, self.move_to_object_callback, queue_size = 1)

    # Mission Action Topics (If Required)
    SNAPSHOT_TRIGGER_TOPIC = NEPI_BASE_NAMESPACE + "snapshot_event"
    self.snapshot_trigger_pub = rospy.Publisher(SNAPSHOT_TRIGGER_TOPIC, Empty, queue_size = 1)
    # Setup snapshot processes
    self.reset_delay_timer = 10000
    self.last_reset_time = time.time()
    self.rbx_set_cmd_timeout_pub.publish(5)
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

  # Action upon detection and targeting for object of interest 
  def move_to_object_callback(self,target_data_msg):
    rospy.loginfo("Recieved target data message")
    rospy.loginfo(target_data_msg)
    # Check for the object of interest and take appropriate actions
    if target_data_msg.name == OBJ_LABEL_OF_INTEREST:
      rospy.loginfo("Detected a " + OBJ_LABEL_OF_INTEREST)
      # Get target data for detected object of interest
      target_range_m = target_data_msg.range_m # [x,y,z]
      target_yaw_d = target_data_msg.azimuth_deg  # dz
      target_pitch_d = target_data_msg.elevation_deg # dy
      # Calculate setpoint to target using offset goal
      if target_range_m != -999:
        setpoint_range_m = target_range_m - TARGET_OFFSET_GOAL_M
        sp_x_m = setpoint_range_m * math.cos(math.radians(target_yaw_d))
        sp_y_m = setpoint_range_m * math.sin(math.radians(target_yaw_d))
        sp_z_m = - setpoint_range_m * math.sin(math.radians(target_pitch_d))
        sp_yaw_d = target_yaw_d
        setpoint_position_body_m = [sp_x_m,sp_y_m,sp_z_m,sp_yaw_d]
        ##########################################
        # Switch to Guided Mode and Send GoTo Position Command
        rospy.loginfo("Switching to Guided mode")
        nepi_rbx.set_rbx_mode(self,"GUIDED") # Change mode to Guided
        time.sleep(2)
        # Send goto command and wait for completion
        rospy.loginfo("Sending setpoint position body command")
        rospy.loginfo(setpoint_position_body_m)
        success = nepi_rbx.goto_rbx_position(self,setpoint_position_body_m)
        #########################################
        # Run Mission Actions
        #rospy.loginfo("Starting Mission Actions")
        #success = self.mission_actions()
        ##########################################
        rospy.loginfo("Switching back to original mode")
        nepi_rbx.set_rbx_mode(self,"RESUME")
        rospy.loginfo("Delaying next trigger for " + str(TRIGGER_RESET_DELAY_S) + " secs")
        nepi.sleep(TRIGGER_RESET_DELAY_S,100)
        rospy.loginfo("Waiting for next " + OBJ_LABEL_OF_INTEREST + " detection")
      else:
        rospy.loginfo("No valid range value for target, skipping actions")
    else:
      rospy.loginfo("No " + OBJ_LABEL_OF_INTEREST + " type for target data")
      time.sleep(1)


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


  



