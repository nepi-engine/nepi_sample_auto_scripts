#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

# Sample NEPI Mission Script.
# 1) Subscribes to NEPI nav_pose_current heading, orientation, position, location topics
# 2) Runs pre-mission processes
# 3) Runs mission goto command processes
# 4) Runs mission action processes
# 5) Runs post-mission processes

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
from nepi_edge_sdk_base import nepi_ros 
from nepi_edge_sdk_base import nepi_rbx

from std_msgs.msg import Empty, UInt8, Int8, Float32, Float64, Float64MultiArray

#########################################
# USER SETTINGS - Edit as Necessary 
#########################################
TAKEOFF_ALT_M = 1
TAKEOFF_EXTRA_WAIT_TIME = 2

# GoTo Position Global Settings
###################
# goto_location is [LAT, LONG, ALT_WGS84, YAW_NED_DEGREES]
# Altitude is specified as meters above the WGS-84 and converted to AMSL before sending
# Yaw is specified in NED frame degrees 0-360 or +-180 
GOTO_LOCATION = [47.6541208,-122.3186620, 10, -999] # [Lat, Long, Alt WGS84, Yaw NED Frame], Enter -999 to use current value
GOTO_LOCATION_CORNERS =  [[47.65412620,-122.31881480, -999, -999],[47.65402050,-122.31875320, -999, -999],[47.65391570,-122.31883630, -999, -999]]

#########################################
# ROS NAMESPACE SETUP
#########################################

# ROS namespace setup
NEPI_BASE_NAMESPACE = nepi_ros.get_base_namespace()

#########################################
# Node Class
#########################################

class drone_inspection_demo_mission(object):

  # RBX State and Mode Dictionaries
  RBX_STATES = ["DISARM","ARM"]
  RBX_MODES = ["STABILIZE","LAND","RTL","LOITER","GUIDED","RESUME"]
  RBX_ACTIONS = ["TAKEOFF"] 

  #######################
  ### Node Initialization
  def __init__(self):
    rospy.loginfo("Starting Initialization Processes")
    nepi_rbx.rbx_initialize(self, NEPI_BASE_NAMESPACE)
    # AI Detector Subscriber Topics (If Required)
    AI_BOUNDING_BOXES_TOPIC = NEPI_BASE_NAMESPACE + "classifier/bounding_boxes"
    AI_DETECTION_IMAGE_TOPIC = NEPI_BASE_NAMESPACE + "classifier/detection_image"
    # Mission Action Topics (If Required)
    SNAPSHOT_TRIGGER_TOPIC = NEPI_BASE_NAMESPACE + "snapshot_event"
    self.snapshot_trigger_pub = rospy.Publisher(SNAPSHOT_TRIGGER_TOPIC, Empty, queue_size = 1)
    ## Start Node Processes
    # Set Takeoff Height in meters
    self.rbx_set_takeoff_alt_m_pub.publish(data=TAKEOFF_ALT_M)
    ## Initiation Complete
    rospy.loginfo("Initialization Complete")

  #######################
  ### Node Methods

  ## Function for custom pre-mission actions
  def pre_mission_actions(self):
    ###########################
    # Start Your Custom Actions
    ###########################
    success = True
    # Set Mode to Guided
    success = nepi_rbx.set_rbx_mode(self,"GUIDED")
    # Arm System
    success = nepi_rbx.set_rbx_state(self,"ARM")
    # Send Takeoff Command
    success=nepi_rbx.go_rbx_action(self,"TAKEOFF")
    nepi_ros.sleep(TAKEOFF_EXTRA_WAIT_TIME,20)
    ###########################
    # Stop Your Custom Actions
    ###########################
    print("Pre-Mission Actions Complete")
    return success

  ## Function for custom mission
  def mission(self):
    ###########################
    # Start Your Custom Process
    ###########################
    success = True
    ##########################################
    # Send goto Position Command
    print("Starting goto Location Process")
    success = nepi_rbx.goto_rbx_location(self,GOTO_LOCATION)
    #########################################
    # Run Mission Actions
    print("Starting Mission Actions")
    success = self.mission_actions()
   #########################################
    # Send goto Location Loop Command
 
    for ind in range(3):
      # Send goto Location Command
      print("Starting goto Location Corners Process")
      success = nepi_rbx.goto_rbx_location(self,GOTO_LOCATION_CORNERS[ind])
      # Run Mission Actions
      print("Starting Mission Actions")
      success = self.mission_actions()
    ###########################
    # Stop Your Custom Process
    ###########################
    print("Mission Processes Complete")
    return success

  ## Function for custom mission actions
  def mission_actions(self):
    ###########################
    # Start Your Custom Actions
    ###########################
    ## Send Snapshot Trigger
    success = True
    success = nepi_rbx.set_rbx_process_name(self,"SNAPSHOT EVENT")
    rospy.loginfo("Sending snapshot event trigger")
    self.snapshot()
    nepi_ros.sleep(2,10)
    ###########################
    # Stop Your Custom Actions
    ###########################
    print("Mission Actions Complete")
    return success

  ## Function for custom post-mission actions
  def post_mission_actions(self):
    ###########################
    # Start Your Custom Actions
    ###########################
    success = True
    #success = nepi_rbx.set_rbx_mode(self,"LAND") # Uncomment to change to Land mode
    #success = nepi_rbx.set_rbx_mode(self,"LOITER") # Uncomment to change to Loiter mode
    success = nepi_rbx.set_rbx_mode(self,"RTL") # Uncomment to change to home mode
    #success = nepi_rbx.set_rbx_mode(self,"RESUME") # Uncomment to return to last mode
    nepi_ros.sleep(1,10)
    ###########################
    # Stop Your Custom Actions
    ###########################
    print("Post-Mission Actions Complete")
    return success

  #######################
  # RBX Status Callbacks
  ### Callback to update rbx current state value
  def rbx_status_callback(self,status_msg):
    self.rbx_status = status_msg

  #######################
  # Mission Action Functions

  ### Function to send snapshot event trigger and wait for completion
  def snapshot(self):
    self.snapshot_trigger_pub.publish(Empty())
    print("Snapshot trigger sent")

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
  #########################################
  # Run Pre-Mission Custom Actions
  print("Starting Pre-goto Actions")
  success = node.pre_mission_actions()
  while not rospy.is_shutdown(): 
    #########################################
    # Start Mission
    #########################################
    # Send goto Location Command
    print("Starting Mission Processes")
    success = node.mission()
    #########################################
  # End Mission
  #########################################
  # Run Post-Mission Actions
  print("Starting Post-Goto Actions")
  success = node.post_mission_actions()
  #########################################
  # Mission Complete, Shutting Down
  #rospy.signal_shutdown("Mission Complete, Shutting Down")

  #Set up node shutdown
  rospy.on_shutdown(node.cleanup_actions)
  # Spin forever (until object is detected)
  rospy.spin()


  



