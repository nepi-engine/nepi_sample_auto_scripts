#!/usr/bin/env python

# Sample NEPI Automation Script.
# Uses onboard ROS python and mavros libraries to
### Expects Classifier to be running ###
# 1) Monitors detection for specfic target class 
# 2) Sets system to Loiter mode
# 3) Calls a Snapshot Event Auto Script
# 4) Delays some set time for snapshot action
# 5) Sets system back to original mode
# 6) Delays, then starts monitoring again

# Requires the following additional scripts are running
# (Optional) Some Snapshot Action Automation Script like the following
# a)snapshot_event_save_data_auto_script.py
# b)snapshot_event_send_to_cloud_auto_script.py
# These scripts are available for download at:
# [link text](https://github.com/numurus-nepi/nepi_sample_auto_scripts)

import rospy
import time
import numpy as np

from std_msgs.msg import Empty
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from darknet_ros_msgs.msg import BoundingBoxes


#####################################################################################
# SETUP - Edit as Necessary ##################################
##########################################

OBJ_LABEL_OF_INTEREST = 'person'
SET_MODE = 'Loiter' # Requires GPS, use mavros_fake_gps_altitude_auto_script.py to fake
ACTION_DELAY_S = 10 # Time to Loiter at this spot
RESET_DELAY_S = 5 # Min delay between triggers


# ROS namespace setup
NEPI_BASE_NAMESPACE = "/nepi/s2x/"
MAVROS_NAMESPACE = NEPI_BASE_NAMESPACE + "pixhawk_mavlink/"
# MAVROS Subscriber Topics
MAVROS_STATE_TOPIC = MAVROS_NAMESPACE + "state"
# MAVROS Required Services
MAVROS_ARMING_SERVICE = MAVROS_NAMESPACE + "cmd/arming"
MAVROS_SET_MODE_SERVICE = MAVROS_NAMESPACE + "set_mode"
# AI Classifier Subscription Topic
AI_BOUNDING_BOXES_TOPIC = NEPI_BASE_NAMESPACE + "classifier/bounding_boxes"
# Snapshot Event Publish Topic
SNAPSHOT_TOPIC = NEPI_BASE_NAMESPACE + "snapshot_event"


#####################################################################################
# Globals
#####################################################################################
arming_client = rospy.ServiceProxy(MAVROS_ARMING_SERVICE, CommandBool)
set_mode_client = rospy.ServiceProxy(MAVROS_SET_MODE_SERVICE, SetMode)
snapshot_trigger_pub = rospy.Publisher(SNAPSHOT_TOPIC, Empty, queue_size = 1)
state_current = None
org_mode = None
org_arm = None

#####################################################################################
# Methods
#####################################################################################

### System Initialization processes
def initialize_actions():
  global state_current
  global org_mode
  global org_arm
  global set_mode
  global set_arm
  print("Starting get_state_callback")
  rospy.Subscriber(MAVROS_STATE_TOPIC, State, callback = get_state_callback)
  print("Waiting for first state update")
  while state_current is not None:
    time.sleep(0.1)
  time.sleep(1)
  print("Getting Original State")
  print(state_current)
  org_mode = state_current.mode
  org_arm = state_current.armed
  print("Completed Initialization")

### Callback to get current state
def get_state_callback(state_msg):
  global state_current
  state_current = state_msg

### Function to set mode
def update_mode(mode_new):
  global state_current
  global set_mode_client
  print('gothere')
  new_mode = SetModeRequest()
  new_mode.custom_mode = mode_new
  print("Updating mode")
  print(mode_new)
  while state_current.mode != mode_new:
    set_mode_client.call(new_mode)
    print("Waiting for mode to set")
    time.sleep(.1)

### Function to set armed state
def update_armed(armed_new):
  global state_current
  global arming_client
  arm_cmd = CommandBoolRequest()
  arm_cmd.value = armed_new
  print("Updating armed")
  print(armed_new)
  while state_current.armed != armed_new:
    arming_client.call(arm_cmd)
    print("Waiting for armed value to set")
    time.sleep(.1)

# Action upon detection of object of interest
def object_detected_callback(bounding_box_msg):
  global org_mode
  global snapshot_trigger_pub
  # Iterate over all of the objects reported by the detector
  for box in bounding_box_msg.bounding_boxes:
    # Check for the object of interest and take appropriate actions
    if box.Class == OBJ_LABEL_OF_INTEREST:
      print("Detected a " + OBJ_LABEL_OF_INTEREST)
      print("Switching to " + str(SET_MODE) + " mode")
      update_mode(SET_MODE)
      print("Sending snapshot event trigger")
      snapshot_trigger_pub.publish(Empty())
      print("Staying in " + str(SET_MODE) + " for " + str(ACTION_DELAY_S) + " secs")
      time.sleep(RESET_DELAY_S)
      print("Switching back to original mode")
      update_mode(org_mode)
      print("Delaying next trigger for " + str(RESET_DELAY_S) + " secs")
      time.sleep(RESET_DELAY_S)

### Cleanup processes on node shutdown
def cleanup_actions():
  global org_mode
  time.sleep(.1)
  
### Script Entrypoint
def startNode():
  rospy.loginfo("Starting MAVROS Set Attitude Target automation script")
  rospy.init_node("mavros_set_attitude_target_auto_script")
  #initialize system including pan scan process
  initialize_actions()
  # Set up object detector subscriber
  print("Waiting for Classifier Topic to detect and publish")
  rospy.Subscriber(AI_BOUNDING_BOXES_TOPIC, BoundingBoxes, object_detected_callback, queue_size = 1)
  # run cleanup actions on shutdown
  rospy.on_shutdown(cleanup_actions)
  # Spin forever
  rospy.spin()


#####################################################################################
# Main
#####################################################################################

if __name__ == '__main__':
  startNode()

