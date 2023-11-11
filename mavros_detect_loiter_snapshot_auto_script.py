#!/usr/bin/env python

# Sample NEPI Automation Script.
# Uses onboard ROS python and mavros libraries to
### Expects Classifier to be running ###
# 1) Monitors detection for specfic target class 
# 2) Sets system to Loiter mode
# 3) Calls a Snapshot Event Auto Script
# 4) Delays some set time
# 5) Sets system back to original mode
# 6) Delays, then starts monitoring again


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
LOITER_DELAY_S = 10 # Time to Loiter at this spot
RESET_DELAY_S = 5 # Min delay between triggers
STATE_UPDATE_INTERVAL_SEC = 0.2

SET_MODE = 'Loiter' # Requires GPS, use mavros_fake_gps_altitude_auto_script.py to fake

# ROS namespace setup
BASE_NAMESPACE = "/nepi/s2x/"
MAVROS_NAMESPACE = BASE_NAMESPACE + "pixhawk_mavlink/"

MAVROS_STATE_TOPIC = MAVROS_NAMESPACE + "state"
MAVROS_ARMING_SERVICE = MAVROS_NAMESPACE + "cmd/arming"
MAVROS_SET_MODE_SERVICE = MAVROS_NAMESPACE + "set_mode"
CL_BOUNDING_BOXES_TOPIC = BASE_NAMESPACE + "classifier/bounding_boxes"
SNAPSHOT_TOPIC = BASE_NAMESPACE + "snapshot_event"


#####################################################################################
# Globals
#####################################################################################
arming_client = rospy.ServiceProxy(MAVROS_ARMING_SERVICE, CommandBool)
set_mode_client = rospy.ServiceProxy(MAVROS_SET_MODE_SERVICE, SetMode)
snapshot_trigger_pub = rospy.Publisher(SNAPSHOT_TOPIC, Empty, queue_size = 1)
current_state = None
org_mode = None
org_arm = None
set_mode = None # global control 
set_arm = False # global control
pub_enable = True


#####################################################################################
# Methods
#####################################################################################


### System Initialization processes
def initialize_actions():
  global current_state
  global org_mode
  global org_arm
  global set_mode
  global set_arm
  print("Starting get_state_callback")
  rospy.Subscriber(MAVROS_STATE_TOPIC, State, callback = get_state_callback)
  print("Waiting for first state update")
  while current_state is not None:
    time.sleep(0.1)
  time.sleep(1)
  print("Getting Original State")
  print(current_state)
  org_mode = current_state.mode
  org_arm = current_state.armed
  print("Starting set_state_callback")
  set_mode = org_mode
  set_arm = org_arm
  rospy.Timer(rospy.Duration(STATE_UPDATE_INTERVAL_SEC), set_state_callback)
  time.sleep(0.25)
  print("Completed Initialization")

### Callback to get current state
def get_state_callback(state_msg):
  global current_state
  current_state = state_msg

### Setup a regular set state publisher
def set_state_callback(timer):
  global arming_client
  global set_mode_client
  global set_mode
  global set_arm
  global pub_enable
  if pub_enable:
    if set_mode is not None:
      new_mode = SetModeRequest()
      new_mode.custom_mode = set_mode
      set_mode_client.call(new_mode)
    arm_cmd = CommandBoolRequest()
    arm_cmd.value = set_arm
    arming_client.call(arm_cmd)

# Action upon detection of object of interest
def object_detected_callback(bounding_box_msg):
  global org_mode
  global org_arm
  global set_mode
  global set_arm
  global snapshot_trigger_pub
  # Iterate over all of the objects reported by the detector
  for box in bounding_box_msg.bounding_boxes:
    # Check for the object of interest and take appropriate actions
    if box.Class == OBJ_LABEL_OF_INTEREST:
      print("Detected a " + OBJ_LABEL_OF_INTEREST)
      print("Switching to " + str(SET_MODE) + " mode")
      set_mode = SET_MODE
      set_arm = True
      time.sleep(1)
      print("Sending snapshot event trigger")
      snapshot_trigger_pub.publish()
      print("Staying in " + str(SET_MODE) + " for " + str(LOITER_DELAY_S) + " secs")
      time.sleep(RESET_DELAY_S)
      print("Switching to original mode")
      set_mode = org_mode
      set_arm = org_arm
      time.sleep(1)
      print("Delaying next trigger for " + str(RESET_DELAY_S) + " secs")
      time.sleep(RESET_DELAY_S)

### Cleanup processes on node shutdown
def cleanup_actions():
  global current_state
  global org_mode
  global org_arm
  global set_mode
  global set_arm
  global pub_enable
  ("Setting System End_Mode and End_Arm states")
  set_mode=org_mode
  set_arm=org_arm
  while(current_state.mode != org_mode  or current_state.armed != org_arm):
    time.sleep(0.1)
  pub_enable = False
  time.sleep(2)
  
### Script Entrypoint
def startNode():
  rospy.loginfo("Starting MAVROS Set Attitude Target automation script")
  rospy.init_node("mavros_set_attitude_target_auto_script")
  #initialize system including pan scan process
  initialize_actions()
  # Set up object detector subscriber
  rospy.loginfo("Starting object detection subscriber: Object of interest = " + OBJ_LABEL_OF_INTEREST + "...")
  rospy.Subscriber(CL_BOUNDING_BOXES_TOPIC, BoundingBoxes, object_detected_callback, queue_size = 1)
  # run cleanup actions on shutdown
  rospy.on_shutdown(cleanup_actions)
  # Spin forever
  rospy.spin()


#####################################################################################
# Main
#####################################################################################

if __name__ == '__main__':
  startNode()

