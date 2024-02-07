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
# 1) Connects to Target Range and Bearing data Topic
# 2) Monitors AI detector output for specfic target class 
# 3) Changes system to Guided mode
# 4) Sends GoTo Position command based on target range and bearing
# 5) Waits to achieve setpoint
# 6) Sets system back to original mode
# 6) Delays, then waits for next detection

# Requires the following additional scripts are running
# a) ardupilot_rbx_driver_script.py
# b) zed2_idx_driver_script.py
# c) ardupilot_rbx_navpose_config_script.py
# d) ai_detector_config_script.py
# e) ai_3d_targeting_process_script.py
# f) (Optional) ardupilot_rbx_fake_gps_process_script.py if a real GPS fix is not available
# These scripts are available for download at:
# [link text](https://github.com/numurus-nepi/nepi_sample_auto_scripts)

import rospy
import time
import numpy as np
import math
import tf

from std_msgs.msg import Empty, Int8, UInt8, Bool, String, Float32, Float64, Float64MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped
from geographic_msgs.msg import GeoPoint, GeoPose, GeoPoseStamped
from mavros_msgs.msg import State, AttitudeTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL, CommandHome
from nepi_ros_interfaces.msg import TargetLocalization
from darknet_ros_msgs.msg import BoundingBoxes

###################################################
# RBX State and Mode Dictionaries
RBX_STATES = ["DISARM","ARM"]
RBX_MODES = ["STABILIZE","LAND","RTL","LOITER","GUIDED","RESUME"]
RBX_ACTIONS = ["TAKEOFF"] 

#########################################
# USER SETTINGS - Edit as Necessary 
#########################################

OBJ_LABEL_OF_INTEREST = 'chair'
TARGET_OFFSET_GOAL_M = 0.5 # How close to set setpoint to target
RESET_DELAY_S = 1 # Min delay between triggering a new move

#########################################
# ROS NAMESPACE SETUP
#########################################

# ROS namespace setup
NEPI_BASE_NAMESPACE = "/nepi/s2x/"
NEPI_NAVPOSE_SERVICE_NAME = NEPI_BASE_NAMESPACE + "nav_pose_query"
NEPI_RBX_NAMESPACE = NEPI_BASE_NAMESPACE + "ardupilot/rbx/"

# NEPI MAVLINK RBX Driver Capabilities Publish Topics
NEPI_RBX_CAPABILITIES_NAVPOSE_TOPIC = NEPI_RBX_NAMESPACE + "navpose_support"
NEPI_RBX_CAPABILITIES_STATES_TOPIC = NEPI_RBX_NAMESPACE + "state_options"
NEPI_RBX_CAPABILITIES_MODES_TOPIC = NEPI_RBX_NAMESPACE + "mode_options"
NEPI_RBX_CAPABILITIES_ACTIONS_TOPIC = NEPI_RBX_NAMESPACE + "action_options"

# NEPI MAVLINK RBX Driver Status Publish Topic
NEPI_RBX_STATUS_STATE_TOPIC = NEPI_RBX_NAMESPACE + "state"  # Int to Defined Dictionary RBX_STATES
NEPI_RBX_STATUS_MODE_TOPIC = NEPI_RBX_NAMESPACE + "mode" # Int to Defined Dictionary RBX_MODES
NEPI_RBX_STATUS_READY_TOPIC = NEPI_RBX_NAMESPACE + "ready" # Bool, True if goto is complete or no active goto process
NEPI_RBX_STATUS_GOTO_ERRORS_TOPIC = NEPI_RBX_NAMESPACE + "goto_errors" # Floats [X_Meters,Y_Meters,Z_Meters,Heading_Degrees,Roll_Degrees,Pitch_Degrees,Yaw_Degrees]
NEPI_RBX_STATUS_CMD_SUCCESS_TOPIC = NEPI_RBX_NAMESPACE + "cmd_success" # Bool - Any command that changes ready state

# NEPI MAVLINK RBX Driver Settings Subscriber Topics
NEPI_RBX_SET_TIMEOUT_SEC = 5
NEPI_RBX_SET_STATE_TOPIC = NEPI_RBX_NAMESPACE + "set_state" # Int to Defined Dictionary RBX_STATES
NEPI_RBX_SET_MODE_TOPIC = NEPI_RBX_NAMESPACE + "set_mode"  # Int to Defined Dictionary RBX_MODES
NEPI_RBX_SET_HOME_CURRENT_TOPIC = NEPI_RBX_NAMESPACE + "set_home_current" # Emplty

# NEPI MAVLINK RBX Driver Control Subscriber Topics
NEPI_RBX_GO_ACTION_TOPIC = NEPI_RBX_NAMESPACE + "go_action"  # Int to Defined Dictionary RBX_ACTIONS
NEPI_RBX_GO_HOME_TOPIC = NEPI_RBX_NAMESPACE + "go_home" # Aborts any active goto processes
NEPI_RBX_GO_STOP_TOPIC = NEPI_RBX_NAMESPACE + "go_stop" # Aborts any active goto processes
NEPI_RBX_GOTO_POSE_TOPIC = NEPI_RBX_NAMESPACE + "goto_pose" # Ignored if any active goto processes
NEPI_RBX_GOTO_POSITION_TOPIC = NEPI_RBX_NAMESPACE + "goto_position" # Ignored if any active goto processes
NEPI_RBX_GOTO_LOCATION_TOPIC = NEPI_RBX_NAMESPACE + "goto_location" # Ignored if any active goto processes

# AI 3D Targeting Subscriber Topic
TARGET_DATA_INPUT_TOPIC = NEPI_BASE_NAMESPACE + "targeting/targeting_data"

# Mission Action Topics
SNAPSHOT_TRIGGER_TOPIC = NEPI_BASE_NAMESPACE + "snapshot_event"

#########################################
# Globals
#########################################

rbx_set_state_pub = rospy.Publisher(NEPI_RBX_SET_STATE_TOPIC, UInt8, queue_size=1)
rbx_set_mode_pub = rospy.Publisher(NEPI_RBX_SET_MODE_TOPIC, UInt8, queue_size=1)
rbx_set_home_current_pub = rospy.Publisher(NEPI_RBX_SET_HOME_CURRENT_TOPIC, Empty, queue_size=1)

rbx_go_action_pub = rospy.Publisher(NEPI_RBX_GO_ACTION_TOPIC, UInt8, queue_size=1)
rbx_go_home_pub = rospy.Publisher(NEPI_RBX_GO_HOME_TOPIC, Empty, queue_size=1)
rbx_go_stop_pub = rospy.Publisher(NEPI_RBX_GO_STOP_TOPIC, Empty, queue_size=1)
rbx_goto_pose_pub = rospy.Publisher(NEPI_RBX_GOTO_POSE_TOPIC, Float64MultiArray, queue_size=1)
rbx_goto_position_pub = rospy.Publisher(NEPI_RBX_GOTO_POSITION_TOPIC, Float64MultiArray, queue_size=1)
rbx_goto_location_pub = rospy.Publisher(NEPI_RBX_GOTO_LOCATION_TOPIC, Float64MultiArray, queue_size=1)

rbx_cap_navpose = None
rbx_cap_states = None
rbx_cap_modes = None
rbx_cap_actions = None

rbx_status_state = None
rbx_status_mode = None
rbx_status_ready = None
rbx_status_goto_errors = None
rbx_status_cmd_success = None

snapshot_trigger_pub = rospy.Publisher(SNAPSHOT_TRIGGER_TOPIC, Empty, queue_size = 1)
               
#########################################
# Methods
#########################################

### System Initialization processes
def initialize_actions():
  global rbx_cap_navpose
  global rbx_cap_states
  global rbx_cap_modes
  global rbx_cap_actions
  global rbx_status_state
  global rbx_status_mode
  global rbx_status_ready
  global rbx_status_cmd_success
  global img_height
  global img_width
  ##########################################
  ### Capabilities Subscribers
  # Wait for topic
  print("Waiting for topic: " + NEPI_RBX_CAPABILITIES_NAVPOSE_TOPIC)
  wait_for_topic(NEPI_RBX_CAPABILITIES_NAVPOSE_TOPIC)
  print("Starting capabilities navpose scubscriber callback")
  rospy.Subscriber(NEPI_RBX_CAPABILITIES_NAVPOSE_TOPIC, UInt8, rbx_cap_navpose_callback)
  while rbx_cap_navpose is None and not rospy.is_shutdown():
    print("Waiting for capabilities navpose to publish")
    time.sleep(0.1)
  print(rbx_cap_navpose)
  # Wait for topic
  print("Waiting for topic: " + NEPI_RBX_CAPABILITIES_STATES_TOPIC)
  wait_for_topic(NEPI_RBX_CAPABILITIES_STATES_TOPIC)
  print("Starting state scubscriber callback")
  rospy.Subscriber(NEPI_RBX_CAPABILITIES_STATES_TOPIC, String, rbx_cap_states_callback)
  while rbx_cap_states is None and not rospy.is_shutdown():
    print("Waiting for capabilities states to publish")
    time.sleep(0.1)
  print(rbx_cap_states)
  # Wait for topic
  print("Waiting for topic: " + NEPI_RBX_CAPABILITIES_MODES_TOPIC)
  wait_for_topic(NEPI_RBX_CAPABILITIES_MODES_TOPIC)
  print("Starting modes scubscriber callback")
  rospy.Subscriber(NEPI_RBX_CAPABILITIES_MODES_TOPIC, String, rbx_cap_modes_callback)
  while rbx_cap_modes is None and not rospy.is_shutdown():
    print("Waiting for capabilities modes to publish")
    time.sleep(0.1)
  print(rbx_cap_modes)
  # Wait for topic
  print("Waiting for topic: " + NEPI_RBX_CAPABILITIES_ACTIONS_TOPIC)
  wait_for_topic(NEPI_RBX_CAPABILITIES_ACTIONS_TOPIC)
  print("Starting actions scubscriber callback")
  rospy.Subscriber(NEPI_RBX_CAPABILITIES_ACTIONS_TOPIC, String, rbx_cap_actions_callback)
  while rbx_cap_actions is None and not rospy.is_shutdown():
    print("Waiting for capabilities actions to publish")
    time.sleep(0.1)
  print(rbx_cap_actions)
  ##########################################
  ### Status Subscribers
  # Wait for topic
  print("Waiting for topic: " + NEPI_RBX_STATUS_STATE_TOPIC)
  wait_for_topic(NEPI_RBX_STATUS_STATE_TOPIC)
  print("Starting state scubscriber callback")
  rospy.Subscriber(NEPI_RBX_STATUS_STATE_TOPIC, Int8, rbx_state_callback)
  while rbx_status_state is None and not rospy.is_shutdown():
    print("Waiting for current state to publish")
    time.sleep(0.1)
  print(rbx_status_state)
  # Wait for topic
  print("Waiting for topic: " + NEPI_RBX_STATUS_MODE_TOPIC)
  wait_for_topic(NEPI_RBX_STATUS_MODE_TOPIC)
  print("Starting mode scubscriber callback")
  rospy.Subscriber(NEPI_RBX_STATUS_MODE_TOPIC, Int8, rbx_mode_callback)
  while rbx_status_mode is None and not rospy.is_shutdown():
    print("Waiting for current mode to publish")
    time.sleep(0.1)
  print(rbx_status_mode)
  # Wait for goto controls status to publish
  print("Waiting for topic: " + NEPI_RBX_STATUS_READY_TOPIC)
  wait_for_topic(NEPI_RBX_STATUS_READY_TOPIC)
  # Start ready status monitor
  print("Starting mavros ready status subscriber")
  rospy.Subscriber(NEPI_RBX_STATUS_READY_TOPIC, Bool, rbx_status_ready_callback)
  while rbx_status_ready is None and not rospy.is_shutdown():
    print("Waiting for ready status to publish")
    time.sleep(0.1)
  # Start goto errors status monitor
  print("Starting mavros goto errors subscriber")
  rospy.Subscriber(NEPI_RBX_STATUS_GOTO_ERRORS_TOPIC, Float64MultiArray, rbx_status_goto_errors_callback)
  while rbx_status_goto_errors is None and not rospy.is_shutdown():
    print("Waiting for goto errors status to publish")
    time.sleep(0.1)
  # Start cmd success status monitor
  print("Starting mavros cmd success subscriber")
  rospy.Subscriber(NEPI_RBX_STATUS_CMD_SUCCESS_TOPIC, Bool, rbx_status_cmd_success_callback)
  while rbx_status_cmd_success is None and not rospy.is_shutdown():
    print("Waiting for cmd success status to publish")
    time.sleep(0.1)
  print("Initialization Complete")

# Action upon detection and targeting for object of interest 
def move_to_object_callback(target_data_msg):
  global rbx_status_mode
  global rbx_status_mode_start
  print("Recieved target data message")
  print(target_data_msg)
  # Check for the object of interest and take appropriate actions
  if target_data_msg.name == OBJ_LABEL_OF_INTEREST:
    rbx_status_mode_start = rbx_status_mode
    print("Detected a " + OBJ_LABEL_OF_INTEREST)
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
      print("Switching to Guided mode")
      set_rbx_mode("GUIDED") # Change mode to Guided
      # Send goto command and wait for completion
      print("Sending setpoint position body command")
      print(setpoint_position_body_m)
      success = goto_rbx_position(setpoint_position_body_m)
      #########################################
      # Run Mission Actions
      print("Starting Mission Actions")
      success = mission_actions()
      ##########################################
      print("Switching back to original mode")
      set_rbx_mode("RESUME")
      print("Delaying next trigger for " + str(RESET_DELAY_S) + " secs")
      time.sleep(RESET_DELAY_S)
      print("Waiting for next " + OBJ_LABEL_OF_INTEREST + " detection")
    else:
      print("No valid range value for target, skipping actions")
  else:
    print("No " + OBJ_LABEL_OF_INTEREST + " type for target data")
    time.sleep(1)


## Function for custom pre-mission actions
def pre_mission_actions():
  ###########################
  # Start Your Custom Actions
  ###########################
  success = True
##  # Set Mode to Guided
##  success = set_rbx_mode("GUIDED")
##  # Arm System
##  success = set_rbx_state("ARM")
##  # Send Takeoff Command
##  success=go_rbx_action("TAKEOFF")
  ###########################
  # Stop Your Custom Actions
  ###########################
  print("Pre-Mission Actions Complete")
  return success

## Function for custom mission actions
def mission_actions():
  ###########################
  # Start Your Custom Actions
  ###########################
  ## Change Vehicle Mode to Guided
  success = True
##  print("Sending snapshot event trigger")
##  snapshot()
  ###########################
  # Stop Your Custom Actions
  ###########################
  print("Mission Actions Complete")
  return success
  
## Function for custom post-mission actions
def post_mission_actions():
  ###########################
  # Start Your Custom Actions
  ###########################
  success = True
##  #success = set_rbx_mode("LAND") # Uncomment to change to Land mode
##  #success = set_rbx_mode("LOITER") # Uncomment to change to Loiter mode
##  success = set_rbx_mode("RTL") # Uncomment to change to home mode
##  #success = set_rbx_mode("RESUME") # Uncomment to return to last mode
##  time.sleep(1)
  ###########################
  # Stop Your Custom Actions
  ###########################
  print("Post-Mission Actions Complete")
  return success


#######################
# RBX Capabilities Callbacks

def rbx_cap_navpose_callback(cap_navpose_msg):
  global rbx_cap_navpose
  rbx_cap_navpose = cap_navpose_msg.data

def rbx_cap_states_callback(cap_states_msg):
  global rbx_cap_states
  cap_states_str = cap_states_msg.data
  rbx_cap_states = eval(cap_states_str)

def rbx_cap_modes_callback(cap_modes_msg):
  global rbx_cap_modes
  cap_modes_str = cap_modes_msg.data
  rbx_cap_modes = eval(cap_modes_str)

def rbx_cap_actions_callback(cap_actions_msg):
  global rbx_cap_actions
  cap_actions_str = cap_actions_msg.data
  rbx_cap_actions = eval(cap_actions_str) 

#######################
# RBX Status Callbacks
### Callback to update rbx current state value
def rbx_state_callback(state_msg):
  global rbx_status_state
  rbx_status_state = state_msg.data  

### Callback to update rbx current mode value
def rbx_mode_callback(mode_msg):
  global rbx_status_mode
  rbx_status_mode = mode_msg.data

### Callback to update rbx ready status value
def rbx_status_ready_callback(ready_msg):
  global rbx_status_ready
  rbx_status_ready = ready_msg.data

### Callback to update rbx goto errors status value
def rbx_status_goto_errors_callback(goto_errors_msg):
  global rbx_status_goto_errors
  rbx_status_goto_errors = goto_errors_msg.data  

### Callback to update rbx cmd success status value
def rbx_status_cmd_success_callback(cmd_success_msg):
  global rbx_status_cmd_success
  rbx_status_cmd_success = cmd_success_msg.data

#######################
# RBX Settings Functions

### Function to set rbx state
def set_rbx_state(state_str):
  global rbx_status_state
  global rbx_set_state_pub
  print("*******************************")  
  print("Set State Request Recieved: " + state_str)
  success = False
  new_state_ind = -1
  for ind, state in enumerate(rbx_cap_states):
    if state == state_str:
      new_state_ind = ind
  if new_state_ind == -1:
    print("No matching state found")
  else:
    print("Setting state to: " + state_str)
    rbx_set_state_pub.publish(new_state_ind)
    timeout_timer = 0
    sleep_time_sec = 1
    while rbx_status_state != new_state_ind and timeout_timer < NEPI_RBX_SET_TIMEOUT_SEC and not rospy.is_shutdown():
      print("Waiting for rbx state " + RBX_STATES[new_state_ind] + " to set")
      print("Current rbx state is " + RBX_STATES[rbx_status_state])
      time.sleep(sleep_time_sec)
      timeout_timer = timeout_timer + sleep_time_sec
    if rbx_status_state == new_state_ind:
      success = True
  print("Current rbx state is " + RBX_STATES[rbx_status_state])
  return success
  
### Function to set rbx mode
def set_rbx_mode(mode_str):
  global rbx_status_mode
  global rbx_set_mode_pub
  print("*******************************")  
  print("Set Mode Request Recieved: " + mode_str)
  success = False
  new_mode_ind = -1
  for ind, mode in enumerate(rbx_cap_modes):
    if mode == mode_str:
      new_mode_ind = ind
  if new_mode_ind == -1:
    print("No matching mode found")
  else:
    print("Setting mode to: " + mode_str)
    rbx_set_mode_pub.publish(new_mode_ind)
    timeout_timer = 0
    sleep_time_sec = 1
    while rbx_status_mode != new_mode_ind and timeout_timer < NEPI_RBX_SET_TIMEOUT_SEC and not rospy.is_shutdown():
      print("Waiting for rbx mode " + RBX_MODES[new_mode_ind] + " to set")
      print("Current rbx mode is " + RBX_MODES[rbx_status_mode])
      time.sleep(sleep_time_sec)
      timeout_timer = timeout_timer + sleep_time_sec
    if rbx_status_mode == new_mode_ind:
      success = True
  print("Current rbx mode is " + RBX_MODES[rbx_status_mode])
  return success

### Function to set home current
def set_rbx_set_home_current():
  global rbx_set_home_current_pub
  print("*******************************")  
  print("Set Home Current Request Recieved: ")
  success = False
  rbx_set_home_current_pub.publish(Empty())
  success = True
  return success

#######################
# RBX Control Functions

### Function to send rbx action control
def go_rbx_action(action_str):
  global rbx_status_cmd_success
  global rbx_set_action_pub
  print("*******************************")  
  print("Go Action Request Recieved: " + action_str)
  success = False
  action_ind = -1
  for ind, action in enumerate(rbx_cap_actions):
    if action == action_str:
      action_ind = ind
  if action_ind == -1:
    print("No matching action found")
  else:
    ready = wait_for_rbx_status_ready()
    rbx_go_action_pub.publish(action_ind)
    ready = wait_for_rbx_status_busy()
    ready = wait_for_rbx_status_ready()
    success = rbx_status_cmd_success
  return success

### Function to send rbx home control
def go_rbx_home():
  global rbx_status_cmd_success
  global rbx_set_home_pub
  print("*******************************")  
  print("Go Home Request Recieved: ")
  success = False
  ready = wait_for_rbx_status_ready()
  rbx_go_home_pub.publish()
  ready = wait_for_rbx_status_busy()
  ready = wait_for_rbx_status_ready()
  success = rbx_status_cmd_success
  return success

### Function to call goto Location Global control
def goto_rbx_location(goto_data):
  global rbx_status_cmd_success
  global rbx_goto_location_pub
  # Send goto Location Command
  print("Starting goto Location Global Process")
  goto_location_msg = create_goto_message(goto_data)
  ready = wait_for_rbx_status_ready()
  rbx_goto_location_pub.publish(goto_location_msg)
  ready = wait_for_rbx_status_busy()
  ready = wait_for_rbx_status_ready()
  return rbx_status_cmd_success

### Function to call goto Position Body control
def goto_rbx_position(goto_data):
  global rbx_status_cmd_success
  global rbx_goto_location_pub
  # Send goto Position Command
  print("Starting goto Position Body Process")
  goto_position_msg = create_goto_message(goto_data)
  ready = wait_for_rbx_status_ready()
  rbx_goto_position_pub.publish(goto_position_msg)
  ready = wait_for_rbx_status_busy()
  ready = wait_for_rbx_status_ready()
  return rbx_status_cmd_success

### Function to call goto Attititude NED control
def goto_rbx_pose(goto_data):
  global rbx_status_cmd_success
  global rbx_goto_pose_pub
  # Send goto Attitude Command
  wait_for_rbx_status_ready()
  print("Starting goto Attitude NED Process")
  goto_attitude_msg = create_goto_message(goto_data)
  ready = wait_for_rbx_status_ready()
  rbx_goto_pose_pub.publish(goto_attitude_msg)
  ready = wait_for_rbx_status_busy()
  ready = wait_for_rbx_status_ready()
  return rbx_status_cmd_success
  
### Function to wait for goto control process to complete
def wait_for_rbx_status_ready( ):
  global rbx_status_ready
  print("Waiting for status ready = True")
  count_goal = 3 # fix for strange ready glitch
  counter = 0
  while (counter < count_goal) and (not rospy.is_shutdown()):
    if rbx_status_ready is True:
      counter = counter + 1
      #print("Got status ready = " + str(rbx_status_ready))
    else:
      counter = 0
    time.sleep(.1)
  print("Got status ready True")
  return rbx_status_ready

### Function to wait for goto control process to complete
def wait_for_rbx_status_busy( ):
  global rbx_status_ready
  print("Waiting for status ready = False")
  count_goal = 3 # fix for strange ready glitch
  counter = 0
  while (counter < count_goal) and (not rospy.is_shutdown()):
    if rbx_status_ready is False:
      counter = counter + 1
      #print("Got status ready = " + str(rbx_status_ready))
    else:
      counter = 0
    time.sleep(.1)
  print("Got status ready False")
  return rbx_status_ready

#######################
# Mission Action Functions

### Function to send snapshot event trigger and wait for completion
def snapshot():
  global snapshot_trigger_pub
  snapshot_trigger_pub.publish(Empty())
  print("Snapshot trigger sent")

#######################
# Process Functions

### Function for creating goto messages
def create_goto_message(goto):
  print(goto)
  goto_msg = Float64MultiArray()
  goto_data=[]
  for ind in range(len(goto)):
    goto_data.append(float(goto[ind]))
  print(goto_data)
  goto_msg.data = goto_data
  print("")
  print("goto Message Created")
  print(goto_msg)
  return goto_msg


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
  print("Shutting down: Executing script cleanup actions")
  success = post_mission_actions()

  
### Script Entrypoint
def startNode():
  rospy.loginfo("Starting Drone Follow Object action Script")
  rospy.init_node("drone_follow_object_action_script")
  #initialize system including pan scan process
  initialize_actions()
  #########################################
  # Run Pre-Mission Custom Actions
  print("Starting Pre-goto Actions")
  success = pre_mission_actions()
  #########################################
  # Set up object detector subscriber
  print("Starting move to object callback")
  rospy.Subscriber(TARGET_DATA_INPUT_TOPIC, TargetLocalization, move_to_object_callback, queue_size = 1)
  #########################################
  # run cleanup actions on shutdown
  rospy.on_shutdown(cleanup_actions)
  #########################################
  # Run cleanup actions on rospy shutdown
  rospy.on_shutdown(cleanup_actions)
  # Spin forever
  rospy.spin()


#########################################
# Main
#########################################

if __name__ == '__main__':
  startNode()

