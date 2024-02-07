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

# Sample NEPI Driver Script. 
# NEPI RBX Driver for Ardupilot Autopilot Systems

### Set the namespace before importing rospy
import os
os.environ["ROS_NAMESPACE"] = "/nepi/s2x"


import rospy
import time
import numpy as np
import math
import tf
import random
import sys
import cv2

# try and import geoid height calculation module and databases
GEOID_DATABASE_FILE='/mnt/nepi_storage/databases/geoids/egm2008-2_5.pgm' # Ignored if PyGeodesy module or Geoids Database is not available
FALLBACK_GEOID_HEIGHT_M = 0.0 # Ignored if if PyGeodesy module or Geoids Database are available
try:
  import pygeodesy
  GEOID_DATABASE_FILE=GEOID_DATABASE_FILE
  print(['Loading Geoids Database from: ' + GEOID_DATABASE_FILE])
  ginterpolator = pygeodesy.GeoidKarney(GEOID_DATABASE_FILE)
except rospy.ServiceException as e:
  print("Geoids database failed to import: %s"%e)
  def ginterpolator(single_position):
    return FALLBACK_GEOID_HEIGHT_M

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Bool, String, Float32, Float64, Float64MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, NavSatFix, BatteryState
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped
from geographic_msgs.msg import GeoPoint, GeoPose, GeoPoseStamped
from mavros_msgs.msg import State, AttitudeTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL, CommandHome
from nepi_ros_interfaces.srv import NavPoseQuery, NavPoseQueryRequest
from pygeodesy.ellipsoidalKarney import LatLon
from cv_bridge import CvBridge


#########################################
# DRIVER SETTINGS
#########################################

##############################
# RBX State and Mode Dictionaries
RBX_NAVPOSE = 7 # Byte Mask [GPS, Heading, Orienation]
RBX_STATES = ["DISARM","ARM"]
RBX_MODES = ["STABILIZE","LAND","RTL","LOITER","GUIDED","RESUME"]
RBX_ACTIONS = ["TAKEOFF"]

RBX_STATE_FUNCTIONS = ["disarm","arm"]
RBX_MODE_FUNCTIONS = ["stabilize","land","rtl","loiter","guided","resume"]
RBX_ACTION_FUNCTIONS = ["takeoff"]


##############################
# RBX Initialization Values
GOTO_TRAN_SPEED_RATIO = 0.5
GOTO_ROT_SPEED_RATIO = 0.5
GOTO_MAX_ERROR_M = 2.0 # Goal reached when all translation move errors are less than this value
GOTO_MAX_ERROR_DEG = 2.0 # Goal reached when all rotation move errors are less than this value
GOTO_STABILIZED_SEC = 1.0 # Window of time that setpoint error values must be good before proceeding
CMD_TIMEOUT_SEC = 20 # Any action that changes 
PRINT_STATUS_1HZ = True # Print current rbx status at 1Hz
IMAGE_INPUT_TOPIC_NAME = "color_2d_image" # Partial or full ROS namespace string 

##############################
# ARDUPILOT Settings
TAKEOFF_MIN_PITCH_DEG = 10
TAKEOFF_ALT_M = 10

#########################################
# ROS NAMESPACE SETUP
#########################################

NEPI_BASE_NAMESPACE = "/nepi/s2x/"

NEPI_RBX_NAME = "ardupilot"
NEPI_RBX_NAMESPACE = NEPI_BASE_NAMESPACE + NEPI_RBX_NAME + "/rbx/"
NEPI_NAVPOSE_SERVICE_NAME = NEPI_BASE_NAMESPACE + "nav_pose_query"

# NEPI RBX Driver Capabilities Publish Topics
NEPI_RBX_CAPABILITIES_RATE_HZ = 1
NEPI_RBX_CAPABILITIES_NAVPOSE_TOPIC = NEPI_RBX_NAMESPACE + "navpose_support"
NEPI_RBX_CAPABILITIES_STATES_TOPIC = NEPI_RBX_NAMESPACE + "state_options"
NEPI_RBX_CAPABILITIES_MODES_TOPIC = NEPI_RBX_NAMESPACE + "mode_options"
NEPI_RBX_CAPABILITIES_ACTIONS_TOPIC = NEPI_RBX_NAMESPACE + "action_options"
# NEPI RBX Driver Status Publish Topics
NEPI_RBX_STATUS_RATE_HZ = 10
NEPI_RBX_STATUS_STATE_TOPIC = NEPI_RBX_NAMESPACE + "state"  # Int to Defined Dictionary RBX_STATES
NEPI_RBX_STATUS_MODE_TOPIC = NEPI_RBX_NAMESPACE + "mode" # Int to Defined Dictionary RBX_MODES
NEPI_RBX_STATUS_BATTERY_TOPIC = NEPI_RBX_NAMESPACE + "battery" # Float Ratio 0-1
NEPI_RBX_STATUS_READY_TOPIC = NEPI_RBX_NAMESPACE + "ready" # Bool, True if goto is complete or no active goto process
NEPI_RBX_STATUS_GOTO_SPEEDS_TOPIC = NEPI_RBX_NAMESPACE + "goto_speeds" # Floats [Translation_Ratio,Rotation_Ratio]
NEPI_RBX_STATUS_GOTO_GOALS_TOPIC = NEPI_RBX_NAMESPACE + "goto_goals" # Floats [Max_Meters,Max_Degrees,Stabilize_Time_Sec]
NEPI_RBX_STATUS_GOTO_ERRORS_TOPIC = NEPI_RBX_NAMESPACE + "goto_errors" # Floats [X_Meters,Y_Meters,Z_Meters,Heading_Degrees,Roll_Degrees,Pitch_Degrees,Yaw_Degrees]
NEPI_RBX_STATUS_CMD_TIMEOUT_TOPIC = NEPI_RBX_NAMESPACE + "cmd_timeout" # Int Seconds  - Any command that changes ready state
NEPI_RBX_STATUS_CMD_SUCCESS_TOPIC = NEPI_RBX_NAMESPACE + "cmd_success" # Bool - Any command that changes ready state
NEPI_RBX_STATUS_IMAGE_SOURCE_TOPIC= NEPI_RBX_NAMESPACE + "status_image_source" # Partial or full ROS namespace string
NEPI_RBX_STATUS_IMAGE_TOPIC= NEPI_RBX_NAMESPACE + "status_image" # Image 
# NEPI RBX Driver NavPose Publish Topics
NEPI_RBX_NAVPOSE_GPS_TOPIC = NEPI_RBX_NAMESPACE + "gps_fix"
NEPI_RBX_NAVPOSE_ODOM_TOPIC = NEPI_RBX_NAMESPACE + "odom"
NEPI_RBX_NAVPOSE_HEADING_TOPIC = NEPI_RBX_NAMESPACE + "heading"
# NEPI RBX Driver Settings Subscriber Topics
NEPI_RBX_SET_STATE_TOPIC = NEPI_RBX_NAMESPACE + "set_state" # Int to Defined Dictionary RBX_STATES
NEPI_RBX_SET_MODE_TOPIC = NEPI_RBX_NAMESPACE + "set_mode"  # Int to Defined Dictionary RBX_MODES
NEPI_RBX_SET_HOME_CURRENT_TOPIC = NEPI_RBX_NAMESPACE + "set_home_current" # Emplty
NEPI_RBX_SET_GOTO_SPEEDS_TOPIC = NEPI_RBX_NAMESPACE + "set_goto_speeds" # Float [Translation_Ratio,Rotation_Ratio]
NEPI_RBX_SET_GOTO_GOALS_TOPIC = NEPI_RBX_NAMESPACE + "set_goto_goals" # Float [Max_Meters,Max_Degrees,Stabilize_Time_Sec]
NEPI_RBX_SET_CMD_TIMEOUT_TOPIC = NEPI_RBX_NAMESPACE + "set_cmd_timeout" # Int Seconds  - Any command that changes ready state
NEPI_RBX_SET_STATUS_IMAGE_TOPIC = NEPI_RBX_NAMESPACE + "set_image_topic" # full ROS namespace 
# NEPI RBX Driver Control Subscriber Topics
NEPI_RBX_GO_ACTION_TOPIC = NEPI_RBX_NAMESPACE + "go_action"  # Int to Defined Dictionary RBX_ACTIONS
NEPI_RBX_GO_HOME_TOPIC = NEPI_RBX_NAMESPACE + "go_home" # Ignored if any active goto processes
NEPI_RBX_GO_STOP_TOPIC = NEPI_RBX_NAMESPACE + "go_stop" # Aborts any active goto processes
NEPI_RBX_GOTO_POSE_TOPIC = NEPI_RBX_NAMESPACE + "goto_pose" # Ignored if any active goto processes
NEPI_RBX_GOTO_POSITION_TOPIC = NEPI_RBX_NAMESPACE + "goto_position" # Ignored if any active goto processes
NEPI_RBX_GOTO_LOCATION_TOPIC = NEPI_RBX_NAMESPACE + "goto_location" # Ignored if any active goto processes

###################################################
MAVLINK_NAMESPACE = NEPI_BASE_NAMESPACE + "mavlink/"
# MAVLINK Subscriber Topics
MAVLINK_STATE_TOPIC = MAVLINK_NAMESPACE + "state"
MAVLINK_BATTERY_TOPIC = MAVLINK_NAMESPACE + "battery"
# MAVLINK Required Services
MAVLINK_SET_HOME_SERVICE = MAVLINK_NAMESPACE + "cmd/set_home"
MAVLINK_SET_MODE_SERVICE = MAVLINK_NAMESPACE + "set_mode"
MAVLINK_ARMING_SERVICE = MAVLINK_NAMESPACE + "cmd/arming"
MAVLINK_TAKEOFF_SERVICE = MAVLINK_NAMESPACE + "cmd/takeoff"
# MAVLINK NavPose Source Topics
MAVLINK_SOURCE_GPS_TOPIC = MAVLINK_NAMESPACE + "global_position/global"
MAVLINK_SOURCE_ODOM_TOPIC = MAVLINK_NAMESPACE + "global_position/local"
MAVLINK_SOURCE_HEADING_TOPIC = MAVLINK_NAMESPACE + "global_position/compass_hdg"
# MAVLINK Setpoint Control Topics
MAVLINK_SETPOINT_ATTITUDE_TOPIC = MAVLINK_NAMESPACE + "setpoint_raw/attitude"
MAVLINK_SETPOINT_POSITION_LOCAL_TOPIC = MAVLINK_NAMESPACE + "setpoint_position/local"
MAVLINK_SETPOINT_LOCATION_GLOBAL_TOPIC = MAVLINK_NAMESPACE + "setpoint_position/global"


#########################################
# Globals
#########################################

rbx_capabilities_navpose_pub = rospy.Publisher(NEPI_RBX_CAPABILITIES_NAVPOSE_TOPIC, UInt8, queue_size=1)
rbx_capabilities_states_pub = rospy.Publisher(NEPI_RBX_CAPABILITIES_STATES_TOPIC, String, queue_size=1)
rbx_capabilities_mode_pub = rospy.Publisher(NEPI_RBX_CAPABILITIES_MODES_TOPIC, String, queue_size=1)
rbx_capabilities_actions_pub = rospy.Publisher(NEPI_RBX_CAPABILITIES_ACTIONS_TOPIC, String, queue_size=1)

rbx_status_state_pub = rospy.Publisher(NEPI_RBX_STATUS_STATE_TOPIC, Int8, queue_size=1)
rbx_status_mode_pub = rospy.Publisher(NEPI_RBX_STATUS_MODE_TOPIC, Int8, queue_size=1)
rbx_status_battery_pub = rospy.Publisher(NEPI_RBX_STATUS_BATTERY_TOPIC, UInt8, queue_size=1)
rbx_status_ready_pub = rospy.Publisher(NEPI_RBX_STATUS_READY_TOPIC, Bool, queue_size=1)
rbx_status_goto_speeds_pub = rospy.Publisher(NEPI_RBX_STATUS_GOTO_SPEEDS_TOPIC, Float64MultiArray, queue_size=1)
rbx_status_goto_goals_pub = rospy.Publisher(NEPI_RBX_STATUS_GOTO_GOALS_TOPIC, Float64MultiArray, queue_size=1)
rbx_status_goto_errors_pub = rospy.Publisher(NEPI_RBX_STATUS_GOTO_ERRORS_TOPIC, Float64MultiArray, queue_size=1)
rbx_status_cmd_timeout_pub = rospy.Publisher(NEPI_RBX_STATUS_CMD_TIMEOUT_TOPIC, UInt32, queue_size=1)
rbx_status_cmd_success_pub = rospy.Publisher(NEPI_RBX_STATUS_CMD_SUCCESS_TOPIC, Bool, queue_size=1)
rbx_status_image_source_pub = rospy.Publisher(NEPI_RBX_STATUS_IMAGE_SOURCE_TOPIC, String, queue_size=10)
rbx_status_image_pub = rospy.Publisher(NEPI_RBX_STATUS_IMAGE_TOPIC, Image, queue_size=10)

rbx_navpose_gps_pub = rospy.Publisher(NEPI_RBX_NAVPOSE_GPS_TOPIC, NavSatFix, queue_size=1)
rbx_navpose_odom_pub = rospy.Publisher(NEPI_RBX_NAVPOSE_ODOM_TOPIC, Odometry, queue_size=1)
rbx_navpose_heading_pub = rospy.Publisher(NEPI_RBX_NAVPOSE_HEADING_TOPIC, Float64, queue_size=1)

update_navpose_interval = 0.1 # 10 Hz
current_heading_deg = None
current_orientation_enu_degs = None
current_orientation_ned_degs = None
current_position_enu_m = None
current_position_ned_m = None
current_location_amsl_geo = None
current_location_wgs84_geo = None
current_geoid_height_m = 0

rbx_state = None
rbx_mode = None
rbx_battery = 0
rbx_ready = True
rbx_goto_speeds = [GOTO_TRAN_SPEED_RATIO,GOTO_ROT_SPEED_RATIO]
rbx_goto_error_goals = [GOTO_MAX_ERROR_M,GOTO_MAX_ERROR_DEG,GOTO_STABILIZED_SEC]
rbx_goto_errors_current = [0,0,0,0,0,0]
rbx_goto_errors_last = [0,0,0,0,0,0]
rbx_cmd_timeout = CMD_TIMEOUT_SEC
rbx_cmd_success_current = True
rbx_cmd_success_last = True
rbx_status_image_source = IMAGE_INPUT_TOPIC_NAME
rbx_status_image_source_last = ""
rbx_status_image_blank = np.zeros((350, 700, 3), dtype = np.uint8) # Empty Black Image
rbx_status_image_base = rbx_status_image_blank

rbx_state_start = None
rbx_state_last = None
rbx_mode_start = None
rbx_mode_last = None
rbx_process_current = "None"
rbx_process_last = "None"

rbx_capabilities_pub_interval = float(1.0)/float(NEPI_RBX_CAPABILITIES_RATE_HZ)
rbx_status_pub_interval = float(1.0)/float(NEPI_RBX_STATUS_RATE_HZ)


mavlink_state = None
                
#########################################
# Methods
#########################################

### System Initialization processes
def initialize_actions():
  global rbx_state
  global rbx_mode
  global rbx_battery
  global rbx_state_start
  global rbx_mode_start
  global rbx_state_last
  global rbx_mode_last
  print("*******************************")  
  print("Starting Initialization Process")
  # Wait for MAVLink State topic to publish then subscribe
  print("Waiting for topic: " + MAVLINK_STATE_TOPIC)
  wait_for_topic(MAVLINK_STATE_TOPIC)
  print("Starting state scubscriber callback")
  rospy.Subscriber(MAVLINK_STATE_TOPIC, State, get_state_callback)
  while rbx_state is None and not rospy.is_shutdown():
    print("Waiting for rbx state status to set")
    time.sleep(0.1)
  while rbx_mode is None and not rospy.is_shutdown():
    print("Waiting for rbx mode status to set")
    time.sleep(0.1)
  print("Starting State: " + RBX_STATES[rbx_state])
  print("Starting Mode: " + RBX_MODES[rbx_mode])
  rbx_state_start = rbx_state
  rbx_mode_start = rbx_mode
  rbx_state_last = rbx_state
  rbx_mode_last = rbx_mode
  # Subscribe to MAVLink Battery topic
  rospy.Subscriber(MAVLINK_BATTERY_TOPIC, BatteryState, get_mavlink_battery_callback)
  # Start NavPose Data Updater
  rospy.Timer(rospy.Duration(update_navpose_interval), update_current_navpose_callback)
  ##############################
  print("Starting RBX driver publisher and subscriber topics")
  # Start RBX Capabilities and Status Publishers
  rospy.Timer(rospy.Duration(rbx_capabilities_pub_interval), rbx_capabilities_pub_callback)
  rospy.Timer(rospy.Duration(rbx_status_pub_interval), rbx_status_pub_callback)
  ### Start RBX NavPose Publishers
  rospy.Subscriber(MAVLINK_SOURCE_GPS_TOPIC, NavSatFix, rbx_gps_topic_callback)
  rospy.Subscriber(MAVLINK_SOURCE_ODOM_TOPIC, Odometry, rbx_odom_topic_callback)
  rospy.Subscriber(MAVLINK_SOURCE_HEADING_TOPIC, Float64, rbx_heading_topic_callback)
  ### Start RBX Settings Subscribe Topics
  rospy.Subscriber(NEPI_RBX_SET_STATE_TOPIC, UInt8, rbx_set_state_callback)
  rospy.Subscriber(NEPI_RBX_SET_MODE_TOPIC, UInt8, rbx_set_mode_callback)
  rospy.Subscriber(NEPI_RBX_SET_HOME_CURRENT_TOPIC, Empty, rbx_set_home_current_callback)
  rospy.Subscriber(NEPI_RBX_SET_GOTO_SPEEDS_TOPIC, Float64MultiArray, rbx_set_goto_speeds_callback)
  rospy.Subscriber(NEPI_RBX_SET_GOTO_GOALS_TOPIC, Float64MultiArray, rbx_set_goto_goals_callback)
  rospy.Subscriber(NEPI_RBX_SET_CMD_TIMEOUT_TOPIC, UInt32, rbx_set_cmd_timeout_callback)
  ### Start RBX Control Subscribe Topics
  rospy.Subscriber(NEPI_RBX_GO_ACTION_TOPIC, UInt8, rbx_go_action_callback)
  rospy.Subscriber(NEPI_RBX_GO_HOME_TOPIC, Empty, rbx_go_home_callback)
  rospy.Subscriber(NEPI_RBX_GO_STOP_TOPIC, Empty, rbx_go_stop_callback)
  rospy.Subscriber(NEPI_RBX_GOTO_POSE_TOPIC, Float64MultiArray, rbx_goto_pose_callback)
  rospy.Subscriber(NEPI_RBX_GOTO_POSITION_TOPIC, Float64MultiArray, rbx_goto_position_callback)
  rospy.Subscriber(NEPI_RBX_GOTO_LOCATION_TOPIC, Float64MultiArray, rbx_goto_location_callback)
  # Start Print Callback if Enabled
  if PRINT_STATUS_1HZ:
    rospy.Timer(rospy.Duration(1.0), print_rbx_status_callback)
  #####
  print("Initialization Complete")


##############################
### Status Base Image Source Subscriber
def update_status_base_image_callback(img_msg):
  global rbx_status_image_base
  #Convert image from ros to cv2
  bridge = CvBridge()
  rbx_status_image_base = bridge.imgmsg_to_cv2(img_msg, "bgr8")


##############################
# RBX Capabilities Topic Publishers
### Callback to publish RBX capabilite option lists
def rbx_capabilities_pub_callback(timer):
  global rbx_capabilities_states_pub
  global rbx_capabilities_mode_pub
  if not rospy.is_shutdown():
    rbx_capabilities_navpose_pub.publish(data=RBX_NAVPOSE)
    rbx_capabilities_states_pub.publish(str(RBX_STATES))
    rbx_capabilities_mode_pub.publish(str(RBX_MODES))
    rbx_capabilities_actions_pub.publish(str(RBX_ACTIONS))
  

##############################
# RBX Status Topic Publishers
### Callback for rbx status publisher
def rbx_status_pub_callback(timer):
  global rbx_state
  global rbx_mode
  global rbx_battery
  global rbx_ready
  global rbx_goto_speeds
  global rbx_goto_error_goals
  global rbx_goto_errors_current
  global rbx_goto_errors_last
  global rbx_cmd_timeout
  global rbx_cmd_success_current
  global rbx_cmd_success_last
  global rbx_process_current
  global rbx_process_last
  global rbx_status_image_source
  global rbx_status_image_source_last
  global rbx_status_image_base
  global rbx_status_state_pub
  global rbx_status_mode_pub
  global rbx_status_battery_pub
  global rbx_status_ready_pub
  global rbx_status_goto_speeds_pub
  global rbx_status_goto_goals_pub
  global rbx_status_goto_errors_pub
  global rbx_status_cmd_timeout_pub
  global rbx_status_cmd_success_pub
  global rbx_status_image_source_pub
  global rbx_status_image_pub
  if not rospy.is_shutdown():
    rbx_status_state_pub.publish(rbx_state)
    rbx_status_mode_pub.publish(rbx_mode)
    rbx_status_battery_pub.publish(rbx_battery)
    rbx_status_ready_pub.publish(rbx_ready)
    rbx_status_goto_speeds_pub.publish(data=rbx_goto_speeds)
    rbx_status_goto_goals_pub.publish(data=rbx_goto_error_goals)
    rbx_status_goto_errors_pub.publish(data=rbx_goto_errors_current)
    rbx_status_cmd_timeout_pub.publish(data=rbx_cmd_timeout)
    rbx_status_cmd_success_pub.publish(data=rbx_cmd_success_last)
    rbx_status_image_source_pub.publish(data=rbx_status_image_source)
    ### Update Status Image and Publish
    rbx_status_image = rbx_status_image_base # Initialize status image
    box_x = 10
    box_y = 10
    box_w = 350
    box_h = 450
    # Add status box overlay
    cv2.rectangle(rbx_status_image, (box_x, box_y), (box_w, box_h), (255, 255, 255), -1)
    # Create Status Info Text List
    text_list = ["RBX Status"]
    if rbx_battery < 0.1:
      battery_string = "No Reading"
    else:
      battery_string = '%.2f' % rbx_battery
    text_list.append("Battery: " + battery_string)
    text_list.append("State: " + RBX_STATES[rbx_state])
    text_list.append("Mode Current: " + RBX_MODES[rbx_mode])
    text_list.append("Mode Last: " + RBX_MODES[rbx_mode_last])
    text_list.append("Ready: " + str(rbx_ready))
    text_list.append("")
    text_list.append("Current Process: " + rbx_process_current)
    text_list.append(" X,Y,Z Errors Meters: ")
    text_list.append(" " + '%.2f' % rbx_goto_errors_current[0] + "," + '%.2f' % rbx_goto_errors_current[1] + "," + '%.2f' % rbx_goto_errors_current[2])
    text_list.append(" R,P,Y Errors Degrees: ")
    text_list.append(" " + '%.2f' % rbx_goto_errors_current[3] + "," + '%.2f' % rbx_goto_errors_current[4] + "," + '%.2f' % rbx_goto_errors_current[5])
    text_list.append("")
    text_list.append("Last Process: " + rbx_process_last)
    text_list.append(" Success: " + str(rbx_cmd_success_last))
    text_list.append(" X,Y,Z Errors Meters: ")
    text_list.append(" " + '%.2f' % rbx_goto_errors_last[0] + "," + '%.2f' % rbx_goto_errors_last[1] + "," + '%.2f' % rbx_goto_errors_last[2])
    text_list.append(" R,P,Y Errors Degrees: ")
    text_list.append(" " + '%.2f' % rbx_goto_errors_last[3] + "," + '%.2f' % rbx_goto_errors_last[4] + "," + '%.2f' % rbx_goto_errors_last[5])
    text_list.append("")
    # Overlay Status Text List
    x=box_x+10 
    y=box_y+20
    for text in text_list:
      status_text_overlay(rbx_status_image,text,x, y)
      y = y + 20
    # Create ROS Image message
    bridge = CvBridge()
    img_out_msg = bridge.cv2_to_imgmsg(rbx_status_image,"bgr8")#desired_encoding='passthrough')
    # Publish new image to ros
    if not rospy.is_shutdown():
      rbx_status_image_pub.publish(img_out_msg)
      # You can view the enhanced_2D_image topic at 
      # //192.168.179.103:9091/ in a connected web browser
  ### Update image source topic and subscriber if changed from last time.
  if rbx_status_image_source != rbx_status_image_source_last:
    #  If currently set, first unregister current image topic
    if rbx_status_image_source_last != "": 
      rbx_status_image_pub.unregister()
      time.sleep(1)
    # Try to find and subscribe to new image source topic
    print("Looking for topic: " + rbx_status_image_source)
    image_topic = find_topic(rbx_status_image_source)
    if image_topic != "":  # If image topic exists subscribe
      rospy.Subscriber(image_topic, Image, update_status_base_image_callback, queue_size = 1)
      rbx_status_image_source_last = rbx_status_image_source
  if rbx_status_image_source == "":
    rbx_status_image_base = rbx_status_image_blank # Set to blank image if source topic is cleared.
    
### Status Text Overlay Function
def status_text_overlay(cv_image,status_text,x,y):
    font                   = cv2.FONT_HERSHEY_SIMPLEX
    bottomLeftCornerOfText = (x,y)
    fontScale              = 0.5
    fontColor              = (0, 0, 0)
    thickness              = 1
    lineType               = 1
    cv2.putText(cv_image,status_text, 
      bottomLeftCornerOfText, 
      font, 
      fontScale,
      fontColor,
      thickness,
      lineType)
  


### Callback to print the current rbx status at 1Hz
def print_rbx_status_callback(timer):
  global rbx_state
  global rbx_mode
  global rbx_mode_last
  global rbx_battery
  global rbx_ready
  global rbx_goto_speeds
  global rbx_goto_error_goals
  global rbx_goto_errors_current
  global rbx_goto_errors_last
  global rbx_cmd_timeout
  global rbx_cmd_success_current
  global rbx_cmd_success_last
  global rbx_process_current
  global rbx_process_last
  print("*******************************")
  # Create Status Info Text List
  text_list = ["RBX Status"]
  if rbx_battery < 0.1:
    battery_string = "No Reading"
  else:
    battery_string = '%.2f' % rbx_battery
  text_list.append("Battery: " + battery_string)
  text_list.append("State: " + RBX_STATES[rbx_state])
  text_list.append("Mode Current: " + RBX_MODES[rbx_mode])
  text_list.append("Mode Last: " + RBX_MODES[rbx_mode_last])
  text_list.append("Ready: " + str(rbx_ready))
  text_list.append("")
  text_list.append("Current Process: " + rbx_process_current)
  text_list.append(" X,Y,Z Errors Meters: ")
  text_list.append(" " + '%.2f' % rbx_goto_errors_current[0] + "," + '%.2f' % rbx_goto_errors_current[1] + "," + '%.2f' % rbx_goto_errors_current[2])
  text_list.append(" R,P,Y Errors Degrees: ")
  text_list.append(" " + '%.2f' % rbx_goto_errors_current[3] + "," + '%.2f' % rbx_goto_errors_current[4] + "," + '%.2f' % rbx_goto_errors_current[5])
  text_list.append("")
  text_list.append("Last Process: " + rbx_process_last)
  text_list.append(" Success: " + str(rbx_cmd_success_last))
  text_list.append(" X,Y,Z Errors Meters: ")
  text_list.append(" " + '%.2f' % rbx_goto_errors_last[0] + "," + '%.2f' % rbx_goto_errors_last[1] + "," + '%.2f' % rbx_goto_errors_last[2])
  text_list.append(" R,P,Y Errors Degrees: ")
  text_list.append(" " + '%.2f' % rbx_goto_errors_last[3] + "," + '%.2f' % rbx_goto_errors_last[4] + "," + '%.2f' % rbx_goto_errors_last[5])
  text_list.append("")
  # Print Status Text List
  for text in text_list:
    print(text)

##############################
# RBX NavPose Topic Publishers
### Callback to publish RBX navpose gps topic
def rbx_gps_topic_callback(navsatfix_msg):
  global rbx_navpose_gps_pub
  global current_geoid_height_m
  #Fix Mavros Altitude Error
  altitude_amsl = navsatfix_msg.altitude
  navsatfix_msg.altitude = altitude_amsl - current_geoid_height_m
  if not rospy.is_shutdown():
    rbx_navpose_gps_pub.publish(navsatfix_msg)
  
### Callback to publish RBX odom topic
def rbx_odom_topic_callback(odom_msg):
  global rbx_navpose_odom_pub
  if not rospy.is_shutdown():
    rbx_navpose_odom_pub.publish(odom_msg)

### Callback to publish RBX heading topic
def rbx_heading_topic_callback(heading_msg):
  global rbx_navpose_heading_pub
  if not rospy.is_shutdown():
    rbx_navpose_heading_pub.publish(heading_msg)

##############################
# RBX Settings Topic Callbacks

# ToDo: Create a custom RBX status message
### Callback to set state
def rbx_set_state_callback(state_msg):
  print("*******************************")
  print("Received set state message")
  print(state_msg)
  state_val = state_msg.data
  rbx_set_state(state_val)

### Function to set state
def rbx_set_state(new_state_ind):
  global rbx_state
  global rbx_state_last
  if new_state_ind < 0 or new_state_ind > (len(RBX_STATE_FUNCTIONS)-1):
    print("No matching rbx state found")
  else:
    rbx_state_last = rbx_state
    print("Waiting for rbx state " + RBX_STATES[new_state_ind] + " to set")
    print("Current rbx state is " + RBX_STATES[rbx_state])
    set_state_function = globals()[RBX_STATE_FUNCTIONS[new_state_ind]]
    set_state_function()
    rbx_state = new_state_ind

### Callback to set mode
def rbx_set_mode_callback(mode_msg):
  print("*******************************")
  print("Received set mode message")
  print(mode_msg)
  mode_val = mode_msg.data
  rbx_set_mode(mode_val)

### Function to set mode
def rbx_set_mode(new_mode_ind):
  global rbx_mode
  global rbx_mode_last
  time.sleep(1)
  if new_mode_ind < 0 or new_mode_ind > (len(RBX_MODE_FUNCTIONS)-1):
    print("No matching rbx mode found")
  else:
    if RBX_MODES[new_mode_ind] != "RESUME":
      rbx_mode_last = rbx_mode # Don't update last on resume
    print("Setting rbx mode to : " + RBX_MODES[new_mode_ind])
    print("Calling rbx mode function: " + RBX_MODE_FUNCTIONS[new_mode_ind])
    set_mode_function = globals()[RBX_MODE_FUNCTIONS[new_mode_ind]]
    success = set_mode_function()
    rbx_mode = new_mode_ind

### Callback to set home
def rbx_set_home_current_callback(set_home_msg):
  print("*******************************")
  print("Received set home message")
  print(set_home_msg)
  sethome_current()
  
### Callback to start rbx set goto speeds process
def rbx_set_goto_speeds_callback(goto_speeds_msg):
  global rbx_goto_speeds
  print("*******************************")
  print("Received set speeds message")
  print(goto_speeds_msg)
  goto_speeds_list=list(goto_speeds_msg.data)
  if len(goto_speeds_list) != 2:
    print("Ignoring this Request")
    print("Messge is wrong length. Should be float list of size 2")
  else:    
    rbx_goto_speeds = goto_speeds_list

### Callback to start rbx set goto goals process
def rbx_set_goto_goals_callback(goto_goals_msg):
  global rbx_goto_error_goals
  print("*******************************")
  print("Received set goals message")
  print(goto_goals_msg)
  goto_goals_list=list(goto_goals_msg.data)
  if len(goto_goals_list) != 3:
    print("Ignoring this Request")
    print("Messge is wrong length. Should be float list of size 3")
  else:
    rbx_goto_error_goals = goto_goals_list

### Callback to set cmd timeout
def rbx_set_cmd_timeout_callback(cmd_timeout_msg):
  global rbx_cmd_timeout
  print("*******************************")
  print("Received set timeout message")
  print(cmd_timeout_msg)
  rbx_cmd_timeout=cmd_timeout_msg.data


##############################
# RBX Control Topic Callbacks

### Callback to execute action
def rbx_go_action_callback(action_msg):
  global rbx_ready
  global rbx_cmd_timeout
  global rbx_cmd_success_current
  global rbx_cmd_success_last
  global rbx_process_current
  global rbx_process_last
  global rbx_goto_errors_current
  print("*******************************")
  print("Received go action message")
  print(action_msg)
  rbx_cmd_success_current = False
  rbx_goto_errors_current = [0,0,0,0,0,0]
  action_ind = action_msg.data
  if action_ind < 0 or action_ind > (len(RBX_ACTION_FUNCTIONS)-1):
    print("No matching rbx action found")
  else:
    rbx_process_current = RBX_ACTIONS[action_ind]
    rbx_ready = False
    print("Starting action: " + rbx_process_current)
    set_action_function = globals()[RBX_ACTION_FUNCTIONS[action_ind]]
    rbx_cmd_success_current = set_action_function(rbx_cmd_timeout)
    print("Finished action: " + rbx_process_current)
    time.sleep(1)
    rbx_ready = True
  rbx_process_last = rbx_process_current
  rbx_process_current = "None"
  rbx_cmd_success_last = rbx_cmd_success_current

### Callback to start rbx go home
def rbx_go_home_callback(home_msg):
  global rbx_ready
  global rbx_cmd_timeout
  global rbx_cmd_success_current
  global rbx_cmd_success_last
  global rbx_process_current
  global rbx_process_last
  rbx_process_current = "Go Home"
  rbx_cmd_success_current = False
  rbx_ready = False
  print("*******************************")
  print("Received go home message")
  print(home_msg)
  rtl()
  rbx_cmd_success_current = True
  time.sleep(1)
  rbx_ready = True
  rbx_process_last = rbx_process_current
  rbx_process_current = "None"
  rbx_cmd_success_last = rbx_cmd_success_current

### Callback to start rbx stop
def rbx_go_stop_callback(stop_msg):
  global rbx_ready
  global rbx_cmd_timeout
  global rbx_cmd_success_current
  global rbx_cmd_success_last
  global rbx_process_current
  global rbx_process_last
  global rbx_goto_errors_current
  rbx_process_current = "Stop"
  rbx_cmd_success_current = False
  rbx_ready = False
  rbx_goto_errors_current = [0,0,0,0,0,0]
  print("*******************************")
  print("Received go stop message")
  print(stop_msg)
  loiter()
  rbx_cmd_success_current = True
  time.sleep(1)
  rbx_ready = True
  rbx_process_last = rbx_process_current
  rbx_process_current = "None"
  rbx_cmd_success_last = rbx_cmd_success_current

### Callback to start rbx goto pose process
def rbx_goto_pose_callback(pose_cmd_msg):
  global rbx_ready
  global rbx_cmd_timeout
  global rbx_cmd_success_current
  global rbx_cmd_success_last
  global rbx_process_current
  global rbx_process_last
  global rbx_goto_errors_current
  rbx_process_current = "GoTo Pose"
  rbx_cmd_success_current = False
  rbx_goto_errors_current = [0,0,0,0,0,0]
  print("*******************************")
  print("Recieved GoTo Pose Message")
  print("")
  print(pose_cmd_msg)
  setpoint_data=list(pose_cmd_msg.data)
  if len(setpoint_data) != 3:
    print("Ignoring this Request")
    print("Messge is wrong length. Should be float list of size 3")
    print("[Roll,Pitch,Yaw]")
  else:
    if rbx_ready is False:
      print("Another GoTo Command Process is Active")
      print("Ignoring this Request")
    else:
      rbx_ready = False
      rbx_cmd_success_current = setpoint_attitude_ned(setpoint_data,rbx_cmd_timeout)
      rbx_ready = True
  rbx_process_last = rbx_process_current
  rbx_process_current = "None"
  rbx_cmd_success_last = rbx_cmd_success_current


### Callback to start rbx goto position process
def rbx_goto_position_callback(position_cmd_msg):
  global rbx_ready
  global rbx_cmd_timeout
  global rbx_cmd_success_current
  global rbx_cmd_success_last
  global rbx_process_current
  global rbx_process_last
  global rbx_goto_errors_current
  rbx_process_current = "GoTo Position"
  rbx_cmd_success_current = False
  rbx_goto_errors_current = [0,0,0,0,0,0]
  print("*******************************")
  print("Recieved GoTo Position Command Message")
  print("")
  print(position_cmd_msg)
  setpoint_data=list(position_cmd_msg.data)
  if len(setpoint_data) != 4:
    print("Ignoring this Request")
    print("Messge is wrong length. Should be float list of size 4")
    print("[X,Y,Z,Yaw]")
  else:    
    if rbx_ready is False:
      print("Another GoTo Command Process is Active")
      print("Ignoring this Request")
    else:
      rbx_ready = False
      rbx_cmd_success_current = setpoint_position_local_body(setpoint_data,rbx_cmd_timeout)
      rbx_ready = True
  rbx_process_last = rbx_process_current
  rbx_process_current = "None"
  rbx_cmd_success_last = rbx_cmd_success_current


### Callback to start rbx goto location subscriber
def rbx_goto_location_callback(location_cmd_msg):
  global rbx_ready
  global rbx_cmd_timeout
  global rbx_cmd_success_current
  global rbx_cmd_success_last
  global rbx_process_current
  global rbx_process_last
  global rbx_goto_errors_current
  rbx_process_current = "GoTo Location"
  rbx_cmd_success_current = False
  rbx_goto_errors_current = [0,0,0,0,0,0]
  print("*******************************")
  print("Recieved GoTo Location Message")
  print("")
  print(location_cmd_msg)
  setpoint_data=list(location_cmd_msg.data)
  if len(setpoint_data) != 4:
    print("Ignoring this Request")
    print("Messge is wrong length. Should be float list of size 4")
    print("[Lat,Long,Alt,Yaw]")
  else:
    if rbx_ready is False:
      print("Another GoTo Command Process is Active")
      print("Ignoring this Request")
    else:
      rbx_ready = False
      rbx_cmd_success_current = setpoint_location_global_wgs84(setpoint_data,rbx_cmd_timeout)
      rbx_ready = True
  rbx_process_last = rbx_process_current
  rbx_process_current = "None"
  rbx_cmd_success_last = rbx_cmd_success_current


#######################
# Mavlink Interface Methods

### Callback to get current state message
def get_state_callback(mavlink_state_msg):
  global rbx_state
  global rbx_mode
  global mavlink_state
  mavlink_state = mavlink_state_msg
  # Update rbx state value
  arm_val = mavlink_state_msg.armed
  if arm_val == True:
    rbx_state=1
  else:
    rbx_state=0
  # Update rbx mode value
  mode_val = mavlink_state_msg.mode
  mode_ind=-1
  for ind, mode in enumerate(RBX_MODES):
    if mode == mode_val:
      mode_ind=ind
  rbx_mode=mode_ind   


### Callback to get current mavlink battery message
def get_mavlink_battery_callback(battery_msg):
  global rbx_battery
  rbx_battery = battery_msg.percentage


### Function to set mavlink armed state
def set_mavlink_arm_state(arm_value):
  global mavlink_state
  global rbx_ready
  global arming_client
  arm_cmd = CommandBoolRequest()
  arm_cmd.value = arm_value
  print("Updating armed")
  print(arm_value)
  rbx_ready = False
  time.sleep(1) # Give time for other process to see busy
  arming_client = rospy.ServiceProxy(MAVLINK_ARMING_SERVICE, CommandBool)
  while mavlink_state.armed != arm_value and not rospy.is_shutdown():
    time.sleep(.25)
    arming_client.call(arm_cmd)
    print("Waiting for armed value to set")
    print("Set Value: " + str(arm_value))
    print("Cur Value: " + str(mavlink_state.armed))
  rbx_ready = True

### Function to set mavlink mode
def set_mavlink_mode(mode_new):
  global mavlink_state
  global rbx_ready
  global mode_client
  new_mode = SetModeRequest()
  new_mode.custom_mode = mode_new
  print("Updating mode")
  print(mode_new)
  mode_client = rospy.ServiceProxy(MAVLINK_SET_MODE_SERVICE, SetMode)
  rbx_ready = False
  time.sleep(1) # Give time for other process to see busy
  while mavlink_state.mode != mode_new and not rospy.is_shutdown():
    time.sleep(.25)
    mode_client.call(new_mode)
    print("Waiting for mode to set")
    print("Set Value: " + mode_new)
    print("Cur Value: " + str(mavlink_state.mode))
  rbx_ready = True






### Function to set and check setpoint attitude NED command
###################################################
# Input is [ROLL_NED_DEG, PITCH_NED_DEG, YEW_NED_DEGREES]
# Converted to ENU befor sending message
###################################################
def setpoint_attitude_ned(setpoint_attitude,timeout_sec=CMD_TIMEOUT_SEC):
  # setpoint_attitude is [ROLL_NED_DEG, PITCH_NED_DEG, YEW_NED_DEGREES]
  # Use value -999 to use current value
  global current_orientation_enu_degs
  global current_orientation_ned_degs
  global current_heading_degs
  global rbx_goto_error_goals
  global rbx_goto_errors_current
  global rbx_goto_errors_last
  cmd_success = True
  rbx_goto_errors_current = [0,0,0,0,0,0,0]
  print("Starting Setpoint Attitude Create-Send-Check Process")
  ##############################################
  # Capture Current NavPose Data
  ##############################################
  start_orientation_ned_degs=list(current_orientation_ned_degs)
  print('')
  print("Attitude Current NED Degrees")
  print(" Roll, Pitch, Yaw")
  print(["%.2f" % start_orientation_ned_degs[0],"%.2f" % start_orientation_ned_degs[1],"%.2f" % start_orientation_ned_degs[2]])
  ##############################################
  # Condition Inputs
  ##############################################
  input_attitude_ned_degs = list(setpoint_attitude)
  print('')
  print("Attitude Input NED Degrees")
  print(" Roll, Pitch, Yaw")
  print(["%.2f" % input_attitude_ned_degs[0],"%.2f" % input_attitude_ned_degs[1],"%.2f" % input_attitude_ned_degs[2]])
  # Set new attitude in degs NED
  new_attitude_ned_degs=list(start_orientation_ned_degs) # Initialize with start values
  for ind in range(3): # Overwrite current with new if set and valid
    if setpoint_attitude[ind] != -999:
      new_attitude_ned_degs[ind]=setpoint_attitude[ind]
    # Condition to +-180 deg
    if new_attitude_ned_degs[ind] > 180:
      new_attitude_ned_degs[ind] = new_attitude_ned_degs[ind] - 360
  print('')
  print("Attitude Input Conditioned NED Degrees")
  print(" Roll, Pitch, Yaw")
  print(["%.2f" % new_attitude_ned_degs[0],"%.2f" % new_attitude_ned_degs[1],"%.2f" % new_attitude_ned_degs[2]])
  ##############################################
  # Convert NED attitude to Pose
  ##############################################
  # Convert to ROS ENU attitude degs and create ENU quaternion setpoint attitude goal
  yaw_enu_deg = convert_yaw_ned2enu(new_attitude_ned_degs[2])
  new_attitude_enu_degs = [new_attitude_ned_degs[0],new_attitude_ned_degs[1],yaw_enu_deg]
  print('')
  print("Attitude Goal ENU Degrees")
  print(" Roll, Pitch, Yaw")
  print(["%.2f" % new_attitude_enu_degs[0],"%.2f" % new_attitude_enu_degs[1],"%.2f" % new_attitude_enu_degs[2]])
  new_attitude_enu_quat = convert_rpy2quat(new_attitude_enu_degs)
  new_orientation_enu_quat = Quaternion()
  new_orientation_enu_quat.x = new_attitude_enu_quat[0]
  new_orientation_enu_quat.y = new_attitude_enu_quat[1]
  new_orientation_enu_quat.z = new_attitude_enu_quat[2]
  new_orientation_enu_quat.w = new_attitude_enu_quat[3]
  # Set other setpoint attitude message values
  body_rate = Vector3()
  body_rate.x = 0
  body_rate.y = 0
  body_rate.z = 0
  type_mask = 1|2|4
  thrust_ratio = 0
  ##############################################
  # Create Setpoint Attitude Message
  ##############################################
  print('')
  print("Creating Message")
  attitude_target_msg = AttitudeTarget()
  attitude_target_msg.orientation = new_orientation_enu_quat
  attitude_target_msg.body_rate = body_rate
  attitude_target_msg.type_mask = type_mask
  attitude_target_msg.thrust = thrust_ratio
  print('')
  print("Setpoint Goal Attitude ENU Message")
  print(attitude_target_msg)
  ##############################################
  ## Send Setpoint Message and Check for Success
  ##############################################
  setpoint_attitude_pub = rospy.Publisher(MAVLINK_SETPOINT_ATTITUDE_TOPIC, AttitudeTarget, queue_size=1)
  time.sleep(1) # VERY IMPORTANT - Sleep a bit between declaring a publisher and using it  
  print('')
  print("Sending Setpoint Attitude Command at 50 Hz and")
  print("Waiting for Attitude Setpoint to complete")
  setpoint_attitude_reached = False
  stabilize_timer=0
  attitude_errors = [] # Initialize running list of errors
  timeout_timer = 0 # Initialize timeout timer
  while setpoint_attitude_reached is False and not rospy.is_shutdown():  # Wait for setpoint goal to be set
    if timeout_timer > timeout_sec:
      print("Setpoint cmd timed out")
      cmd_success = False
      break
    time2sleep = 0.02
    time.sleep(time2sleep) # update setpoint position at 50 Hz
    stabilize_timer=stabilize_timer+time2sleep # Increment print message timer
    timeout_timer = timeout_timer+time2sleep
    setpoint_attitude_pub.publish(attitude_target_msg) # Publish Setpoint
    # Calculate setpoint attitude errors
    cur_attitude_ned_degs = [current_orientation_ned_degs[0],current_orientation_ned_degs[1],current_orientation_ned_degs[2]]
    attitude_errors_degs = np.array(new_attitude_ned_degs) - np.array(cur_attitude_ned_degs)
    for ind in range(3):
      if input_attitude_ned_degs[ind] == -999.0: # Ignore error check if set to current
        attitude_errors_degs[ind]=0.0
    max_attutude_error_deg = max(abs(attitude_errors_degs))
    # Check for setpoint position local point goal
    if  setpoint_attitude_reached is False:
      if stabilize_timer > rbx_goto_error_goals[2]:
        print(stabilize_timer)
        print(rbx_goto_error_goals[2])
        max_attitude_errors = max(attitude_errors) # Get max from error window
        attitude_errors = [max_attutude_error_deg] # reset running list of errors
        # Print some information
        print('')
        print("Current Attitude NED Degrees")
        print(" Roll, Pitch, Yaw")
        print(["%.2f" % current_orientation_ned_degs[0],"%.2f" % current_orientation_ned_degs[1],"%.2f" % current_orientation_ned_degs[2]])
        print('')
        print("Current Goal NED Degrees")
        print(["%.2f" % new_attitude_ned_degs[0],"%.2f" % new_attitude_ned_degs[1],"%.2f" % new_attitude_ned_degs[2]])
        print('')
        print("Current Attitude Errors")
        print(["%.3f" % attitude_errors_degs[0],"%.3f" % attitude_errors_degs[1],"%.3f" % attitude_errors_degs[2]])
        print("Max Error from Stabilized Check Window Meters")
        print(["%.2f" % max_attitude_errors])
        if max_attitude_errors < rbx_goto_error_goals[1]:
          print('')
          print("Attitude Setpoint Reached")
          setpoint_attitude_reached = True
      else:
        attitude_errors.append(max_attutude_error_deg) # append last
    # Reset print timer if past
    if stabilize_timer > GOTO_STABILIZED_SEC:
      stabilize_timer=0 # Reset print timer
    rbx_goto_errors_current = [0,0,0,0,attitude_errors_degs[0],attitude_errors_degs[1],attitude_errors_degs[2]] 
  if cmd_success:
    print("************************")
    print("Setpoint Reached")
  rbx_goto_errors_current = [0,0,0,0,0,0]
  rbx_goto_errors_last = [0,0,0,0,attitude_errors_degs[0],attitude_errors_degs[1],attitude_errors_degs[2]]
  return cmd_success
  


### Function to set and check setpoint position local body command
###################################################
# Input is [X_BODY_METERS, Y_BODY_METERS, Z_BODY_METERS, YEW_BODY_DEGREES]
# Converted to Local ENU Frame before sending
# Local Body Position Setpoint Function use these body relative x,y,z,yaw conventions
# x+ axis is forward
# y+ axis is right
# z+ axis is down
# Only yaw orientation updated
# yaw+ clockwise, yaw- counter clockwise from x axis (0 degrees faces x+ and rotates positive using right hand rule around z+ axis down)
#####################################################
def setpoint_position_local_body(setpoint_position,timeout_sec=CMD_TIMEOUT_SEC):
  # setpoint_position is [X_BODY_METERS, Y_BODY_METERS, Z_BODY_METERS, YEW_BODY_DEGREES]
  # use value 0 for no change
  global current_orientation_ned_degs
  global current_position_ned_m
  global current_location_wgs84_geo
  global current_heading_deg
  global setpoint_position_local_pub
  global rbx_goto_error_goals
  global rbx_goto_errors_current
  global rbx_goto_errors_last
  cmd_success = True
  rbx_goto_errors_current = [0,0,0,0,0,0,0]
  print('')
  print("Starting Setpoint Position Local Create-Send-Check Process")
  ##############################################
  # Capture Current NavPose Data
  ##############################################
  start_geopoint_wgs84 = list(current_location_wgs84_geo)
  print('')
  print("Start Location WSG84 geopoint")
  print(" Lat, Long, Alt")
  print(["%.2f" % start_geopoint_wgs84[0],"%.2f" % start_geopoint_wgs84[1],"%.2f" % start_geopoint_wgs84[2]])
  start_position_ned_m = list(current_position_ned_m)
  print('')
  print("Start Position NED degs")
  print(" X, Y, Z")
  print(["%.2f" % start_position_ned_m[0],"%.2f" % start_position_ned_m[1],"%.2f" % start_position_ned_m[2]])   
  start_orientation_ned_degs=list(current_orientation_ned_degs)
  print('')
  print("Start Orientation NED degs")
  print(" Roll, Pitch, Yaw")
  print(["%.2f" % start_orientation_ned_degs[0],"%.2f" % start_orientation_ned_degs[1],"%.2f" % start_orientation_ned_degs[2]])
  print('')
  start_yaw_ned_deg = start_orientation_ned_degs[2]
  print('')
  print("Start Yaw NED degs")
  print(start_yaw_ned_deg) 
  start_heading_deg=current_heading_deg
  print('')
  print("Start Heading degs")
  print(start_heading_deg)   
  ##############################################
  # Condition Body Input Data
  ##############################################
  # Condition Point Input
  input_point_body_m=setpoint_position[0:3]
  print('')
  print("Point Input Body Meters")
  print(" X, Y, Z")
  print(["%.2f" % input_point_body_m[0],"%.2f" % input_point_body_m[1],"%.2f" % input_point_body_m[2]])
  new_point_body_m=list(input_point_body_m) # No conditioning required
  print('')
  print("Point Conditioned Body Meters")
  print(" X, Y, Z")
  print(["%.2f" % new_point_body_m[0],"%.2f" % new_point_body_m[1],"%.2f" % new_point_body_m[2]])
  # Condition Orienation Input
  input_yaw_body_deg = setpoint_position[3]
  print('')
  print("Yaw Input Body Degrees")
  print(["%.2f" % input_yaw_body_deg])
  new_yaw_body_deg = input_yaw_body_deg
  # Condition to +-180 deg
  if new_yaw_body_deg > 180:
    new_yaw_body_deg = new_yaw_body_deg - 360
  print('')
  print("Yaw Input Conditioned Body Degrees")
  print(["%.2f" % new_yaw_body_deg])      
  ##############################################
  # Convert Body Data to NED Data
  ##############################################
  # Set new yaw orientation in NED degrees
  offset_ned_m = convert_point_body2ned(new_point_body_m,start_yaw_ned_deg)
  print('')
  print("Point Goal Offsets NED Meters")
  print(" X, Y, Z")
  print(["%.2f" % offset_ned_m[0],"%.2f" % offset_ned_m[1],"%.2f" % offset_ned_m[2]])
  new_x_ned_m = start_position_ned_m[0] + offset_ned_m[0]
  new_y_ned_m = start_position_ned_m[1] + offset_ned_m[1]
  new_z_ned_m = start_position_ned_m[2] + offset_ned_m[2]
  new_point_ned_m = [new_x_ned_m,new_y_ned_m,new_z_ned_m]
  print('')
  print("Point Goal NED Meters")
  print(" X, Y, Z")
  print(["%.2f" % new_point_ned_m[0],"%.2f" % new_point_ned_m[1],"%.2f" % new_point_ned_m[2]])
  new_yaw_ned_deg = convert_yaw_body2ned(new_yaw_body_deg,start_yaw_ned_deg)
  print('')
  print("Yaw Goal NED Degrees")
  print(["%.2f" % new_yaw_ned_deg])
  ##############################################
  # Convert NED Data to ENU Data
  ##############################################
  # New Point ENU in meters
  new_point_enu_m=Point()
  new_point_enu_m.x = new_point_ned_m[1]
  new_point_enu_m.y = new_point_ned_m[0]
  new_point_enu_m.z = - new_point_ned_m[2]
  print('')
  print("Point Goal ENU Meters")
  print(" X, Y, Z")
  print(["%.2f" % new_point_enu_m.x,"%.2f" % new_point_enu_m.y,"%.2f" % new_point_enu_m.z])
  new_yaw_enu_deg = convert_yaw_ned2enu(new_yaw_ned_deg)
  print('')
  print("Yaw Goal ENU Degrees")
  print(["%.2f" % new_yaw_enu_deg])
  ##############################################
  # Create Local ENU Position and Orienation Setpoint Values
  ##############################################
  # New Local Position ENU in meters
  new_point_enu_m=Point()
  new_point_enu_m.x = new_point_enu_m.x
  new_point_enu_m.y = new_point_enu_m.y
  new_point_enu_m.z = new_point_enu_m.z
  print('')
  print("Position Goal ENU Meters")
  print(" X, Y, Z")
  print(["%.2f" % new_point_enu_m.x,"%.2f" % new_point_enu_m.y,"%.2f" % new_point_enu_m.z])
  # New Local Orienation ENU in meters  
  new_orientation_enu_deg = [start_orientation_ned_degs[0],start_orientation_ned_degs[1],new_yaw_enu_deg]
  print('')
  print("Orienation Goal ENU Degrees")
  print(" Roll, Pitch, Yaw")
  print(["%.2f" % new_orientation_enu_deg[0],"%.2f" % new_orientation_enu_deg[1],"%.2f" % new_orientation_enu_deg[2]])
  new_orientation_enu_q = convert_rpy2quat(new_orientation_enu_deg)
  new_orientation_enu_quat = Quaternion()
  new_orientation_enu_quat.x = new_orientation_enu_q[0]
  new_orientation_enu_quat.y = new_orientation_enu_q[1]
  new_orientation_enu_quat.z = new_orientation_enu_q[2]
  new_orientation_enu_quat.w = new_orientation_enu_q[3]
  ##############################################
  # Create PoseStamped Setpoint Local ENU Message
  ##############################################
  new_pose_enu=Pose()
  new_pose_enu.position = new_point_enu_m
  new_pose_enu.orientation = new_orientation_enu_quat
  position_local_target_msg = PoseStamped()
  position_local_target_msg.pose = new_pose_enu
  print('')
  print("Setpoint Goal Position Local Message")
  print(position_local_target_msg)
  ##############################################
  ## Send Message and Check for Setpoint Success
  ##############################################
  setpoint_position_local_pub = rospy.Publisher(MAVLINK_SETPOINT_POSITION_LOCAL_TOPIC, PoseStamped, queue_size=1)
  time.sleep(1) # VERY IMPORTANT - Sleep a bit between declaring a publisher and using it
  print('')
  print("Sending Setpoint Position Local Command at 50 Hz and")
  print("Waiting for Attitude Setpoint to complete")
  setpoint_position_local_point_reached = False
  setpoint_position_local_yaw_reached = False
  stabilize_timer=0
  point_errors = [] # Initialize running list of errors
  yaw_errors = [] # Initialize running list of errors
  timeout_timer = 0 # Initialize timeout timer
  while setpoint_position_local_point_reached is False or setpoint_position_local_yaw_reached is False and not rospy.is_shutdown():  # Wait for setpoint goal to be set
    if timeout_timer > timeout_sec:
      print("Setpoint cmd timed out")
      cmd_success = False
      break
    time2sleep = 0.02
    time.sleep(time2sleep) # update setpoint position at 50 Hz
    stabilize_timer=stabilize_timer+time2sleep # Increment print message timer
    timeout_timer = timeout_timer+time2sleep
    setpoint_position_local_pub.publish(position_local_target_msg) # Publish Setpoint
    # Calculate setpoint position ned errors    
    point_ned_error_m = np.array(current_position_ned_m) - np.array(new_point_ned_m)
    for ind in range(3):
      if input_point_body_m == -999: # Ignore error check if set to current
        point_ned_error_m[ind] = 0
    max_point_ned_error_m = np.max(np.abs(point_ned_error_m))
    # Calculate setpoint yaw ned error
    if input_yaw_body_deg == -999: # Ignore error check if set to current
      setpoint_position_local_yaw_reached = True
      max_yaw_ned_error_deg = 0
    else:
      cur_yaw_ned_deg = current_orientation_ned_degs[2]
      yaw_ned_error_deg =  cur_yaw_ned_deg - new_yaw_ned_deg
      max_yaw_ned_error_deg = abs(yaw_ned_error_deg)
    # Check for setpoint position local point goal
    if  setpoint_position_local_point_reached is False:
      if stabilize_timer > GOTO_STABILIZED_SEC:
        max_point_errors = max(point_errors) # Get max from error window
        point_errors = [max_point_ned_error_m] # reset running list of errors
        # Print some information every second
        print('')
        print("Current Position NED Meters")
        print(" X, Y, Z")
        print(["%.2f" % current_position_ned_m[0],"%.2f" % current_position_ned_m[1],"%.2f" % current_position_ned_m[2]])
        print("Current Goal NED Meters")
        print(" X, Y, Z")
        print(["%.2f" % new_point_ned_m[0],"%.2f" % new_point_ned_m[1],"%.2f" % new_point_ned_m[2]])
        print("Current Errors Meters")
        print(" X, Y, Z")
        print(["%.2f" % point_ned_error_m[0],"%.2f" % point_ned_error_m[1],"%.2f" % point_ned_error_m[2]])
        print("Max Error from Stabilized Check Window Meters")
        print(["%.2f" % max_point_errors])
        if max_point_errors < rbx_goto_error_goals[0]:
          print('')
          print("Position Setpoint Reached")
          setpoint_position_local_point_reached = True
      else:
        point_errors.append(max_point_ned_error_m) # append last
    # Check for setpoint position yaw point goal
    if  setpoint_position_local_yaw_reached is False:
      if stabilize_timer > rbx_goto_error_goals[2]:
        max_yaw_errors = max(yaw_errors) # Get max from error window
        yaw_errors = [max_yaw_ned_error_deg] # reset running list of errors
        # Print some information every second
        print('')
        print("Current Yaw NED Degrees")
        print(current_orientation_ned_degs[2])
        print("Current Goal NED Degrees")
        print(new_yaw_ned_deg)
        print("Current Error Degree")
        print(max_yaw_ned_error_deg)
        print("Max Error from Stabilized Check Window Meters")
        print(["%.2f" % max_yaw_errors])
        if max_yaw_errors < rbx_goto_error_goals[1]:
          print('')
          print("Yaw Setpoint Reached")
          setpoint_position_local_yaw_reached = True
      else:
        yaw_errors.append(max_yaw_ned_error_deg) # append last
    # Reset print timer if past
    if stabilize_timer > rbx_goto_error_goals[2]:
      stabilize_timer=0 # Reset print timer
    rbx_goto_errors_current =  [point_ned_error_m[0],point_ned_error_m[1],point_ned_error_m[2],0,0,0,max_yaw_ned_error_deg]
  if cmd_success:
    print("************************")
    print("Setpoint Reached")
  rbx_goto_errors_current = [0,0,0,0,0,0]
  rbx_goto_errors_last =  [point_ned_error_m[0],point_ned_error_m[1],point_ned_error_m[2],0,0,0,max_yaw_ned_error_deg]
  return cmd_success



### Function to set and check setpoint location global geopoint and yaw command
###################################################
# Input is [LAT, LONG, ALT_WGS84, YAW_NED_DEGREES]
# Converted to AMSL Altitude and ENU Yaw berore sending
# Altitude is specified as meters above the WGS-84 and converted to AMSL before sending
# Yaw is specified in NED frame degrees 0-360 or +-180 
#####################################################
def setpoint_location_global_wgs84(setpoint_location,timeout_sec=CMD_TIMEOUT_SEC):
  # setpoint_location is [LAT, LONG, ALT_WGS84, YEW_NED_DEGREES 0-360 or +-180]
  # Use value -999 to use current value
  global current_orientation_ned_degs
  global current_geoid_height_m
  global current_location_wgs84_geo
  global current_heading_deg
  global rbx_goto_error_goals
  global rbx_goto_errors_current
  cmd_success = True
  rbx_goto_errors_current = [0,0,0,0,0,0,0]
  print('')
  print("Starting Setpoint Location Global Create-Send-Check Process")
  ##############################################
  # Capture Current NavPose Data
  ##############################################
  start_geopoint_wgs84 = list(current_location_wgs84_geo)  
  print('')
  print("Start Location WSG84 geopoint")
  print(" Lat, Long, Alt")
  print(["%.6f" % start_geopoint_wgs84[0],"%.6f" % start_geopoint_wgs84[1],"%.2f" % start_geopoint_wgs84[2]])
  start_orientation_ned_degs=list(current_orientation_ned_degs)
  print('')
  print("Start Orientation NED degs")
  print(" Roll, Pitch, Yaw")
  print(["%.6f" % start_orientation_ned_degs[0],"%.6f" % start_orientation_ned_degs[1],"%.2f" % start_orientation_ned_degs[2]])
  print('')
  start_yaw_ned_deg = start_orientation_ned_degs[2]
  if start_yaw_ned_deg < 0:
    start_yaw_ned_deg = start_yaw_ned_deg + 360
  print('')
  print("Start Yaw NED degs 0-360")
  print(start_yaw_ned_deg) 
  start_heading_deg=current_heading_deg
  print('')
  print("Start Heading degs")
  print(start_heading_deg)
  start_geoid_height_m = current_geoid_height_m
  ##############################################
  # Condition NED Input Data
  ##############################################
  # Condition Location Input
  input_geopoint_wgs84 = list(setpoint_location[0:3])
  print('')
  print("Location Input Global Geo")
  print(" Lat, Long, Alt_WGS84")
  print(["%.8f" % input_geopoint_wgs84[0],"%.8f" % input_geopoint_wgs84[1],"%.2f" % input_geopoint_wgs84[2]])
  new_geopoint_wgs84=list(start_geopoint_wgs84) # Initialize with start
  for ind in range(3): # Overwrite current with new if set and valid
    if input_geopoint_wgs84[ind] != -999:
      new_geopoint_wgs84[ind]=input_geopoint_wgs84[ind]
  print('')
  print("Location Input Conditioned Global Geo")
  print(" Lat, Long, Alt_WGS84")
  print(["%.8f" % new_geopoint_wgs84[0],"%.8f" % new_geopoint_wgs84[1],"%.2f" % new_geopoint_wgs84[2]])
  # Condition Yaw Input
  input_yaw_ned_deg = setpoint_location[3]
  print('')
  print("Yaw Input NED Degrees")
  print(["%.2f" % input_yaw_ned_deg])
  new_yaw_ned_deg = start_yaw_ned_deg # Initialize to current
  if input_yaw_ned_deg != -999: # Replace if not -999
    new_yaw_ned_deg = input_yaw_ned_deg
  # Condition to 0-360 degs
  if new_yaw_ned_deg < 0:
    new_yaw_ned_deg = new_yaw_ned_deg + 360
  print('')
  print("Yaw Input Conditioned NED Degrees 0-360")
  print(["%.2f" % new_yaw_ned_deg])      
  ##############################################
  # Create Global AMSL Location and NED Orienation Setpoint Values
  ##############################################
  # New Global location ENU in meters
  new_geopoint_amsl=GeoPoint()
  new_geopoint_amsl.latitude = new_geopoint_wgs84[0]
  new_geopoint_amsl.longitude = new_geopoint_wgs84[1]
  new_geopoint_amsl.altitude = new_geopoint_wgs84[2] - start_geoid_height_m
  print('')
  print("Location Goal AMSL Meters")
  print(" Lat, Long, Alt_AMSL")
  print(["%.8f" % new_geopoint_amsl.latitude,"%.8f" % new_geopoint_amsl.longitude,"%.2f" % new_geopoint_amsl.altitude])
  # New Local Orienation NED in degs  
  new_orientation_ned_deg = [start_orientation_ned_degs[0],start_orientation_ned_degs[1],new_yaw_ned_deg]
  print('')
  print("Orienation Goal NED Degrees")
  print(" Roll, Pitch, Yaw")
  print(["%.2f" % new_orientation_ned_deg[0],"%.2f" % new_orientation_ned_deg[1],"%.2f" % new_orientation_ned_deg[2]])
  new_orientation_ned_q = convert_rpy2quat(new_orientation_ned_deg)
  new_orientation_ned_quat = Quaternion()
  new_orientation_ned_quat.x = new_orientation_ned_q[0]
  new_orientation_ned_quat.y = new_orientation_ned_q[1]
  new_orientation_ned_quat.z = new_orientation_ned_q[2]
  new_orientation_ned_quat.w = new_orientation_ned_q[3]
  ##############################################
  # Create GeoPose Setpoint Global AMSL and Yaw NED Message
  ##############################################
  new_geopose_enu=GeoPose()
  new_geopose_enu.position = new_geopoint_amsl
  new_geopose_enu.orientation = new_orientation_ned_quat
  location_global_target_msg = GeoPoseStamped()
  location_global_target_msg.pose = new_geopose_enu
  print('')
  print("Setpoint Location Goal Message")
  print(location_global_target_msg)
  ##############################################
  ## Send Message and Check for Setpoint Success
  ##############################################
  setpoint_location_global_pub = rospy.Publisher(MAVLINK_SETPOINT_LOCATION_GLOBAL_TOPIC, GeoPoseStamped, queue_size=1)
  time.sleep(1) # VERY IMPORTANT - Sleep a bit between declaring a publisher and using it
  print("Sending MAVLINK Setpoint Position Local Command at 50 Hz and")
  print(" checking for Setpoint Reached")
  setpoint_location_global_geopoint_reached = False
  setpoint_location_global_yaw_reached = False 
  stabilize_timer=0
  print('')
  print("Waiting for Position Local Setpoint to complete")
  geopoint_errors = [] # Initialize running list of errors
  yaw_errors = [] # Initialize running list of errors
  timeout_timer = 0 # Initialize timeout timer
  while setpoint_location_global_geopoint_reached is False or setpoint_location_global_yaw_reached is False and not rospy.is_shutdown(): # Wait for setpoint goal to be set
    if timeout_timer > timeout_sec:
      print("Setpoint cmd timed out")
      cmd_success = False
      break
    time2sleep = 0.02
    time.sleep(time2sleep) # update setpoint position at 50 Hz
    stabilize_timer=stabilize_timer+time2sleep # Increment print message timer
    timeout_timer = timeout_timer+time2sleep
    setpoint_location_global_pub.publish(location_global_target_msg) # Publish Setpoint
    # Calculate setpoint position and yaw errors
    geopoint_errors_geo = np.array(current_location_wgs84_geo) - np.array(new_geopoint_wgs84)
    geopoint_errors_m = [geopoint_errors_geo[0]*111139,geopoint_errors_geo[1]*111139,geopoint_errors_geo[2]]
    for ind in range(3):  # Ignore error check if set to current
      if input_geopoint_wgs84[ind] == -999.0:
        geopoint_errors_m[ind] = 0
    max_geopoint_error_m = np.max(np.abs(geopoint_errors_m))
    if input_yaw_ned_deg == -999: # Ignore error check if set to current
      setpoint_location_global_yaw_reached = True
      max_yaw_ned_error_deg = 0
    else:
      cur_yaw_ned_deg = current_orientation_ned_degs[2]
      if cur_yaw_ned_deg < 0:
        cur_yaw_ned_deg = cur_yaw_ned_deg + 360
      yaw_ned_error_deg =  cur_yaw_ned_deg - new_yaw_ned_deg
      max_yaw_ned_error_deg = abs(yaw_ned_error_deg)
    # Check for setpoint position global goal
    if  setpoint_location_global_geopoint_reached is False:
      if stabilize_timer > rbx_goto_error_goals[2]:
        max_geopoint_errors = max(geopoint_errors) # Get max from error window
        geopoint_errors = [max_geopoint_error_m] # reset running list of errors
        # Print some information every second
        print('')
        print("Current Location WGS84")
        print(" Lat, Long, Alt_WGS84")
        print(["%.7f" % current_location_wgs84_geo[0],"%.7f" % current_location_wgs84_geo[1],"%.2f" % current_location_wgs84_geo[2]])
        print("Current Goal WGS84")
        print(" Lat, Long, Alt_WGS84")
        print(["%.7f" % new_geopoint_wgs84[0],"%.7f" % new_geopoint_wgs84[1],"%.2f" % new_geopoint_wgs84[2]])
        print("Current Errors Meters")
        print(" Lat, Long, Alt")
        print(["%.2f" % geopoint_errors_m[0],"%.2f" % geopoint_errors_m[1],"%.2f" % geopoint_errors_m[2]])
        print("Max Error from Stabilized Check Window Meters")
        print(["%.2f" % max_geopoint_errors])
        if max_geopoint_errors < rbx_goto_error_goals[0]:
          print('')
          print("Location Setpoint Reached")
          setpoint_location_global_geopoint_reached = True
      else:
        geopoint_errors.append(max_geopoint_error_m) # append last
    # Check for setpoint position yaw goal
    if  setpoint_location_global_yaw_reached is False:
      if stabilize_timer > rbx_goto_error_goals[2]:
        max_yaw_errors = max(yaw_errors) # Get max from error window
        yaw_errors = [max_yaw_ned_error_deg] # reset running list of errors
        # Print some information every second
        print('')
        print("Current Yaw NED Degrees")
        print(cur_yaw_ned_deg)
        print("Current Goal NED Degrees")
        print(new_yaw_ned_deg)
        print("Current Error Degree")
        print(max_yaw_ned_error_deg)
        print("Max Error from Stabilized Check Window Meters")
        print(["%.2f" % max_yaw_errors])
        if max_yaw_errors < rbx_goto_error_goals[1]:
          print('')
          print("Yaw Setpoint Reached")
          setpoint_location_global_yaw_reached = True
      else:
        yaw_errors.append(max_yaw_ned_error_deg) # append last
    # Reset print timer if past
    if stabilize_timer > 1:
      stabilize_timer=0 # Reset print timer
    rbx_goto_errors_current = [geopoint_errors_m[0],geopoint_errors_m[1],geopoint_errors_m[2],0,0,0,max_yaw_ned_error_deg]
  if cmd_success:
    print("************************")
    print("Setpoint Reached")
  rbx_goto_errors_current = [0,0,0,0,0,0]
  rbx_goto_errors_last = [geopoint_errors_m[0],geopoint_errors_m[1],geopoint_errors_m[2],0,0,0,max_yaw_ned_error_deg]
  return cmd_success
  


#######################
# Mavlink Ardupilot Interface Methods

### Function for switching to arm state
def arm():
  set_mavlink_arm_state(True)

### Function for switching to disarm state
def disarm():
  set_mavlink_arm_state(False)

## Function for sending takeoff command
def takeoff(timeout_sec):
  global current_location_wgs84_geo
  global rbx_goto_error_goals
  global rbx_goto_errors_current
  global rbx_goto_errors_last
  global rbx_cmd_timeout
  cmd_success = True
  rbx_goto_errors_current = [0,0,0,0,0,0,0]
  start_alt_m = current_location_wgs84_geo[2]
  start_alt_goal = start_alt_m + TAKEOFF_ALT_M
  print("Sending Takeoff Command to altitude to " + str(TAKEOFF_ALT_M) + " meters")
  takeoff_client = rospy.ServiceProxy(MAVLINK_TAKEOFF_SERVICE, CommandTOL)
  time.sleep(1) # VERY IMPORTANT - Sleep a bit between declaring a publisher and using it
  takeoff_client(min_pitch=TAKEOFF_MIN_PITCH_DEG,altitude=TAKEOFF_ALT_M)
  print("Waiting for takeoff process to complete")
  print_timer=0
  max_alt_error_m = abs(current_location_wgs84_geo[2] - start_alt_goal)
  print(max_alt_error_m)
  timeout_timer = 0
  while max_alt_error_m > rbx_goto_error_goals[0] and not rospy.is_shutdown():
    time2sleep = 0.2
    if timeout_timer > timeout_sec:
      print("Takeoff action timed out")
      cmd_success = False
      break
    else:
      timeout_timer = timeout_timer+time2sleep
    time.sleep(time2sleep) # update setpoint position at 50 Hz
    print_timer = print_timer+time2sleep
    alt_error_m = current_location_wgs84_geo[2] - start_alt_goal
    max_alt_error_m = abs(alt_error_m)
    rbx_goto_errors_current = [0,0,alt_error_m,0,0,0,0]
    if print_timer > 1:
      print_timer = 0
      print("Takeoff errors")
      print(rbx_goto_errors_current)
  if cmd_success:
    print("Takeoff action complete")
  rbx_goto_errors_current = [0,0,0,0,0,0]
  rbx_goto_errors_last = [0,0,alt_error_m,0,0,0,0]
  return cmd_success


### Function for switching to STABILIZE mode
def stabilize():
  set_mavlink_mode('STABILIZE')
  cmd_success = True
  return cmd_success
    
### Function for switching to LAND mode
def land():
  global rbx_state
  set_mavlink_mode('LAND')
  print("Waiting for land process to complete and disarm")
  while rbx_state == 1:
    time.sleep(1)
  success = True
  return success


### Function for sending go home command
def rtl():
  set_mavlink_mode('RTL')
  success = True
  return success


### Function for switching to LOITER mode
def loiter():
  set_mavlink_mode('LOITER')
  success = True
  return success


### Function for switching to Guided mode
def guided():
  set_mavlink_mode('GUIDED')
  success = True
  return success

### Function for switching back to current mission
def resume():
  global rbx_mode
  global rbx_mode_last
  # Reset mode to last
  print("Switching mavlink mode from " + RBX_MODES[rbx_mode] + " back to " + RBX_MODES[rbx_mode_last])
  rbx_set_mode(rbx_mode_last)
  success = True
  return success


### Function for sending set home current
def sethome_current():
  global current_home
  print('Sending mavlink set home current command')
  set_home_client = rospy.ServiceProxy(MAVLINK_SET_HOME_SERVICE, CommandHome)
  time.sleep(.1) # VERY IMPORTANT - Sleep a bit between declaring a publisher and using it
  set_home_client(current_gps=True)
  success = True
  return success




#######################
# NEPI NavPose Interface Methods

### Setup a regular background navpose get and update navpose data
def update_current_navpose_callback(timer):
  global current_heading_deg
  global current_orientation_enu_degs
  global current_orientation_ned_degs
  global current_position_enu_m
  global current_position_ned_m
  global current_location_amsl_geo
  global current_location_wgs84_geo
  global current_geoid_height_m
  # Get current NEPI NavPose data from NEPI ROS nav_pose_query service call
  try:
    get_navpose_service = rospy.ServiceProxy(NEPI_NAVPOSE_SERVICE_NAME, NavPoseQuery)
    nav_pose_response = get_navpose_service(NavPoseQueryRequest())
    #print(nav_pose_response)
    # Set current navpose
    current_navpose = nav_pose_response.nav_pose
    # Set current heading in degrees
    current_heading_deg = nav_pose_response.nav_pose.heading.heading
    # Set current orientation vector (roll, pitch, yaw) in degrees enu frame
    pose_enu_o = nav_pose_response.nav_pose.odom.pose.pose.orientation
    xyzw_enu_o = list([pose_enu_o.x,pose_enu_o.y,pose_enu_o.z,pose_enu_o.w])
    rpy_enu_d = convert_quat2rpy(xyzw_enu_o)
    current_orientation_enu_degs = [rpy_enu_d[0],rpy_enu_d[1],rpy_enu_d[2]]
    # Set current orientation vector (roll, pitch, yaw) in degrees ned frame +-180
    pose_enu_o = nav_pose_response.nav_pose.odom.pose.pose.orientation
    xyzw_enu_o = list([pose_enu_o.x,pose_enu_o.y,pose_enu_o.z,pose_enu_o.w])
    rpy_enu_d = convert_quat2rpy(xyzw_enu_o)
    yaw_ned_d = convert_yaw_enu2ned(rpy_enu_d[2])
    rpy_ned_d = [rpy_enu_d[0],rpy_enu_d[1],yaw_ned_d]
    current_orientation_ned_degs = [rpy_ned_d[0],rpy_ned_d[1],rpy_ned_d[2]]
    # Set current position vector (x, y, z) in meters enu frame
    pose_enu_p = nav_pose_response.nav_pose.odom.pose.pose.position
    current_position_enu_ma = [pose_enu_p.x, pose_enu_p.y, pose_enu_p.z]
    # Set current position vector (x, y, z) in meters ned frame
    pose_enu_p = nav_pose_response.nav_pose.odom.pose.pose.position
    current_position_ned_m = [pose_enu_p.y, pose_enu_p.x, -pose_enu_p.z]
    # Set current location vector (lat, long, alt) in geopoint data with WGS84 height
    fix_wgs84 = nav_pose_response.nav_pose.fix
    current_location_wgs84_geo =  [fix_wgs84.latitude,fix_wgs84.longitude,fix_wgs84.altitude]
    # Set current geoid height
    single_position=LatLon(fix_wgs84.latitude,fix_wgs84.longitude)
    geoid_height = ginterpolator(single_position)
    current_geoid_height_m =  geoid_height
    # Set current location vector (lat, long, alt) in geopoint data with WGS84 height
    fix_amsl = nav_pose_response.nav_pose.fix
    current_location_wgs84_geo =  [fix_amsl.latitude,fix_amsl.longitude,fix_amsl.altitude]
    # Set current location vector (lat, long, alt) in geopoint data with AMSL height
    current_location_amsl_geo =  [fix_wgs84.latitude,fix_wgs84.longitude,(fix_wgs84.altitude + geoid_height)]
  except Exception as e:
    print("navpose service call failed: " + str(e))

#######################
# Data Converstion Methods

### Function to Convert Quaternion Attitude to Roll, Pitch, Yaw Degrees
def convert_quat2rpy(xyzw_attitude):
  rpy_attitude_rad = tf.transformations.euler_from_quaternion(xyzw_attitude)
  rpy_attitude_ned_deg = np.array(rpy_attitude_rad) * 180/math.pi
  roll_deg = rpy_attitude_ned_deg[0] 
  pitch_deg = rpy_attitude_ned_deg[1] 
  yaw_deg = rpy_attitude_ned_deg[2]
  return rpy_attitude_ned_deg

### Function to Convert Roll, Pitch, Yaw Degrees to Quaternion Attitude
def convert_rpy2quat(rpy_attitude_ned_deg):
  roll_deg = rpy_attitude_ned_deg[0] 
  pitch_deg = rpy_attitude_ned_deg[1] 
  yaw_deg = rpy_attitude_ned_deg[2]
  xyzw_attitude = tf.transformations.quaternion_from_euler(math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg))
  return xyzw_attitude

### Function to Convert Yaw NED to Yaw ENU
def convert_yaw_ned2enu(yaw_ned_deg):
  yaw_enu_deg = 90-yaw_ned_deg
  if yaw_enu_deg < -180:
    yaw_enu_deg = 360 + yaw_enu_deg
  elif yaw_enu_deg > 180:
    yaw_enu_deg = yaw_enu_deg - 360
  return yaw_enu_deg

### Function to Convert Yaw ENU to Yaw NED
def convert_yaw_enu2ned(yaw_enu_deg):
  yaw_ned_deg =  90-yaw_enu_deg
  if yaw_ned_deg < -180:
    yaw_ned_deg = 360 + yaw_ned_deg
  elif yaw_ned_deg > 180:
    yaw_ned_deg = yaw_ned_deg - 360
  return yaw_ned_deg

### Function to Convert Yaw from Body to NED Frame
def convert_yaw_body2ned(yaw_body_deg,cur_heading_deg):
  cur_yaw_ned_deg = cur_heading_deg
  if cur_yaw_ned_deg > 180: # Convert to +-180
    cur_yaw_ned_deg = cur_yaw_ned_deg - 360
  yaw_ned_deg =  cur_yaw_ned_deg + yaw_body_deg
  return yaw_ned_deg

### Function to Convert Point from Body to NED Frame
def convert_point_body2ned(setpoint_position,yaw_ned_deg):
  point_bearing_ned_deg = yaw_ned_deg + math.degrees(math.atan2(setpoint_position[1],setpoint_position[0]))
  point_bearing_ned_rad = math.radians(point_bearing_ned_deg)
  xy_body_m = math.sqrt(setpoint_position[0]**2 + setpoint_position[1]**2)
  x_ned_m = xy_body_m * math.cos(point_bearing_ned_rad)
  y_ned_m = xy_body_m * math.sin(point_bearing_ned_rad)
  point_ned_m = [x_ned_m,y_ned_m,setpoint_position[2]]
  return point_ned_m

### Function to Convert Altitude from AMSL to WGS84 Height
def convert_altitude_amsl2wgs84(alt_amsl_m,cur_geoid_height):
  alt_wgs84_m = alt_amsl_m + cur_geoid_height
  return alt_wgs84_m

### Function to Convert Altitude from WGS84 to AMSL Height
def convert_altitude_wgs84amsl(alt_wgs84_m,cur_geoid_height):
  alt_amsl_m = alt_wgs84_m - cur_geoid_height
  return  alt_amsl_m
  
### Function to get distance between two geo latlong locations
def distance_geopoints(geopoint1,geopoint2):
  lat1 = math.radians(geopoint1[0])
  lat2 = math.radians(geopoint2[0])
  lon1 = math.radians(geopoint1[1])
  lon2 = math.radians(geopoint2[1])
  # Haversine formula 
  dlon = lon2 - lon1 
  dlat = lat2 - lat1
  a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
  c = 2 * math.asin(math.sqrt(a)) 
  # Radius of earth in kilometers. Use 3956 for miles
  r = 6371
  xy_m=c*r/1000
  alt_m = abs(geopoint1[2]-geopoint2[2])
  distance_m = math.sqrt(alt_m**2 + xy_m**2) 
  # calculate the result
  return(distance_m)



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
  stabilize() # Change mode
  
### Script Entrypoint
def startNode():
  rospy.loginfo("Starting Ardupilot RBX Driver Script")
  rospy.init_node
  rospy.init_node(name= NEPI_RBX_NAME)
  #initialize system including pan scan process
  initialize_actions()
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

