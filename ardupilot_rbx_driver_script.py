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
from resources import nepi
from resources import nepi_navpose

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Bool, String, Float32, Float64, Float64MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, NavSatFix, BatteryState
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped
from geographic_msgs.msg import GeoPoint, GeoPose, GeoPoseStamped
from mavros_msgs.msg import State, AttitudeTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL, CommandHome
from pygeodesy.ellipsoidalKarney import LatLon
from cv_bridge import CvBridge

from nepi_ros_interfaces.msg import RBXStatus, AxisControls, RBXErrorBounds, RBXGotoErrors
from nepi_ros_interfaces.srv import NavPoseQuery, NavPoseQueryRequest, RBXCapabilitiesQuery, RBXCapabilitiesQueryResponse, \
     NavPoseCapabilitiesQuery, NavPoseCapabilitiesQueryResponse


#########################################
# DRIVER SETTINGS
#########################################

##############################

##############################
# RBX Initialization Values
GOTO_TRAN_SPEED_RATIO = 0.5
GOTO_ROT_SPEED_RATIO = 0.5
GOTO_MAX_ERROR_M = 2.0 # Goal reached when all translation move errors are less than this value
GOTO_MAX_ERROR_DEG = 2.0 # Goal reached when all rotation move errors are less than this value
GOTO_STABILIZED_SEC = 1.0 # Window of time that setpoint error values must be good before proceeding
CMD_TIMEOUT_SEC = 25 # Any action that changes 
IMAGE_INPUT_TOPIC_NAME = "color_2d_image" # Partial or full ROS namespace string, "" for black image 

##############################
# ARDUPILOT Settings
TAKEOFF_MIN_PITCH_DEG = 10
TAKEOFF_ALT_M = 1

#########################################
# ROS NAMESPACE SETUP
#########################################

# ROS namespace setup
NEPI_BASE_NAMESPACE = "/nepi/s2x/"
# RBX node name
NEPI_RBX_NODENAME = "ardupilot"

#########################################
# Node Class
#########################################

class ardupilot_rbx_driver(object):
  # RBX State and Mode Dictionaries
  RBX_NAVPOSE_HAS_GPS = True
  RBX_NAVPOSE_HAS_ORIENTATION = True
  RBX_NAVPOSE_HAS_HEADING = True
  RBX_STATES = ["DISARM","ARM"]
  RBX_MODES = ["STABILIZE","LAND","RTL","LOITER","GUIDED","RESUME"]
  RBX_ACTIONS = ["TAKEOFF"]

  RBX_STATE_FUNCTIONS = ["disarm","arm"]
  RBX_MODE_FUNCTIONS = ["stabilize","land","rtl","loiter","guided","resume"]
  RBX_ACTION_FUNCTIONS = ["takeoff"]



  # Define capabilities
  rbx_capabilities_report = RBXCapabilitiesQueryResponse()
  rbx_capabilities_report.has_standby_mode = False
  rbx_capabilities_report.state_options = RBX_STATES
  rbx_capabilities_report.mode_options = RBX_MODES
  rbx_capabilities_report.action_options = RBX_ACTIONS

  rbx_navpose_report = NavPoseCapabilitiesQueryResponse()
  rbx_navpose_report.has_gps = True
  rbx_navpose_report.has_orientation = True
  rbx_navpose_report.has_heading = True

  #######################
  ### Node Initialization
  def __init__(self):
    rospy.loginfo("Starting Initialization Processes")
    ## Initialize Class Variables
    self.takeoff_m = TAKEOFF_ALT_M
    self.update_navpose_interval = 0.1 # 10 Hz
    self.current_heading_deg = None
    self.current_orientation_enu_degs = None
    self.current_orientation_ned_degs = None
    self.current_position_enu_m = None
    self.current_position_ned_m = None
    self.current_location_amsl_geo = None
    self.current_location_wgs84_geo = None
    self.current_geoid_height_m = 0
    
    self.rbx_state_start = None
    self.rbx_state_last = None
    self.rbx_mode_start = None
    self.rbx_mode_last = None
    NEPI_RBX_STATUS_RATE_HZ = 10
    self.rbx_status_pub_interval = float(1.0)/float(NEPI_RBX_STATUS_RATE_HZ)
    self.rbx_status_image_blank = np.zeros((350, 700, 3), dtype = np.uint8) # Empty Black Image
    self.rbx_status_image_base = self.rbx_status_image_blank
    self.rbx_status_image_source_last = ""
    
    self.mavlink_state = None
    
    # Initialize Status Message
    self.rbx_status=RBXStatus()
    self.rbx_status.serial_num = ""
    self.rbx_status.hw_version = ""
    self.rbx_status.sw_version = ""
    self.rbx_status.standby = False
    axis_controls = AxisControls()
    axis_controls.x = True
    axis_controls.y = True
    axis_controls.z = True
    axis_controls.roll = True
    axis_controls.pitch = True
    axis_controls.yaw = True
    self.rbx_status.control_support = axis_controls
    self.rbx_status.state = None
    self.rbx_status.mode = None
    self.rbx_status.process_current = "None"
    self.rbx_status.process_last = "None"
    self.rbx_status.ready = False
    self.rbx_status.battery = 0
    self.rbx_status.move_speed = GOTO_TRAN_SPEED_RATIO
    self.rbx_status.rotate_speed = GOTO_ROT_SPEED_RATIO
    error_bounds = RBXErrorBounds()
    error_bounds.max_distance_error_m = GOTO_MAX_ERROR_M
    error_bounds.max_rotation_error_deg = GOTO_MAX_ERROR_DEG
    error_bounds.max_stabilize_time_s = GOTO_STABILIZED_SEC
    self.rbx_status.error_bounds = error_bounds
    self.zero_errors = RBXGotoErrors()
    self.zero_errors.x_m = 0
    self.zero_errors.y_m = 0
    self.zero_errors.z_m = 0
    self.zero_errors.heading_deg = 0
    self.zero_errors.roll_deg = 0
    self.zero_errors.pitch_deg = 0
    self.zero_errors.yaw_deg = 0
    self.rbx_status.errors_current = self.zero_errors
    self.rbx_status.errors_prev = self.zero_errors
    self.rbx_status.cmd_timeout = CMD_TIMEOUT_SEC
    self.rbx_status.cmd_success = False
    self.rbx_status.status_image_source = IMAGE_INPUT_TOPIC_NAME

    ## Define NEPI Namespaces
    NEPI_SET_NAVPOSE_GPS_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose_mgr/set_gps_fix_topic"
    NEPI_SET_NAVPOSE_HEADING_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose_mgr/set_heading_topic"
    NEPI_SET_NAVPOSE_ORIENTATION_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose_mgr/set_orientation_topic"
    NEPI_ENABLE_NAVPOSE_GPS_CLOCK_SYNC_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose_mgr/enable_gps_clock_sync"

    ## Define Class Namespaces
    NEPI_NAVPOSE_SERVICE_NAME = NEPI_BASE_NAMESPACE + "nav_pose_query"
    NEPI_RBX_NAMESPACE = NEPI_BASE_NAMESPACE + NEPI_RBX_NODENAME + "/rbx/"
    # NEPI RBX Driver Capabilities Publish Topics
    NEPI_RBX_CAPABILITIES_TOPIC = NEPI_RBX_NAMESPACE + "capabilities_query"
    NEPI_RBX_CAPABILITIES_NAVPOSE_TOPIC = NEPI_RBX_NAMESPACE + "navpose_query"
    # NEPI RBX Driver Status Publish Topics
    NEPI_RBX_STATUS_TOPIC = NEPI_RBX_NAMESPACE + "status" 
    NEPI_RBX_STATUS_IMAGE_TOPIC= NEPI_RBX_NAMESPACE + "status_image" # Image with status info overlay
    NEPI_RBX_GET_TAKEOFF_M_TOPIC = NEPI_RBX_NAMESPACE + "get_takeoff_m"  # Float meters takeoff height
    # NEPI RBX Driver NavPose Publish Topics
    NEPI_RBX_NAVPOSE_GPS_TOPIC = NEPI_RBX_NAMESPACE + "gps_fix"
    NEPI_RBX_NAVPOSE_ODOM_TOPIC = NEPI_RBX_NAMESPACE + "odom"
    NEPI_RBX_NAVPOSE_HEADING_TOPIC = NEPI_RBX_NAMESPACE + "heading"
    # NEPI RBX Driver Settings Subscriber Topics
    NEPI_RBX_SET_STATE_TOPIC = NEPI_RBX_NAMESPACE + "set_state" # Int to Defined Dictionary self.RBX_STATES
    NEPI_RBX_SET_MODE_TOPIC = NEPI_RBX_NAMESPACE + "set_mode"  # Int to Defined Dictionary self.RBX_MODES
    NEPI_RBX_SET_HOME_CURRENT_TOPIC = NEPI_RBX_NAMESPACE + "set_home_current" # Emplty
    NEPI_RBX_SET_MOVE_SPEED_TOPIC = NEPI_RBX_NAMESPACE + "set_move_speed" # Float [Translation Speed Ratio]
    NEPI_RBX_SET_ROTATE_SPEED_TOPIC = NEPI_RBX_NAMESPACE + "set_move_speed" # Float [Translation Speed Ratio]
    NEPI_RBX_SET_GOTO_GOALS_TOPIC = NEPI_RBX_NAMESPACE + "set_goto_goals" # Float [Max_Meters,Max_Degrees,Stabilize_Time_Sec]
    NEPI_RBX_SET_CMD_TIMEOUT_TOPIC = NEPI_RBX_NAMESPACE + "set_cmd_timeout" # Int Seconds  - Any command that changes ready state
    NEPI_RBX_SET_STATUS_IMAGE_TOPIC = NEPI_RBX_NAMESPACE + "set_image_topic" # full ROS namespace
    NEPI_RBX_SET_TAKEOFF_M_TOPIC = NEPI_RBX_NAMESPACE + "set_takeoff_m"  # Float meters takeoff height
    # NEPI RBX Driver Control Subscriber Topics
    NEPI_RBX_GO_ACTION_TOPIC = NEPI_RBX_NAMESPACE + "go_action"  # Int to Defined Dictionary self.RBX_ACTIONS
    NEPI_RBX_GO_HOME_TOPIC = NEPI_RBX_NAMESPACE + "go_home" # Ignored if any active goto processes
    NEPI_RBX_GO_STOP_TOPIC = NEPI_RBX_NAMESPACE + "go_stop" # Aborts any active goto processes
    NEPI_RBX_GOTO_POSE_TOPIC = NEPI_RBX_NAMESPACE + "goto_pose" # Ignored if any active goto processes
    NEPI_RBX_GOTO_POSITION_TOPIC = NEPI_RBX_NAMESPACE + "goto_position" # Ignored if any active goto processes
    NEPI_RBX_GOTO_LOCATION_TOPIC = NEPI_RBX_NAMESPACE + "goto_location" # Ignored if any active goto processes
    ###################################################
    # MAVLINK Namespace
    # Find Mavlink NameSpace
    node_string = "mavlink_tty"
    rospy.loginfo("Waiting for node that includes string: " + node_string)
    node_name = nepi.wait_for_node(node_string)
    MAVLINK_NAMESPACE = (node_name + '/')
    rospy.loginfo("Found mavlink namespace: " + MAVLINK_NAMESPACE)
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

    ## Define Mavlink Services Calls
    self.get_navpose_service = rospy.ServiceProxy(NEPI_NAVPOSE_SERVICE_NAME, NavPoseQuery)
    self.set_home_client = rospy.ServiceProxy(MAVLINK_SET_HOME_SERVICE, CommandHome)
    self.mode_client = rospy.ServiceProxy(MAVLINK_SET_MODE_SERVICE, SetMode)
    self.arming_client = rospy.ServiceProxy(MAVLINK_ARMING_SERVICE, CommandBool)
    self.takeoff_client = rospy.ServiceProxy(MAVLINK_TAKEOFF_SERVICE, CommandTOL)
    
    ## Define NEPI Services Calls
    set_gps_pub = rospy.Publisher(NEPI_SET_NAVPOSE_GPS_TOPIC, String, queue_size=1)
    set_orientation_pub = rospy.Publisher(NEPI_SET_NAVPOSE_ORIENTATION_TOPIC, String, queue_size=1)
    set_heading_pub = rospy.Publisher(NEPI_SET_NAVPOSE_HEADING_TOPIC, String, queue_size=1)
    set_gps_timesync_pub = rospy.Publisher(NEPI_ENABLE_NAVPOSE_GPS_CLOCK_SYNC_TOPIC, Bool, queue_size=1)
   
    ## Create Class Sevices
    rospy.Service(NEPI_RBX_CAPABILITIES_TOPIC, RBXCapabilitiesQuery, self.rbx_capabilities_query_callback)
    rospy.Service(NEPI_RBX_CAPABILITIES_NAVPOSE_TOPIC, NavPoseCapabilitiesQuery, self.navpose_capabilities_query_callback)

    ## Create Class Publishers
    self.rbx_status_pub = rospy.Publisher(NEPI_RBX_STATUS_TOPIC, RBXStatus, queue_size=1)
    self.rbx_status_image_pub = rospy.Publisher(NEPI_RBX_STATUS_IMAGE_TOPIC, Image, queue_size=1)
    self.rbx_get_takeoff_m_pub = rospy.Publisher(NEPI_RBX_GET_TAKEOFF_M_TOPIC, Float32, queue_size=1)
                        
    self.rbx_navpose_gps_pub = rospy.Publisher(NEPI_RBX_NAVPOSE_GPS_TOPIC, NavSatFix, queue_size=1)
    self.rbx_navpose_odom_pub = rospy.Publisher(NEPI_RBX_NAVPOSE_ODOM_TOPIC, Odometry, queue_size=1)
    self.rbx_navpose_heading_pub = rospy.Publisher(NEPI_RBX_NAVPOSE_HEADING_TOPIC, Float64, queue_size=1)

    self.setpoint_location_global_pub = rospy.Publisher(MAVLINK_SETPOINT_LOCATION_GLOBAL_TOPIC, GeoPoseStamped, queue_size=1)
    self.setpoint_attitude_pub = rospy.Publisher(MAVLINK_SETPOINT_ATTITUDE_TOPIC, AttitudeTarget, queue_size=1)
    self.setpoint_position_local_pub = rospy.Publisher(MAVLINK_SETPOINT_POSITION_LOCAL_TOPIC, PoseStamped, queue_size=1)
    ## Start Class Subscribers
    # Wait for MAVLink State topic to publish then subscribe
    rospy.loginfo("Waiting for topic: " + MAVLINK_STATE_TOPIC)
    nepi.wait_for_topic(MAVLINK_STATE_TOPIC)
    rospy.loginfo("Starting state scubscriber callback")
    rospy.Subscriber(MAVLINK_STATE_TOPIC, State, self.get_state_callback)
    while self.rbx_status.state is None and not rospy.is_shutdown():
      rospy.loginfo("Waiting for rbx state status to set")
      time.sleep(0.1)
    while self.rbx_status.mode is None and not rospy.is_shutdown():
      rospy.loginfo("Waiting for rbx mode status to set")
      time.sleep(0.1)
    rospy.loginfo("Starting State: " + self.RBX_STATES[self.rbx_status.state])
    rospy.loginfo("Starting Mode: " + self.RBX_MODES[self.rbx_status.mode])
    self.rbx_state_start = self.rbx_status.state
    self.rbx_mode_start = self.rbx_status.mode
    self.rbx_state_last = self.rbx_status.state
    self.rbx_mode_last = self.rbx_status.mode
    # Subscribe to MAVLink topics
    rospy.Subscriber(MAVLINK_BATTERY_TOPIC, BatteryState, self.get_mavlink_battery_callback)
    rospy.Subscriber(MAVLINK_SOURCE_GPS_TOPIC, NavSatFix, self.rbx_gps_topic_callback)
    rospy.Subscriber(MAVLINK_SOURCE_ODOM_TOPIC, Odometry, self.rbx_odom_topic_callback)
    rospy.Subscriber(MAVLINK_SOURCE_HEADING_TOPIC, Float64, self.rbx_heading_topic_callback)
    ### Start RBX Settings Subscribe Topics
    rospy.Subscriber(NEPI_RBX_SET_STATE_TOPIC, UInt8, self.rbx_set_state_callback)
    rospy.Subscriber(NEPI_RBX_SET_MODE_TOPIC, UInt8, self.rbx_set_mode_callback)
    rospy.Subscriber(NEPI_RBX_SET_HOME_CURRENT_TOPIC, Empty, self.rbx_set_home_current_callback)
    rospy.Subscriber(NEPI_RBX_SET_MOVE_SPEED_TOPIC, Float32, self.rbx_set_move_speed_callback)
    rospy.Subscriber(NEPI_RBX_SET_ROTATE_SPEED_TOPIC, Float32, self.rbx_set_rotate_speed_callback)
    rospy.Subscriber(NEPI_RBX_SET_GOTO_GOALS_TOPIC, Float64MultiArray, self.rbx_set_goto_goals_callback)
    rospy.Subscriber(NEPI_RBX_SET_CMD_TIMEOUT_TOPIC, UInt32, self.rbx_set_cmd_timeout_callback)
    rospy.Subscriber(NEPI_RBX_SET_TAKEOFF_M_TOPIC, Float32, self.rbx_set_takeoff_m_callback)
    ### Start RBX Control Subscribe Topics
    rospy.Subscriber(NEPI_RBX_GO_ACTION_TOPIC, UInt8, self.rbx_go_action_callback)
    rospy.Subscriber(NEPI_RBX_GO_HOME_TOPIC, Empty, self.rbx_go_home_callback)
    rospy.Subscriber(NEPI_RBX_GO_STOP_TOPIC, Empty, self.rbx_go_stop_callback)
    rospy.Subscriber(NEPI_RBX_GOTO_POSE_TOPIC, Float64MultiArray, self.rbx_goto_pose_callback)
    rospy.Subscriber(NEPI_RBX_GOTO_POSITION_TOPIC, Float64MultiArray, self.rbx_goto_position_callback)
    rospy.Subscriber(NEPI_RBX_GOTO_LOCATION_TOPIC, Float64MultiArray, self.rbx_goto_location_callback)
    ## Start Node Processes
    # Start NavPose Data Updater
    rospy.Timer(rospy.Duration(self.update_navpose_interval), self.update_current_navpose_callback)
    # Start RBX Capabilities and Status Publishers
    rospy.Timer(rospy.Duration(self.rbx_status_pub_interval), self.rbx_status_pub_callback)
    # Connect to NEPI NavPose Solution
    # Set GPS Topic
    set_gps_pub.publish(NEPI_RBX_NAVPOSE_GPS_TOPIC)
    rospy.loginfo("GPS Topic Set to: " + NEPI_RBX_NAVPOSE_GPS_TOPIC)
    # Set Orientation Topic
    set_orientation_pub.publish(NEPI_RBX_NAVPOSE_ODOM_TOPIC)
    rospy.loginfo("Orientation Topic Set to: " + NEPI_RBX_NAVPOSE_ODOM_TOPIC)
    # Set Heading Topic
    set_heading_pub.publish(NEPI_RBX_NAVPOSE_HEADING_TOPIC)
    rospy.loginfo("Heading Topic Set to: " + NEPI_RBX_NAVPOSE_HEADING_TOPIC)
    # Sync NEPI clock to GPS timestamp
    set_gps_timesync_pub.publish(data=True)
    rospy.loginfo("Setup complete")
    ## Initiation Complete
    rospy.loginfo("Initialization Complete")

  #######################
  ### Node Methods

  ### callback to provide capabilities report ###
  def rbx_capabilities_query_callback(self, _):
    return self.rbx_capabilities_report

  ### callback to provide navpose capabilities report ###
  def navpose_capabilities_query_callback(self, _):
    return self.rbx_navpose_report
  

  ##############################
  # RBX Status Topic Publishers
  ### Callback for rbx status publisher
  def rbx_status_pub_callback(self,timer):
    if not rospy.is_shutdown():
      self.rbx_status_pub.publish(self.rbx_status)
      self.rbx_get_takeoff_m_pub.publish(data=self.takeoff_m)
      ## Update Status Image and Publish
      rbx_status_image = self.rbx_status_image_base # Initialize status image
      box_x = 10
      box_y = 10
      box_w = 350
      box_h = 450
      # Add status box overlay
      cv2.rectangle(rbx_status_image, (box_x, box_y), (box_w, box_h), (255, 255, 255), -1)
      # Create Status Info Text List
      text_list = ["RBX Status"]
      if self.rbx_status.battery < 0.1:
        battery_string = "No Reading"
      else:
        battery_string = '%.2f' % self.rbx_status.battery
      text_list.append("Battery: " + battery_string)
      text_list.append("State: " + self.RBX_STATES[self.rbx_status.state])
      text_list.append("Mode Current: " + self.RBX_MODES[self.rbx_status.mode])
      text_list.append("Mode Last: " + self.RBX_MODES[self.rbx_mode_last])
      text_list.append("Ready: " + str(self.rbx_status.ready))
      text_list.append("")
      text_list.append("Current Process: " + self.rbx_status.process_current)
      text_list.append(" X,Y,Z Errors Meters: ")
      text_list.append(" " + '%.2f' % self.rbx_status.errors_current.x_m + "," + '%.2f' % self.rbx_status.errors_current.y_m + "," + '%.2f' % self.rbx_status.errors_current.z_m)
      text_list.append(" R,P,Y Errors Degrees: ")
      text_list.append(" " + '%.2f' % self.rbx_status.errors_current.roll_deg + "," + '%.2f' % self.rbx_status.errors_current.pitch_deg + "," + '%.2f' % self.rbx_status.errors_current.yaw_deg)
      text_list.append("")
      text_list.append("Last Process: " + self.rbx_status.process_last)
      text_list.append(" Success: " + str(self.rbx_status.cmd_success))
      text_list.append(" X,Y,Z Errors Meters: ")
      text_list.append(" " + '%.2f' % self.rbx_status.errors_prev.x_m + "," + '%.2f' % self.rbx_status.errors_prev.y_m + "," + '%.2f' % self.rbx_status.errors_prev.z_m)
      text_list.append(" R,P,Y Errors Degrees: ")
      text_list.append(" " + '%.2f' % self.rbx_status.errors_prev.roll_deg + "," + '%.2f' % self.rbx_status.errors_prev.pitch_deg + "," + '%.2f' % self.rbx_status.errors_prev.yaw_deg)
      text_list.append("")
      # Overlay Status Text List
      x=box_x+10 
      y=box_y+20
      for text in text_list:
        self.status_text_overlay(rbx_status_image,text,x, y)
        y = y + 20
      # Create ROS Image message
      bridge = CvBridge()
      img_out_msg = bridge.cv2_to_imgmsg(rbx_status_image,"bgr8")#desired_encoding='passthrough')
      # Publish new image to ros
      if not rospy.is_shutdown():
        self.rbx_status_image_pub.publish(img_out_msg)
        # You can view the enhanced_2D_image topic at 
        # //192.168.179.103:9091/ in a connected web browser
    ## Update image source topic and subscriber if changed from last time.
    if self.rbx_status.status_image_source != self.rbx_status_image_source_last:
      #  If currently set, first unregister current image topic
      if self.rbx_status_image_source_last != "": 
        self.rbx_status_image_pub.unregister()
        time.sleep(1)
      # Try to find and subscribe to new image source topic
      #rospy.loginfo("Looking for topic: " + self.rbx_status.status_image_source)
      image_topic = nepi.find_topic(self.rbx_status.status_image_source)
      #rospy.loginfo("Search returned: " + image_topic)
      if image_topic != "":  # If image topic exists subscribe
        rospy.Subscriber(image_topic, Image, self.update_status_base_image_callback, queue_size = 1)
        self.rbx_status_image_source_last = self.rbx_status.status_image_source
    if self.rbx_status.status_image_source == "":
      self.rbx_status_image_base = self.rbx_status_image_blank # Set to blank image if source topic is cleared.
  ## Status Text Overlay Function
  def status_text_overlay(self,cv_image,status_text,x,y):
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
    

  ##############################
  ### Update status image source subscriber
  def update_status_base_image_callback(self,img_msg):
    #Convert image from ros to cv2
    bridge = CvBridge()
    self.rbx_status_image_base = bridge.imgmsg_to_cv2(img_msg, "bgr8")


  ##############################
  # RBX NavPose Topic Publishers
  ### Callback to publish RBX navpose gps topic
  
  def rbx_gps_topic_callback(self,navsatfix_msg):
    #Fix Mavros Altitude Error
    altitude_wgs84 = navsatfix_msg.altitude - self.current_geoid_height_m
    navsatfix_msg.altitude = altitude_wgs84 
    if not rospy.is_shutdown():
      self.rbx_navpose_gps_pub.publish(navsatfix_msg)
    
  ### Callback to publish RBX odom topic
  def rbx_odom_topic_callback(self,odom_msg):
    if not rospy.is_shutdown():
      self.rbx_navpose_odom_pub.publish(odom_msg)

  ### Callback to publish RBX heading topic
  def rbx_heading_topic_callback(self,heading_msg):
    if not rospy.is_shutdown():
      self.rbx_navpose_heading_pub.publish(heading_msg)

  ##############################
  # RBX Settings Topic Callbacks

  # ToDo: Create a custom RBX status message
  ### Callback to set state
  def rbx_set_state_callback(self,state_msg):
    rospy.loginfo("*******************************")
    rospy.loginfo("Received set state message")
    rospy.loginfo(state_msg)
    state_val = state_msg.data
    self.rbx_set_state(state_val)

  ### Function to set state
  def rbx_set_state(self,new_state_ind):
    if new_state_ind < 0 or new_state_ind > (len(self.RBX_STATE_FUNCTIONS)-1):
      rospy.loginfo("No matching rbx state found")
    else:
      self.rbx_status.process_current = self.RBX_STATES[new_state_ind]
      self.rbx_state_last = self.rbx_status.state
      rospy.loginfo("Waiting for rbx state " + self.RBX_STATES[new_state_ind] + " to set")
      rospy.loginfo("Current rbx state is " + self.RBX_STATES[self.rbx_status.state])
      set_state_function = globals()[self.RBX_STATE_FUNCTIONS[new_state_ind]]
      success = set_state_function(self)
      self.rbx_status.process_last = self.RBX_STATES[new_state_ind]
      self.rbx_status.process_current = "None"
      if success:
        self.rbx_status.state = new_state_ind

  ### Callback to set mode
  def rbx_set_mode_callback(self,mode_msg):
    rospy.loginfo("*******************************")
    rospy.loginfo("Received set mode message")
    rospy.loginfo(mode_msg)
    mode_val = mode_msg.data
    self.rbx_set_mode(mode_val)

  ### Function to set mode
  def rbx_set_mode(self,new_mode_ind):
    time.sleep(1)
    if new_mode_ind < 0 or new_mode_ind > (len(self.RBX_MODE_FUNCTIONS)-1):
      rospy.loginfo("No matching rbx mode found")
    else:
      self.rbx_status.process_current = self.RBX_MODES[new_mode_ind]
      if self.RBX_MODES[new_mode_ind] != "RESUME":
        self.rbx_mode_last = self.rbx_status.mode # Don't update last on resume
      rospy.loginfo("Setting rbx mode to : " + self.RBX_MODES[new_mode_ind])
      rospy.loginfo("Calling rbx mode function: " + self.RBX_MODE_FUNCTIONS[new_mode_ind])
      set_mode_function = globals()[self.RBX_MODE_FUNCTIONS[new_mode_ind]]
      success = set_mode_function(self)
      self.rbx_status.process_last = self.RBX_MODES[new_mode_ind]
      self.rbx_status.process_current = "None"
      if success:
        self.rbx_status.mode = new_mode_ind

  ### Callback to set home
  def rbx_set_home_current_callback(self,set_home_msg):
    rospy.loginfo("*******************************")
    rospy.loginfo("Received set home message")
    rospy.loginfo(set_home_msg)
    self.sethome_current()
    
  ### Callback to start rbx set move speed process
  def rbx_set_move_speed_callback(self,move_speed_msg):
    rospy.loginfo("*******************************")
    rospy.loginfo("Received set move speed message")
    rospy.loginfo(move_speed_msg)
    self.rbx_status.move_speed = move_speed_msg.data

  ### Callback to start rbx set rotate speed process
  def rbx_set_rotate_speed_callback(self,rotate_speed_msg):
    rospy.loginfo("*******************************")
    rospy.loginfo("Received set rotate speed message")
    rospy.loginfo(rotate_speed_msg)
    self.rbx_status.rotate_speed = rotate_speed_msg.data

  ### Callback to start rbx set goto goals process
  def rbx_set_goto_goals_callback(self,goto_goals_msg):
    rospy.loginfo("*******************************")
    rospy.loginfo("Received set goals message")
    rospy.loginfo(goto_goals_msg)
    goto_goals_list=list(goto_goals_msg.data)
    if len(goto_goals_list) != 3:
      rospy.loginfo("Ignoring this Request")
      rospy.loginfo("Messge is wrong length. Should be float list of size 3")
    else:
      self.rbx_status.error_bounds = goto_goals_list

  ### Callback to set cmd timeout
  def rbx_set_cmd_timeout_callback(self,cmd_timeout_msg):
    rospy.loginfo("*******************************")
    rospy.loginfo("Received set timeout message")
    rospy.loginfo(cmd_timeout_msg)
    self.rbx_status.cmd_timeout=cmd_timeout_msg.data

  ### Callback to set height goal on takeoff action
  def rbx_set_takeoff_m_callback(self,takeoff_m_msg):
    rospy.loginfo("*******************************")
    rospy.loginfo("Received set takeoff meters height message")
    rospy.loginfo(takeoff_m_msg)
    self.takeoff_m=takeoff_m_msg.data


  ##############################
  # RBX Control Topic Callbacks

  ### Callback to execute action
  def rbx_go_action_callback(self,action_msg):
    rospy.loginfo("*******************************")
    rospy.loginfo("Received go action message")
    rospy.loginfo(action_msg)
    self.rbx_cmd_success_current = False
    self.update_current_errors( [0,0,0,0,0,0,0] )
    action_ind = action_msg.data
    if action_ind < 0 or action_ind > (len(self.RBX_ACTION_FUNCTIONS)-1):
      rospy.loginfo("No matching rbx action found")
    else:
      self.rbx_status.process_current = self.RBX_ACTIONS[action_ind]
      self.rbx_status.ready = False
      rospy.loginfo("Starting action: " + self.rbx_status.process_current)
      set_action_function = globals()[self.RBX_ACTION_FUNCTIONS[action_ind]]
      self.rbx_cmd_success_current = set_action_function(self,self.rbx_status.cmd_timeout)
      rospy.loginfo("Finished action: " + self.rbx_status.process_current)
      time.sleep(1)
      self.rbx_status.ready = True
      self.rbx_status.process_last = self.RBX_ACTIONS[action_ind]
      self.rbx_status.process_current = "None"
    self.rbx_status.cmd_success = self.rbx_cmd_success_current

  ### Callback to start rbx go home
  def rbx_go_home_callback(self,home_msg):
    self.rbx_status.process_current = "Go Home"
    self.rbx_cmd_success_current = False
    self.rbx_status.ready = False
    rospy.loginfo("*******************************")
    rospy.loginfo("Received go home message")
    rospy.loginfo(home_msg)
    self.rtl()
    self.rbx_cmd_success_current = True
    time.sleep(1)
    self.rbx_status.ready = True
    self.rbx_status.process_last = self.rbx_status.process_current
    self.rbx_status.process_current = "None"
    self.rbx_status.cmd_success = self.rbx_cmd_success_current

  ### Callback to start rbx stop
  def rbx_go_stop_callback(self,stop_msg):
    self.rbx_status.process_current = "Stop"
    self.rbx_cmd_success_current = False
    self.rbx_status.ready = False
    self.update_current_errors( [0,0,0,0,0,0,0] )
    rospy.loginfo("*******************************")
    rospy.loginfo("Received go stop message")
    rospy.loginfo(stop_msg)
    self.loiter()
    self.rbx_cmd_success_current = True
    time.sleep(1)
    self.rbx_status.ready = True
    self.rbx_status.process_last = self.rbx_status.process_current
    self.rbx_status.process_current = "None"
    self.rbx_status.cmd_success = self.rbx_cmd_success_current

  ### Callback to start rbx goto pose process
  def rbx_goto_pose_callback(self,pose_cmd_msg):
    self.rbx_status.process_current = "GoTo Pose"
    self.rbx_cmd_success_current = False
    self.update_current_errors( [0,0,0,0,0,0,0] )
    rospy.loginfo("*******************************")
    rospy.loginfo("Recieved GoTo Pose Message")
    rospy.loginfo("")
    rospy.loginfo(pose_cmd_msg)
    setpoint_data=list(pose_cmd_msg.data)
    if len(setpoint_data) != 3:
      rospy.loginfo("Ignoring this Request")
      rospy.loginfo("Messge is wrong length. Should be float list of size 3")
      rospy.loginfo("[Roll,Pitch,Yaw]")
    else:
      if self.rbx_status.ready is False:
        rospy.loginfo("Another GoTo Command Process is Active")
        rospy.loginfo("Ignoring this Request")
      else:
        self.rbx_status.ready = False
        self.rbx_cmd_success_current = self.setpoint_attitude_ned(setpoint_data,self.rbx_status.cmd_timeout)
        self.rbx_status.ready = True
    self.rbx_status.process_last = self.rbx_status.process_current
    self.rbx_status.process_current = "None"
    self.rbx_status.cmd_success = self.rbx_cmd_success_current


  ### Callback to start rbx goto position process
  def rbx_goto_position_callback(self,position_cmd_msg):
    self.rbx_status.process_current = "GoTo Position"
    self.rbx_cmd_success_current = False
    self.update_current_errors( [0,0,0,0,0,0,0] )
    rospy.loginfo("*******************************")
    rospy.loginfo("Recieved GoTo Position Command Message")
    rospy.loginfo("")
    rospy.loginfo(position_cmd_msg)
    setpoint_data=list(position_cmd_msg.data)
    if len(setpoint_data) != 4:
      rospy.loginfo("Ignoring this Request")
      rospy.loginfo("Messge is wrong length. Should be float list of size 4")
      rospy.loginfo("[X,Y,Z,Yaw]")
    else:    
      if self.rbx_status.ready is False:
        rospy.loginfo("Another GoTo Command Process is Active")
        rospy.loginfo("Ignoring this Request")
      else:
        self.rbx_status.ready = False
        self.rbx_cmd_success_current = self.setpoint_position_local_body(setpoint_data,self.rbx_status.cmd_timeout)
        self.rbx_status.ready = True
    self.rbx_status.process_last = self.rbx_status.process_current
    self.rbx_status.process_current = "None"
    self.rbx_status.cmd_success = self.rbx_cmd_success_current


  ### Callback to start rbx goto location subscriber
  def rbx_goto_location_callback(self,location_cmd_msg):
    self.rbx_status.process_current = "GoTo Location"
    self.rbx_cmd_success_current = False
    self.update_current_errors( [0,0,0,0,0,0,0] )
    rospy.loginfo("*******************************")
    rospy.loginfo("Recieved GoTo Location Message")
    rospy.loginfo("")
    rospy.loginfo(location_cmd_msg)
    setpoint_data=list(location_cmd_msg.data)
    if len(setpoint_data) != 4:
      rospy.loginfo("Ignoring this Request")
      rospy.loginfo("Messge is wrong length. Should be float list of size 4")
      rospy.loginfo("[Lat,Long,Alt,Yaw]")
    else:
      if self.rbx_status.ready is False:
        rospy.loginfo("Another GoTo Command Process is Active")
        rospy.loginfo("Ignoring this Request")
      else:
        self.rbx_status.ready = False
        self.rbx_cmd_success_current = self.setpoint_location_global_wgs84(setpoint_data,self.rbx_status.cmd_timeout)
        self.rbx_status.ready = True
    self.rbx_status.process_last = self.rbx_status.process_current
    self.rbx_status.process_current = "None"
    self.rbx_status.cmd_success = self.rbx_cmd_success_current


  #######################
  # Mavlink Interface Methods

  ### Callback to get current state message
  def get_state_callback(self,mavlink_state_msg):
    self.mavlink_state = mavlink_state_msg
    # Update rbx state value
    arm_val = mavlink_state_msg.armed
    if arm_val == True:
      self.rbx_status.state=1
    else:
      self.rbx_status.state=0
    # Update rbx mode value
    mode_val = mavlink_state_msg.mode
    mode_ind=-1
    for ind, mode in enumerate(self.RBX_MODES):
      if mode == mode_val:
        mode_ind=ind
    self.rbx_status.mode=mode_ind   


  ### Callback to get current mavlink battery message
  def get_mavlink_battery_callback(self,battery_msg):
    self.rbx_battery = battery_msg.percentage


  ### Function to set mavlink armed state
  def set_mavlink_arm_state(self,arm_value):
    arm_cmd = CommandBoolRequest()
    arm_cmd.value = arm_value
    rospy.loginfo("Updating armed")
    rospy.loginfo(arm_value)
    self.rbx_status.ready = False
    time.sleep(1) # Give time for other process to see busy
    while self.mavlink_state.armed != arm_value and not rospy.is_shutdown():
      time.sleep(.25)
      self.arming_client.call(arm_cmd)
      rospy.loginfo("Waiting for armed value to set")
      rospy.loginfo("Set Value: " + str(arm_value))
      rospy.loginfo("Cur Value: " + str(self.mavlink_state.armed))
    self.rbx_status.ready = True

  ### Function to set mavlink mode
  def set_mavlink_mode(self,mode_new):
    new_mode = SetModeRequest()
    new_mode.custom_mode = mode_new
    rospy.loginfo("Updating mode")
    rospy.loginfo(mode_new)
    self.rbx_status.ready = False
    time.sleep(1) # Give time for other process to see busy
    while self.mavlink_state.mode != mode_new and not rospy.is_shutdown():
      time.sleep(.25)
      self.mode_client.call(new_mode)
      rospy.loginfo("Waiting for mode to set")
      rospy.loginfo("Set Value: " + mode_new)
      rospy.loginfo("Cur Value: " + str(self.mavlink_state.mode))
    self.rbx_status.ready = True






  ### Function to set and check setpoint attitude NED command
  ###################################################
  # Input is [ROLL_NED_DEG, PITCH_NED_DEG, YEW_NED_DEGREES]
  # Converted to ENU befor sending message
  ###################################################
  def setpoint_attitude_ned(self,setpoint_attitude,timeout_sec=CMD_TIMEOUT_SEC):
    # setpoint_attitude is [ROLL_NED_DEG, PITCH_NED_DEG, YEW_NED_DEGREES]
    # Use value -999 to use current value
    cmd_success = True
    self.update_current_errors( [0,0,0,0,0,0,0] )
    rospy.loginfo("Starting Setpoint Attitude Create-Send-Check Process")
    ##############################################
    # Capture Current NavPose Data
    ##############################################
    start_orientation_ned_degs=list(self.current_orientation_ned_degs)
    rospy.loginfo('')
    rospy.loginfo("Attitude Current NED Degrees")
    rospy.loginfo(" Roll, Pitch, Yaw")
    rospy.loginfo(["%.2f" % start_orientation_ned_degs[0],"%.2f" % start_orientation_ned_degs[1],"%.2f" % start_orientation_ned_degs[2]])
    ##############################################
    # Condition Inputs
    ##############################################
    input_attitude_ned_degs = list(setpoint_attitude)
    rospy.loginfo('')
    rospy.loginfo("Attitude Input NED Degrees")
    rospy.loginfo(" Roll, Pitch, Yaw")
    rospy.loginfo(["%.2f" % input_attitude_ned_degs[0],"%.2f" % input_attitude_ned_degs[1],"%.2f" % input_attitude_ned_degs[2]])
    # Set new attitude in degs NED
    new_attitude_ned_degs=list(start_orientation_ned_degs) # Initialize with start values
    for ind in range(3): # Overwrite current with new if set and valid
      if setpoint_attitude[ind] != -999:
        new_attitude_ned_degs[ind]=setpoint_attitude[ind]
      # Condition to +-180 deg
      if new_attitude_ned_degs[ind] > 180:
        new_attitude_ned_degs[ind] = new_attitude_ned_degs[ind] - 360
    rospy.loginfo('')
    rospy.loginfo("Attitude Input Conditioned NED Degrees")
    rospy.loginfo(" Roll, Pitch, Yaw")
    rospy.loginfo(["%.2f" % new_attitude_ned_degs[0],"%.2f" % new_attitude_ned_degs[1],"%.2f" % new_attitude_ned_degs[2]])
    ##############################################
    # Convert NED attitude to Pose
    ##############################################
    # Convert to ROS ENU attitude degs and create ENU quaternion setpoint attitude goal
    yaw_enu_deg = nepi_navpose.convert_yaw_ned2enu(new_attitude_ned_degs[2])
    new_attitude_enu_degs = [new_attitude_ned_degs[0],new_attitude_ned_degs[1],yaw_enu_deg]
    rospy.loginfo('')
    rospy.loginfo("Attitude Goal ENU Degrees")
    rospy.loginfo(" Roll, Pitch, Yaw")
    rospy.loginfo(["%.2f" % new_attitude_enu_degs[0],"%.2f" % new_attitude_enu_degs[1],"%.2f" % new_attitude_enu_degs[2]])
    new_attitude_enu_quat = nepi_navpose.convert_rpy2quat(new_attitude_enu_degs)
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
    rospy.loginfo('')
    rospy.loginfo("Creating Message")
    attitude_target_msg = AttitudeTarget()
    attitude_target_msg.orientation = new_orientation_enu_quat
    attitude_target_msg.body_rate = body_rate
    attitude_target_msg.type_mask = type_mask
    attitude_target_msg.thrust = thrust_ratio
    rospy.loginfo('')
    rospy.loginfo("Setpoint Goal Attitude ENU Message")
    rospy.loginfo(attitude_target_msg)
    ##############################################
    ## Send Setpoint Message and Check for Success
    ##############################################
    rospy.loginfo('')
    rospy.loginfo("Sending Setpoint Attitude Command at 50 Hz and")
    rospy.loginfo("Waiting for Attitude Setpoint to complete")
    setpoint_attitude_reached = False
    stabilize_timer=0
    timeout_timer = 0 # Initialize timeout timer
    print_timer = 0
    attitude_errors = [] # Initialize running list of errors
    while setpoint_attitude_reached is False and not rospy.is_shutdown():  # Wait for setpoint goal to be set
      if timeout_timer > timeout_sec:
        rospy.loginfo("Setpoint cmd timed out")
        cmd_success = False
        break
      time2sleep = 0.02
      time.sleep(time2sleep) # update setpoint position at 50 Hz
      stabilize_timer=stabilize_timer+time2sleep # Increment rospy.loginfo message timer
      timeout_timer = timeout_timer+time2sleep
      self.setpoint_attitude_pub.publish(attitude_target_msg) # Publish Setpoint
      # Calculate setpoint attitude errors
      cur_attitude_ned_degs = [self.current_orientation_ned_degs[0],self.current_orientation_ned_degs[1],self.current_orientation_ned_degs[2]]
      attitude_errors_degs = np.array(new_attitude_ned_degs) - np.array(cur_attitude_ned_degs)
      for ind in range(3):
        if input_attitude_ned_degs[ind] == -999.0: # Ignore error check if set to current
          attitude_errors_degs[ind]=0.0
      max_attutude_error_deg = max(abs(attitude_errors_degs))
      # Check for setpoint position local point goal
      if  setpoint_attitude_reached is False:
        if stabilize_timer > self.rbx_status.error_bounds.max_stabilize_time_s:
          max_attitude_errors = max(attitude_errors) # Get max from error window
          attitude_errors = [max_attutude_error_deg] # reset running list of errors
        print_timer = print_timer + time2sleep
        if print_timer > 1:
          print_timer = 0
          rospy.loginfo("")
          rospy.loginfo("Goto Pose Updates")
          # rospy.loginfo some information
          rospy.loginfo('')
          rospy.loginfo("Current Attitude NED Degrees")
          rospy.loginfo(" Roll, Pitch, Yaw")
          rospy.loginfo(["%.2f" % self.current_orientation_ned_degs[0],"%.2f" % self.current_orientation_ned_degs[1],"%.2f" % self.current_orientation_ned_degs[2]])
          rospy.loginfo('')
          rospy.loginfo("Current Goal NED Degrees")
          rospy.loginfo(["%.2f" % new_attitude_ned_degs[0],"%.2f" % new_attitude_ned_degs[1],"%.2f" % new_attitude_ned_degs[2]])
          rospy.loginfo('')
          rospy.loginfo("Current Attitude Errors")
          rospy.loginfo(["%.3f" % attitude_errors_degs[0],"%.3f" % attitude_errors_degs[1],"%.3f" % attitude_errors_degs[2]])
          rospy.loginfo("Max Error from Stabilized Check Window Meters")
          rospy.loginfo(["%.2f" % max_attitude_errors])
          if max_attitude_errors < self.rbx_status.error_bounds.max_rotation_error_deg:
            rospy.loginfo('')
            rospy.loginfo("Attitude Setpoint Reached")
            setpoint_attitude_reached = True
        else:
          attitude_errors.append(max_attutude_error_deg) # append last
      # Reset rospy.loginfo timer if past
      if stabilize_timer > GOTO_STABILIZED_SEC:
        stabilize_timer=0 # Reset rospy.loginfo timer
      self.update_current_errors( [0,0,0,0,attitude_errors_degs[0],attitude_errors_degs[1],attitude_errors_degs[2]]  )
    if cmd_success:
      rospy.loginfo("************************")
      rospy.loginfo("Setpoint Reached")
    self.update_current_errors( [0,0,0,0,0,0,0] )
    self.update_prev_errors( [0,0,0,0,attitude_errors_degs[0],attitude_errors_degs[1],attitude_errors_degs[2]] )
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
  def setpoint_position_local_body(self,setpoint_position,timeout_sec=CMD_TIMEOUT_SEC):
    # setpoint_position is [X_BODY_METERS, Y_BODY_METERS, Z_BODY_METERS, YEW_BODY_DEGREES]
    # use value 0 for no change
    cmd_success = True
    self.update_current_errors( [0,0,0,0,0,0,0] )
    rospy.loginfo('')
    rospy.loginfo("Starting Setpoint Position Local Create-Send-Check Process")
    ##############################################
    # Capture Current NavPose Data
    ##############################################
    start_geopoint_wgs84 = list(self.current_location_wgs84_geo)
    rospy.loginfo('')
    rospy.loginfo("Start Location WSG84 geopoint")
    rospy.loginfo(" Lat, Long, Alt")
    rospy.loginfo(["%.2f" % start_geopoint_wgs84[0],"%.2f" % start_geopoint_wgs84[1],"%.2f" % start_geopoint_wgs84[2]])
    start_position_ned_m = list(self.current_position_ned_m)
    rospy.loginfo('')
    rospy.loginfo("Start Position NED degs")
    rospy.loginfo(" X, Y, Z")
    rospy.loginfo(["%.2f" % start_position_ned_m[0],"%.2f" % start_position_ned_m[1],"%.2f" % start_position_ned_m[2]])   
    start_orientation_ned_degs=list(self.current_orientation_ned_degs)
    rospy.loginfo('')
    rospy.loginfo("Start Orientation NED degs")
    rospy.loginfo(" Roll, Pitch, Yaw")
    rospy.loginfo(["%.2f" % start_orientation_ned_degs[0],"%.2f" % start_orientation_ned_degs[1],"%.2f" % start_orientation_ned_degs[2]])
    rospy.loginfo('')
    start_yaw_ned_deg = start_orientation_ned_degs[2]
    rospy.loginfo('')
    rospy.loginfo("Start Yaw NED degs")
    rospy.loginfo(start_yaw_ned_deg) 
    start_heading_deg=self.current_heading_deg
    rospy.loginfo('')
    rospy.loginfo("Start Heading degs")
    rospy.loginfo(start_heading_deg)   
    ##############################################
    # Condition Body Input Data
    ##############################################
    # Condition Point Input
    input_point_body_m=setpoint_position[0:3]
    rospy.loginfo('')
    rospy.loginfo("Point Input Body Meters")
    rospy.loginfo(" X, Y, Z")
    rospy.loginfo(["%.2f" % input_point_body_m[0],"%.2f" % input_point_body_m[1],"%.2f" % input_point_body_m[2]])
    new_point_body_m=list(input_point_body_m) # No conditioning required
    rospy.loginfo('')
    rospy.loginfo("Point Conditioned Body Meters")
    rospy.loginfo(" X, Y, Z")
    rospy.loginfo(["%.2f" % new_point_body_m[0],"%.2f" % new_point_body_m[1],"%.2f" % new_point_body_m[2]])
    # Condition Orienation Input
    input_yaw_body_deg = setpoint_position[3]
    rospy.loginfo('')
    rospy.loginfo("Yaw Input Body Degrees")
    rospy.loginfo(["%.2f" % input_yaw_body_deg])
    new_yaw_body_deg = input_yaw_body_deg
    # Condition to +-180 deg
    if new_yaw_body_deg > 180:
      new_yaw_body_deg = new_yaw_body_deg - 360
    rospy.loginfo('')
    rospy.loginfo("Yaw Input Conditioned Body Degrees")
    rospy.loginfo(["%.2f" % new_yaw_body_deg])      
    ##############################################
    # Convert Body Data to NED Data
    ##############################################
    # Set new yaw orientation in NED degrees
    offset_ned_m = nepi_navpose.convert_point_body2ned(new_point_body_m,start_yaw_ned_deg)
    rospy.loginfo('')
    rospy.loginfo("Point Goal Offsets NED Meters")
    rospy.loginfo(" X, Y, Z")
    rospy.loginfo(["%.2f" % offset_ned_m[0],"%.2f" % offset_ned_m[1],"%.2f" % offset_ned_m[2]])
    new_x_ned_m = start_position_ned_m[0] + offset_ned_m[0]
    new_y_ned_m = start_position_ned_m[1] + offset_ned_m[1]
    new_z_ned_m = start_position_ned_m[2] + offset_ned_m[2]
    new_point_ned_m = [new_x_ned_m,new_y_ned_m,new_z_ned_m]
    rospy.loginfo('')
    rospy.loginfo("Point Goal NED Meters")
    rospy.loginfo(" X, Y, Z")
    rospy.loginfo(["%.2f" % new_point_ned_m[0],"%.2f" % new_point_ned_m[1],"%.2f" % new_point_ned_m[2]])
    new_yaw_ned_deg = nepi_navpose.convert_yaw_body2ned(new_yaw_body_deg,start_yaw_ned_deg)
    rospy.loginfo('')
    rospy.loginfo("Yaw Goal NED Degrees")
    rospy.loginfo(["%.2f" % new_yaw_ned_deg])
    ##############################################
    # Convert NED Data to ENU Data
    ##############################################
    # New Point ENU in meters
    new_point_enu_m=Point()
    new_point_enu_m.x = new_point_ned_m[1]
    new_point_enu_m.y = new_point_ned_m[0]
    new_point_enu_m.z = - new_point_ned_m[2]
    rospy.loginfo('')
    rospy.loginfo("Point Goal ENU Meters")
    rospy.loginfo(" X, Y, Z")
    rospy.loginfo(["%.2f" % new_point_enu_m.x,"%.2f" % new_point_enu_m.y,"%.2f" % new_point_enu_m.z])
    new_yaw_enu_deg = nepi_navpose.convert_yaw_ned2enu(new_yaw_ned_deg)
    rospy.loginfo('')
    rospy.loginfo("Yaw Goal ENU Degrees")
    rospy.loginfo(["%.2f" % new_yaw_enu_deg])
    ##############################################
    # Create Local ENU Position and Orienation Setpoint Values
    ##############################################
    # New Local Position ENU in meters
    new_point_enu_m=Point()
    new_point_enu_m.x = new_point_enu_m.x
    new_point_enu_m.y = new_point_enu_m.y
    new_point_enu_m.z = new_point_enu_m.z
    rospy.loginfo('')
    rospy.loginfo("Position Goal ENU Meters")
    rospy.loginfo(" X, Y, Z")
    rospy.loginfo(["%.2f" % new_point_enu_m.x,"%.2f" % new_point_enu_m.y,"%.2f" % new_point_enu_m.z])
    # New Local Orienation ENU in meters  
    new_orientation_enu_deg = [start_orientation_ned_degs[0],start_orientation_ned_degs[1],new_yaw_enu_deg]
    rospy.loginfo('')
    rospy.loginfo("Orienation Goal ENU Degrees")
    rospy.loginfo(" Roll, Pitch, Yaw")
    rospy.loginfo(["%.2f" % new_orientation_enu_deg[0],"%.2f" % new_orientation_enu_deg[1],"%.2f" % new_orientation_enu_deg[2]])
    new_orientation_enu_q = nepi_navpose.convert_rpy2quat(new_orientation_enu_deg)
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
    rospy.loginfo('')
    rospy.loginfo("Setpoint Goal Position Local Message")
    rospy.loginfo(position_local_target_msg)
    ##############################################
    ## Send Message and Check for Setpoint Success
    ##############################################
    rospy.loginfo('')
    rospy.loginfo("Sending Setpoint Position Local Command at 50 Hz and")
    rospy.loginfo("Waiting for Attitude Setpoint to complete")
    setpoint_position_local_point_reached = False
    setpoint_position_local_yaw_reached = False
    stabilize_timer=0
    point_errors = [] # Initialize running list of errors
    yaw_errors = [] # Initialize running list of errors
    timeout_timer = 0 # Initialize timeout timer
    print_timer_1 = 0
    print_timer_2 = 0
    while setpoint_position_local_point_reached is False or setpoint_position_local_yaw_reached is False and not rospy.is_shutdown():  # Wait for setpoint goal to be set
      if timeout_timer > timeout_sec:
        rospy.loginfo("Setpoint cmd timed out")
        cmd_success = False
        break
      time2sleep = 0.02
      time.sleep(time2sleep) # update setpoint position at 50 Hz
      stabilize_timer=stabilize_timer+time2sleep # Increment rospy.loginfo message timer
      timeout_timer = timeout_timer+time2sleep
      self.setpoint_position_local_pub.publish(position_local_target_msg) # Publish Setpoint
      # Calculate setpoint position ned errors    
      point_ned_error_m = np.array(self.current_position_ned_m) - np.array(new_point_ned_m)
      for ind in range(3):
        if input_point_body_m == -999: # Ignore error check if set to current
          point_ned_error_m[ind] = 0
      max_point_ned_error_m = np.max(np.abs(point_ned_error_m))
      # Calculate setpoint yaw ned error
      if input_yaw_body_deg == -999: # Ignore error check if set to current
        setpoint_position_local_yaw_reached = True
        max_yaw_ned_error_deg = 0
      else:
        cur_yaw_ned_deg = self.current_orientation_ned_degs[2]
        yaw_ned_error_deg =  cur_yaw_ned_deg - new_yaw_ned_deg
        max_yaw_ned_error_deg = abs(yaw_ned_error_deg)
      # Check for setpoint position local point goal
      if  setpoint_position_local_point_reached is False:
        if stabilize_timer > GOTO_STABILIZED_SEC:
          max_point_errors = max(point_errors) # Get max from error window
          point_errors = [max_point_ned_error_m] # reset running list of errors
        print_timer_1 = print_timer_1 + time2sleep
        if print_timer_1 > 1:
          print_timer_1 = 0
          rospy.loginfo("Goto Position Position Updates")
          # rospy.loginfo some information every second
          rospy.loginfo('')
          rospy.loginfo("Current Position NED Meters")
          rospy.loginfo(" X, Y, Z")
          rospy.loginfo(["%.2f" % self.current_position_ned_m[0],"%.2f" % self.current_position_ned_m[1],"%.2f" % self.current_position_ned_m[2]])
          rospy.loginfo("Current Goal NED Meters")
          rospy.loginfo(" X, Y, Z")
          rospy.loginfo(["%.2f" % new_point_ned_m[0],"%.2f" % new_point_ned_m[1],"%.2f" % new_point_ned_m[2]])
          rospy.loginfo("Current Errors Meters")
          rospy.loginfo(" X, Y, Z")
          rospy.loginfo(["%.2f" % point_ned_error_m[0],"%.2f" % point_ned_error_m[1],"%.2f" % point_ned_error_m[2]])
          rospy.loginfo("Max Error from Stabilized Check Window Meters")
          rospy.loginfo(["%.2f" % max_point_errors])
          if max_point_errors < self.rbx_status.error_bounds.max_distance_error_m:
            rospy.loginfo('')
            rospy.loginfo("Position Setpoint Reached")
            setpoint_position_local_point_reached = True
        else:
          point_errors.append(max_point_ned_error_m) # append last
      # Check for setpoint position yaw point goal
      if  setpoint_position_local_yaw_reached is False:
        if stabilize_timer > self.rbx_status.error_bounds.max_stabilize_time_s:
          max_yaw_errors = max(yaw_errors) # Get max from error window
          yaw_errors = [max_yaw_ned_error_deg] # reset running list of errors
        print_timer_2 = print_timer_2 + time2sleep
        if print_timer_2 > 1:
          print_timer_2 = 0
          rospy.loginfo("")
          rospy.loginfo("Goto Position Yaw Updates")
          # rospy.loginfo some information every second
          rospy.loginfo('')
          rospy.loginfo("Current Yaw NED Degrees")
          rospy.loginfo(self.current_orientation_ned_degs[2])
          rospy.loginfo("Current Goal NED Degrees")
          rospy.loginfo(new_yaw_ned_deg)
          rospy.loginfo("Current Error Degree")
          rospy.loginfo(max_yaw_ned_error_deg)
          rospy.loginfo("Max Error from Stabilized Check Window Meters")
          rospy.loginfo(["%.2f" % max_yaw_errors])
          if max_yaw_errors < self.rbx_status.error_bounds.max_rotation_error_deg:
            rospy.loginfo('')
            rospy.loginfo("Yaw Setpoint Reached")
            setpoint_position_local_yaw_reached = True
        else:
          yaw_errors.append(max_yaw_ned_error_deg) # append last
      # Reset rospy.loginfo timer if past
      if stabilize_timer > self.rbx_status.error_bounds.max_stabilize_time_s:
        stabilize_timer=0 # Reset rospy.loginfo timer
      self.update_current_errors(  [point_ned_error_m[0],point_ned_error_m[1],point_ned_error_m[2],0,0,0,max_yaw_ned_error_deg] )
    if cmd_success:
      rospy.loginfo("************************")
      rospy.loginfo("Setpoint Reached")
    self.update_current_errors( [0,0,0,0,0,0,0] )
    self.update_prev_errors(  [point_ned_error_m[0],point_ned_error_m[1],point_ned_error_m[2],0,0,0,max_yaw_ned_error_deg] )
    return cmd_success



  ### Function to set and check setpoint location global geopoint and yaw command
  ###################################################
  # Input is [LAT, LONG, ALT_WGS84, YAW_NED_DEGREES]
  # Converted to AMSL Altitude and ENU Yaw berore sending
  # Altitude is specified as meters above the WGS-84 and converted to AMSL before sending
  # Yaw is specified in NED frame degrees 0-360 or +-180 
  #####################################################
  def setpoint_location_global_wgs84(self,setpoint_location,timeout_sec=CMD_TIMEOUT_SEC):
    # setpoint_location is [LAT, LONG, ALT_WGS84, YEW_NED_DEGREES 0-360 or +-180]
    # Use value -999 to use current value
    cmd_success = True
    self.update_current_errors( [0,0,0,0,0,0,0] )
    rospy.loginfo('')
    rospy.loginfo("Starting Setpoint Location Global Create-Send-Check Process")
    ##############################################
    # Capture Current NavPose Data
    ##############################################
    start_geopoint_wgs84 = list(self.current_location_wgs84_geo)  
    rospy.loginfo('')
    rospy.loginfo("Start Location WSG84 geopoint")
    rospy.loginfo(" Lat, Long, Alt")
    rospy.loginfo(["%.6f" % start_geopoint_wgs84[0],"%.6f" % start_geopoint_wgs84[1],"%.2f" % start_geopoint_wgs84[2]])
    start_orientation_ned_degs=list(self.current_orientation_ned_degs)
    rospy.loginfo('')
    rospy.loginfo("Start Orientation NED degs")
    rospy.loginfo(" Roll, Pitch, Yaw")
    rospy.loginfo(["%.6f" % start_orientation_ned_degs[0],"%.6f" % start_orientation_ned_degs[1],"%.2f" % start_orientation_ned_degs[2]])
    rospy.loginfo('')
    start_yaw_ned_deg = start_orientation_ned_degs[2]
    if start_yaw_ned_deg < 0:
      start_yaw_ned_deg = start_yaw_ned_deg + 360
    rospy.loginfo('')
    rospy.loginfo("Start Yaw NED degs 0-360")
    rospy.loginfo(start_yaw_ned_deg) 
    start_heading_deg=self.current_heading_deg
    rospy.loginfo('')
    rospy.loginfo("Start Heading degs")
    rospy.loginfo(start_heading_deg)
    start_geoid_height_m = self.current_geoid_height_m
    ##############################################
    # Condition NED Input Data
    ##############################################
    # Condition Location Input
    input_geopoint_wgs84 = list(setpoint_location[0:3])
    rospy.loginfo('')
    rospy.loginfo("Location Input Global Geo")
    rospy.loginfo(" Lat, Long, Alt_WGS84")
    rospy.loginfo(["%.8f" % input_geopoint_wgs84[0],"%.8f" % input_geopoint_wgs84[1],"%.2f" % input_geopoint_wgs84[2]])
    new_geopoint_wgs84=list(start_geopoint_wgs84) # Initialize with start
    for ind in range(3): # Overwrite current with new if set and valid
      if input_geopoint_wgs84[ind] != -999:
        new_geopoint_wgs84[ind]=input_geopoint_wgs84[ind]
    rospy.loginfo('')
    rospy.loginfo("Location Input Conditioned Global Geo")
    rospy.loginfo(" Lat, Long, Alt_WGS84")
    rospy.loginfo(["%.8f" % new_geopoint_wgs84[0],"%.8f" % new_geopoint_wgs84[1],"%.2f" % new_geopoint_wgs84[2]])
    # Condition Yaw Input
    input_yaw_ned_deg = setpoint_location[3]
    rospy.loginfo('')
    rospy.loginfo("Yaw Input NED Degrees")
    rospy.loginfo(["%.2f" % input_yaw_ned_deg])
    new_yaw_ned_deg = start_yaw_ned_deg # Initialize to current
    if input_yaw_ned_deg != -999: # Replace if not -999
      new_yaw_ned_deg = input_yaw_ned_deg
    # Condition to 0-360 degs
    if new_yaw_ned_deg < 0:
      new_yaw_ned_deg = new_yaw_ned_deg + 360
    rospy.loginfo('')
    rospy.loginfo("Yaw Input Conditioned NED Degrees 0-360")
    rospy.loginfo(["%.2f" % new_yaw_ned_deg])      
    ##############################################
    # Create Global AMSL Location and NED Orienation Setpoint Values
    ##############################################
    # New Global location ENU in meters
    new_geopoint_amsl=GeoPoint()
    new_geopoint_amsl.latitude = new_geopoint_wgs84[0]
    new_geopoint_amsl.longitude = new_geopoint_wgs84[1]
    new_geopoint_amsl.altitude = new_geopoint_wgs84[2] + start_geoid_height_m
    rospy.loginfo('')
    rospy.loginfo("Location Goal AMSL Meters")
    rospy.loginfo(" Lat, Long, Alt_AMSL")
    rospy.loginfo(["%.8f" % new_geopoint_amsl.latitude,"%.8f" % new_geopoint_amsl.longitude,"%.2f" % new_geopoint_amsl.altitude])
    # New Local Orienation NED in degs  
    new_orientation_ned_deg = [start_orientation_ned_degs[0],start_orientation_ned_degs[1],new_yaw_ned_deg]
    rospy.loginfo('')
    rospy.loginfo("Orienation Goal NED Degrees")
    rospy.loginfo(" Roll, Pitch, Yaw")
    rospy.loginfo(["%.2f" % new_orientation_ned_deg[0],"%.2f" % new_orientation_ned_deg[1],"%.2f" % new_orientation_ned_deg[2]])
    new_orientation_ned_q = nepi_navpose.convert_rpy2quat(new_orientation_ned_deg)
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
    rospy.loginfo('')
    rospy.loginfo("Setpoint Location Goal Message")
    rospy.loginfo(location_global_target_msg)
    ##############################################
    ## Send Message and Check for Setpoint Success
    ##############################################
    rospy.loginfo("Sending MAVLINK Setpoint Position Local Command at 50 Hz and")
    rospy.loginfo(" checking for Setpoint Reached")
    setpoint_location_global_geopoint_reached = False
    setpoint_location_global_yaw_reached = False 
    rospy.loginfo('')
    rospy.loginfo("Waiting for Position Local Setpoint to complete")
    stabilize_timer=0
    geopoint_errors = [] # Initialize running list of errors
    yaw_errors = [] # Initialize running list of errors
    timeout_timer = 0 # Initialize timeout timer
    print_timer_1 = 0
    print_timer_2 = 0
    while setpoint_location_global_geopoint_reached is False or setpoint_location_global_yaw_reached is False and not rospy.is_shutdown(): # Wait for setpoint goal to be set
      if timeout_timer > timeout_sec:
        rospy.loginfo("Setpoint cmd timed out")
        cmd_success = False
        break
      time2sleep = 0.02
      time.sleep(time2sleep) # update setpoint position at 50 Hz
      stabilize_timer=stabilize_timer+time2sleep # Increment rospy.loginfo message timer
      timeout_timer = timeout_timer+time2sleep
      self.setpoint_location_global_pub.publish(location_global_target_msg) # Publish Setpoint
      # Calculate setpoint position and yaw errors
      geopoint_errors_geo = np.array(self.current_location_wgs84_geo) - np.array(new_geopoint_wgs84)
      geopoint_errors_m = [geopoint_errors_geo[0]*111139,geopoint_errors_geo[1]*111139,geopoint_errors_geo[2]]
      for ind in range(3):  # Ignore error check if set to current
        if input_geopoint_wgs84[ind] == -999.0:
          geopoint_errors_m[ind] = 0
      max_geopoint_error_m = np.max(np.abs(geopoint_errors_m))
      if input_yaw_ned_deg == -999: # Ignore error check if set to current
        setpoint_location_global_yaw_reached = True
        max_yaw_ned_error_deg = 0
      else:
        cur_yaw_ned_deg = self.current_orientation_ned_degs[2]
        if cur_yaw_ned_deg < 0:
          cur_yaw_ned_deg = cur_yaw_ned_deg + 360
        yaw_ned_error_deg =  cur_yaw_ned_deg - new_yaw_ned_deg
        max_yaw_ned_error_deg = abs(yaw_ned_error_deg)
      # Check for setpoint position global goal
      if  setpoint_location_global_geopoint_reached is False:
        if stabilize_timer > self.rbx_status.error_bounds.max_stabilize_time_s:
          max_geopoint_errors = max(geopoint_errors) # Get max from error window
          geopoint_errors = [max_geopoint_error_m] # reset running list of errors
        print_timer_1 = print_timer_1 + time2sleep
        if print_timer_1 > 1:
          print_timer_1 = 0
          rospy.loginfo("")
          rospy.loginfo("Goto Location Location Updates")
          rospy.loginfo(self.rbx_status.errors_current)
          # rospy.loginfo some information every second
          rospy.loginfo('')
          rospy.loginfo("Current Location WGS84")
          rospy.loginfo(" Lat, Long, Alt_WGS84")
          rospy.loginfo(["%.7f" % self.current_location_wgs84_geo[0],"%.7f" % self.current_location_wgs84_geo[1],"%.2f" % self.current_location_wgs84_geo[2]])
          rospy.loginfo("Current Goal WGS84")
          rospy.loginfo(" Lat, Long, Alt_WGS84")
          rospy.loginfo(["%.7f" % new_geopoint_wgs84[0],"%.7f" % new_geopoint_wgs84[1],"%.2f" % new_geopoint_wgs84[2]])
          rospy.loginfo("Current Errors Meters")
          rospy.loginfo(" Lat, Long, Alt")
          rospy.loginfo(["%.2f" % geopoint_errors_m[0],"%.2f" % geopoint_errors_m[1],"%.2f" % geopoint_errors_m[2]])
          rospy.loginfo("Max Error from Stabilized Check Window Meters")
          rospy.loginfo(["%.2f" % max_geopoint_errors])
          if max_geopoint_errors < self.rbx_status.error_bounds.max_distance_error_m:
            rospy.loginfo('')
            rospy.loginfo("Location Setpoint Reached")
            setpoint_location_global_geopoint_reached = True
        else:
          geopoint_errors.append(max_geopoint_error_m) # append last
      # Check for setpoint position yaw goal
      if  setpoint_location_global_yaw_reached is False:
        if stabilize_timer > self.rbx_status.error_bounds.max_stabilize_time_s:
          max_yaw_errors = max(yaw_errors) # Get max from error window
          yaw_errors = [max_yaw_ned_error_deg] # reset running list of errors
        print_timer_2 = print_timer_2 + time2sleep
        if print_timer_2 > 1:
          print_timer_2 = 0
          rospy.loginfo("")
          rospy.loginfo("Goto Location Yaw Updates")
          # rospy.loginfo some information every second
          rospy.loginfo('')
          rospy.loginfo("Current Yaw NED Degrees")
          rospy.loginfo(cur_yaw_ned_deg)
          rospy.loginfo("Current Goal NED Degrees")
          rospy.loginfo(new_yaw_ned_deg)
          rospy.loginfo("Current Error Degree")
          rospy.loginfo(max_yaw_ned_error_deg)
          rospy.loginfo("Max Error from Stabilized Check Window Degs")
          rospy.loginfo(["%.2f" % max_yaw_errors])
          if max_yaw_errors < self.rbx_status.error_bounds.max_rotation_error_deg:
            rospy.loginfo('')
            rospy.loginfo("Yaw Setpoint Reached")
            setpoint_location_global_yaw_reached = True
        else:
          yaw_errors.append(max_yaw_ned_error_deg) # append last
      # Reset rospy.loginfo timer if past
      if stabilize_timer > 1:
        stabilize_timer=0 # Reset rospy.loginfo timer
      self.update_current_errors( [geopoint_errors_m[0],geopoint_errors_m[1],geopoint_errors_m[2],0,0,0,max_yaw_ned_error_deg] )
    if cmd_success:
      rospy.loginfo("************************")
      rospy.loginfo("Setpoint Reached")
    self.update_current_errors( [0,0,0,0,0,0,0] )
    self.update_prev_errors( [geopoint_errors_m[0],geopoint_errors_m[1],geopoint_errors_m[2],0,0,0,max_yaw_ned_error_deg] )
    return cmd_success

  #######################
  # Class Utility Functions
  
  ### Function for updating current goto error values
  def update_current_errors(self,error_list):
    if len(error_list) == 7:
      self.rbx_status.errors_current.x_m = error_list[0]
      self.rbx_status.errors_current.y_m = error_list[1]
      self.rbx_status.errors_current.z_m = error_list[2]
      self.rbx_status.errors_current.heading_deg = error_list[3]
      self.rbx_status.errors_current.roll_deg = error_list[4]
      self.rbx_status.errors_current.pitch_deg = error_list[5]
      self.rbx_status.errors_current.yaw_deg = error_list[6]
    else:
      rospy.loginfo("Skipping current error update. Error list to short")

  ### Function for updating last goto error values
  def update_prev_errors(self,error_list):
    if len(error_list) == 7:
      self.rbx_status.errors_prev.x_m = error_list[0]
      self.rbx_status.errors_prev.y_m = error_list[1]
      self.rbx_status.errors_prev.z_m = error_list[2]
      self.rbx_status.errors_prev.heading_deg = error_list[3]
      self.rbx_status.errors_prev.roll_deg = error_list[4]
      self.rbx_status.errors_prev.pitch_deg = error_list[5]
      self.rbx_status.errors_prev.yaw_deg = error_list[6]
    else:
      rospy.loginfo("Skipping current error update. Error list to short")


  #######################
  # Mavlink Ardupilot Interface Methods

  ### Function for switching to arm state
  global arm
  def arm(self):
    self.set_mavlink_arm_state(True)

  ### Function for switching to disarm state
  global disarm
  def disarm(self):
    self.set_mavlink_arm_state(False)

  ## Function for sending takeoff command
  global takeoff
  def takeoff(self,timeout_sec):
    cmd_success = True
    self.update_current_errors( [0,0,0,0,0,0,0] )
    start_alt_m = self.current_location_wgs84_geo[2]
    start_alt_goal = start_alt_m + self.takeoff_m
    rospy.loginfo("Sending Takeoff Command to altitude to " + str(self.takeoff_m) + " meters")
    time.sleep(1) # VERY IMPORTANT - Sleep a bit between declaring a publisher and using it
    self.takeoff_client(min_pitch=TAKEOFF_MIN_PITCH_DEG,altitude=self.takeoff_m)
    rospy.loginfo("Waiting for takeoff process to complete")
    print_timer=0
    stabilize_timer=0
    alt_error_m = (self.current_location_wgs84_geo[2] - start_alt_goal)
    alt_errors = []
    max_alt_errors = self.rbx_status.error_bounds.max_distance_error_m + abs(alt_error_m)
    timeout_timer = 0
    while max_alt_errors > self.rbx_status.error_bounds.max_distance_error_m and not rospy.is_shutdown():
      rospy.loginfo(max_alt_errors)
      time2sleep = 0.2 # takeoff position check rate
      stabilize_timer = stabilize_timer + time2sleep
      if timeout_timer > timeout_sec:
        rospy.loginfo("Takeoff action timed out")
        cmd_success = False
        break
      else:
        timeout_timer = timeout_timer+time2sleep
      time.sleep(time2sleep) 
      print_timer = print_timer+time2sleep
      alt_error_m = self.current_location_wgs84_geo[2] - start_alt_goal
      self.update_current_errors( [0,0,alt_error_m,0,0,0,0] )
      if print_timer > 1:
        print_timer = 0
        rospy.loginfo("Takeoff errors")
        rospy.loginfo(self.rbx_status.errors_current)
        
      if stabilize_timer > self.rbx_status.error_bounds.max_stabilize_time_s:
        max_alt_errors = max(alt_errors)
        rospy.loginfo("Max alt error over stabilized time")
        rospy.loginfo(max_alt_errors)
        alt_errors = abs(alt_error_m) # reinitialize alt_errors list
        stabilize_timer = 0
      else:
        alt_errors.append(abs(alt_error_m)) # append last
    if cmd_success:
      rospy.loginfo("Takeoff action complete")
    self.update_current_errors( [0,0,0,0,0,0,0] )
    self.update_prev_errors( [0,0,alt_error_m,0,0,0,0] )
    return cmd_success


  ### Function for switching to STABILIZE mode
  global stabilize
  def stabilize(self):
    self.set_mavlink_mode('STABILIZE')
    cmd_success = True
    return cmd_success
      
  ### Function for switching to LAND mode
  global land
  def land(self):
    rospy.loginfo("Waiting for land process to complete and disarm")
    while self.rbx_status.state == 1:
      nepi.sleep(1,10)
    rospy.loginfo("Land process complete")
    success = True
    return success


  ### Function for sending go home command
  global rtl
  def rtl(self):
    self.set_mavlink_mode('RTL')
    success = True
    return success


  ### Function for switching to LOITER mode
  global loiter
  def loiter(self):
    self.set_mavlink_mode('LOITER')
    success = True
    return success


  ### Function for switching to Guided mode
  global guided
  def guided(self):
    self.set_mavlink_mode('GUIDED')
    success = True
    return success

  ### Function for switching back to current mission
  global resume
  def resume(self):
    # Reset mode to last
    rospy.loginfo("Switching mavlink mode from " + self.RBX_MODES[self.rbx_status.mode] + " back to " + self.RBX_MODES[self.rbx_mode_last])
    self.rbx_set_mode(self.rbx_mode_last)
    success = True
    return success


  ### Function for sending set home current
  global sethome_current
  def sethome_current(self):
    rospy.loginfo('Sending mavlink set home current command')
    time.sleep(.1) # VERY IMPORTANT - Sleep a bit between declaring a publisher and using it
    self.set_home_client(current_gps=True)
    success = True
    return success




  #######################
  # NEPI NavPose Interface Methods

  ### Setup a regular background navpose get and update navpose data
  def update_current_navpose_callback(self,timer):
    # Get current NEPI NavPose data from NEPI ROS nav_pose_query service call
    try:
      nav_pose_response = self.get_navpose_service(NavPoseQueryRequest())
      #rospy.loginfo(nav_pose_response)
      # Get current navpose
      current_navpose = nav_pose_response.nav_pose
      # Get current heading in degrees
      self.current_heading_deg = nepi_navpose.get_navpose_heading_deg(nav_pose_response)
      # Get current orientation vector (roll, pitch, yaw) in degrees enu frame
      self.current_orientation_enu_degs = nepi_navpose.get_navpose_orientation_enu_degs(nav_pose_response)
      # Get current orientation vector (roll, pitch, yaw) in degrees ned frame +-180
      self.current_orientation_ned_degs = nepi_navpose.get_navpose_orientation_ned_degs(nav_pose_response)
      # Get current position vector (x, y, z) in meters enu frame
      self.current_position_enu_m = nepi_navpose.get_navpose_position_enu_m(nav_pose_response)
      # Get current position vector (x, y, z) in meters ned frame
      self.current_position_ned_m = nepi_navpose.get_navpose_position_ned_m(nav_pose_response)
      # Get current geoid hieght
      self.current_geoid_height_m =  nepi_navpose.get_navpose_geoid_height(nav_pose_response)
      # Get current location vector (lat, long, alt) in geopoint data with WGS84 height
      self.current_location_wgs84_geo =  nepi_navpose.get_navpose_location_wgs84_geo(nav_pose_response) 
      # Get current location vector (lat, long, alt) in geopoint data with AMSL height
      self.current_location_amsl_geo =  nepi_navpose.get_navpose_location_amsl_geo(nav_pose_response)
##      rospy.loginfo("")
##      rospy.loginfo(self.current_geoid_height_m)
##      rospy.loginfo(self.current_location_wgs84_geo)
##      rospy.loginfo(self.current_location_amsl_geo)
      
      
    except Exception as e:
      rospy.loginfo("navpose service call failed: " + str(e))
  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    rospy.loginfo("Shutting down: Executing script cleanup actions")
    self.stabilize() # Change mode


#########################################
# Main
#########################################
if __name__ == '__main__':
  rospy.loginfo("Starting Ardupilot RBX Driver Script")
  rospy.init_node
  rospy.init_node(name= NEPI_RBX_NODENAME)
  #Launch the node
  current_filename = sys.argv[0].split('/')[-1]
  current_filename = current_filename.split('.')[0]
  node_name = current_filename.rpartition("_")[0]
  rospy.loginfo("Launching node named: " + node_name)
  node_class = eval(node_name)
  node = node_class()  
  #Set up node shutdown
  rospy.on_shutdown(node.cleanup_actions)
  # Spin forever (until object is detected)
  rospy.spin()







