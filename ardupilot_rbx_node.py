#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

# Sample NEPI Driver Script. 
# NEPI RBX Driver for Ardupilot Autopilot Systems

### Set the namespace before importing rospy

import os
# ROS namespace setup
NEPI_BASE_NAMESPACE = '/nepi/s2x/'
os.environ["ROS_NAMESPACE"] = NEPI_BASE_NAMESPACE[0:-1]
import rospy

import time
import numpy as np
import math
import tf
import random
import sys
import cv2

from nepi_edge_sdk_base import nepi_ros 
from nepi_edge_sdk_base import nepi_nav

from rbx_robot_if import ROSRBXRobotIF

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Bool, String, Float32, Float64
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped
from geographic_msgs.msg import GeoPoint, GeoPose, GeoPoseStamped
from mavros_msgs.msg import State, AttitudeTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL, CommandHome

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, NavSatFix, BatteryState

from nepi_ros_interfaces.msg import AxisControls

#########################################
# DRIVER SETTINGS
#########################################

##############################

##############################


##############################
# ARDUPILOT Settings

STATUS_UPDATE_RATE_HZ = 20

#########################################
# ROS NAMESPACE SETUP
#########################################



#########################################
# Node Class
#########################################

#class ardupilot_rbx_node(object):
class ArdupilotRBX():
  DEFAULT_NODE_NAME = "ardupilot" # connection port added once discovered

  CAP_SETTINGS = [["Float","takeoff_height_m","0.0","10000.0"],
                  ["Float","takeoff_min_pitch_deg","-90.0","90.0"],
                  ["Float","takeoff_error_bound_m","0","100"],
                  ["Float","takeoff_timeout_s","0","10000"]]

  FACTORY_SETTINGS_OVERRIDES = dict( takeoff_height_m = "5",
                                      takeoff_min_pitch_deg = "10",
                                      takeoff_error_bound_m = "2",
                                      takeoff_timeout_s = "20" )

  
  # RBX State and Mode Dictionaries
  RBX_NAVPOSE_HAS_GPS = True
  RBX_NAVPOSE_HAS_ORIENTATION = True
  RBX_NAVPOSE_HAS_HEADING = True

  RBX_STATES = ["DISARM","ARM"]
  RBX_MODES = ["STABILIZE","LAND","RTL","LOITER","GUIDED","RESUME"]
  RBX_ACTIONS = ["TAKEOFF","LAUNCH"]

  RBX_STATE_FUNCTIONS = ["disarm","arm"]
  RBX_MODE_FUNCTIONS = ["stabilize","land","rtl","loiter","guided","resume"]
  RBX_ACTION_FUNCTIONS = ["takeoff","launch"]

  # Create shared class variables and thread locks 
  
  device_info_dict = dict(node_name = "",
                          sensor_name = "",
                          identifier = "",
                          serial_number = "",
                          hw_version = "",
                          sw_version = "")
                        
  settings_dict = FACTORY_SETTINGS_OVERRIDES 

  axis_controls = AxisControls()
  axis_controls.x = True
  axis_controls.y = True
  axis_controls.z = True
  axis_controls.roll = True
  axis_controls.pitch = True
  axis_controls.yaw = True

  state_ind = 0
  state_current = None
  state_last = None

  mode_ind = 0
  mode_current = None
  mode_last = None

  battery_percent = 0

  mavlink_state = None

  rbx_if = None

  port_id = None

  msg_list = ["","","","","",""]

  takeoff_complete = False
  takeoff_reset_modes = ["LAND","RTL"]

  #######################
  ### Node Initialization
  def __init__(self):


    rospy.loginfo("RBX_ARDU: Starting Initialization Processes")
    self.status_msg_pub = rospy.Publisher("~status_msg", String, queue_size=1)
 
 

    ###################################################
    # MAVLINK Namespace
    # Find Mavlink NameSpace
    mavlink_namespace = rospy.get_name().replace("ardupilot","mavlink")
    self.publishMsg("RBX_ARDU: Waiting for node that includes string: " + mavlink_namespace)
    mavlink_namespace = nepi_ros.wait_for_node(mavlink_namespace)
    MAVLINK_NAMESPACE = (mavlink_namespace + '/')
    self.publishMsg("RBX_ARDU: Found mavlink namespace: " + MAVLINK_NAMESPACE)
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



    ## Start Class Publishers and Subscribers
    # Start a node status msg publisher
    # Wait for MAVLink State topic to publish then subscribe
    self.publishMsg("Waiting for topic: " + MAVLINK_STATE_TOPIC)
    nepi_ros.wait_for_topic(MAVLINK_STATE_TOPIC)
    self.publishMsg("Starting state scubscriber callback")
    rospy.Subscriber(MAVLINK_STATE_TOPIC, State, self.get_state_callback)
    while self.state_current is None and not rospy.is_shutdown():
      self.publishMsg("Waiting for mavlink state status to set")
      time.sleep(0.1)
    while self.mode_current is None and not rospy.is_shutdown():
      self.publishMsg("Waiting for mavlink mode status to set")
      time.sleep(0.1)
    self.publishMsg("Starting State: " + self.state_current)
    self.publishMsg("Starting Mode: " + self.mode_current)

    """     
    # Launch the NEPI ROS RBX node
    node_name = self.DEFAULT_NODE_NAME
    if port_id is not None:
      node_name = node_name + '_' + port_id
    self.publishMsg("")
    self.publishMsg("********************")
    self.publishMsg("Starting " + node_name)
    self.publishMsg("********************")
    self.publishMsg("")
    rospy.init_node(node_name) # Node name could be overridden via remapping
    """


    self.node_name = rospy.get_name().split('/')[-1]
    self.publishMsg(self.node_name + ": ... Connected!")


    ## Define RBX NavPose Publishers
    NEPI_BASE_NAMESPACE = nepi_ros.get_base_namespace()
    NEPI_RBX_NAMESPACE = NEPI_BASE_NAMESPACE + self.node_name + "/rbx/"
    NEPI_RBX_NAVPOSE_GPS_TOPIC = NEPI_RBX_NAMESPACE + "gps_fix"
    NEPI_RBX_NAVPOSE_ODOM_TOPIC = NEPI_RBX_NAMESPACE + "odom"
    NEPI_RBX_NAVPOSE_HEADING_TOPIC = NEPI_RBX_NAMESPACE + "heading"

    self.rbx_navpose_gps_pub = rospy.Publisher(NEPI_RBX_NAVPOSE_GPS_TOPIC, NavSatFix, queue_size=1)
    self.rbx_navpose_odom_pub = rospy.Publisher(NEPI_RBX_NAVPOSE_ODOM_TOPIC, Odometry, queue_size=1)
    self.rbx_navpose_heading_pub = rospy.Publisher(NEPI_RBX_NAVPOSE_HEADING_TOPIC, Float64, queue_size=1)


    ## Define Mavlink Services Calls
    self.set_home_client = rospy.ServiceProxy(MAVLINK_SET_HOME_SERVICE, CommandHome)
    self.mode_client = rospy.ServiceProxy(MAVLINK_SET_MODE_SERVICE, SetMode)
    self.arming_client = rospy.ServiceProxy(MAVLINK_ARMING_SERVICE, CommandBool)
    self.takeoff_client = rospy.ServiceProxy(MAVLINK_TAKEOFF_SERVICE, CommandTOL)


    # Subscribe to MAVLink topics
    rospy.Subscriber(MAVLINK_BATTERY_TOPIC, BatteryState, self.get_mavlink_battery_callback)
    rospy.Subscriber(MAVLINK_SOURCE_GPS_TOPIC, NavSatFix, self.gps_topic_callback)
    rospy.Subscriber(MAVLINK_SOURCE_ODOM_TOPIC, Odometry, self.odom_topic_callback)
    rospy.Subscriber(MAVLINK_SOURCE_HEADING_TOPIC, Float64, self.heading_topic_callback)
    
    ## Define Mavlink Publishers
    self.setpoint_location_global_pub = rospy.Publisher(MAVLINK_SETPOINT_LOCATION_GLOBAL_TOPIC, GeoPoseStamped, queue_size=1)
    self.setpoint_attitude_pub = rospy.Publisher(MAVLINK_SETPOINT_ATTITUDE_TOPIC, AttitudeTarget, queue_size=1)
    self.setpoint_position_local_pub = rospy.Publisher(MAVLINK_SETPOINT_POSITION_LOCAL_TOPIC, PoseStamped, queue_size=1)


    # Initialize settings
    self.cap_settings = self.getCapSettings()
    self.publishMsg("CAPS SETTINGS")
    #for setting in self.cap_settings:
        #self.publishMsg(setting)
    self.factory_settings = self.getFactorySettings()
    self.publishMsg("FACTORY SETTINGS")
    #for setting in self.factory_settings:
        #self.publishMsg(setting)
          

    # Launch the IDX interface --  this takes care of initializing all the camera settings from config. file
    self.publishMsg(self.node_name + ": Launching NEPI IDX (ROS) interface...")
    self.device_info_dict["node_name"] = self.node_name
    self.device_info_dict["device_name"] = self.node_name
    if self.node_name.find("_") != -1:
        split_name = self.node_name.rsplit('_', 1)
        self.device_info_dict["identifier"] = split_name[1] + split_name[1]
    else:
        self.device_info_dict["identifier"] = ""
    
    self.device_info_dict["serial_number"] = ""
    self.device_info_dict["hw_version"] = ""
    self.device_info_dict["sw_version"] = ""

    FAKE_GPS_NAMESPACE = mavlink_namespace.replace("mavlink","fake_gps") + "/"
    rospy.loginfo(FAKE_GPS_NAMESPACE)
    self.rbx_if = ROSRBXRobotIF(device_info = self.device_info_dict,
                                  capSettings = self.cap_settings,
                                  factorySettings = self.factory_settings,
                                  settingUpdateFunction = self.settingUpdateFunction,
                                  getSettingsFunction=self.getSettings,
                                  axisControls = self.axis_controls,
                                  getBatteryPercentFunction = self.getBatteryPercent,
                                  states = self.RBX_STATES,
                                  getStateIndFunction = self.getStateInd,
                                  setStateIndFunction = self.setStateInd,
                                  modes = self.RBX_MODES,
                                  getModeIndFunction = self.getModeInd,
                                  setModeIndFunction = self.setModeInd,
                                  actions = self.RBX_ACTIONS, 
                                  setActionIndFunction = self.setActionInd,
                                  manualControlsReadyFunction = None, #self.manualControlsReady,
                                  getMotorControlRatios=None,
                                  setMotorControlRatio=None,
                                  autonomousControlsReadyFunction = self.autonomousControlsReady,
                                  getHomeFunction=None,setHomeFunction=None,
                                  goHomeFunction = self.goHome, 
                                  goStopFunction = self.goStop, 
                                  gotoPoseFunction = self.gotoPose,
                                  gotoPositionFunction = self.gotoPosition, 
                                  gotoLocationFunction = self.gotoLocation,
                                  gpsTopic = NEPI_RBX_NAVPOSE_GPS_TOPIC,
                                  odomTopic = NEPI_RBX_NAVPOSE_ODOM_TOPIC,
                                  headingTopic = NEPI_RBX_NAVPOSE_HEADING_TOPIC,
                                  fake_gps_namespace = FAKE_GPS_NAMESPACE
                                )


    self.publishMsg(self.node_name + ": ... IDX interface running")
    #updated the system config from the parameters that have been established
    self.rbx_if.updateFromParamServer()
    time.sleep(1)
  
    ## Initiation Complete
    self.publishMsg("RBX Node Initialization Complete")
    #rospy.spin()


  def publishMsg(self,msgStr):

    rospy.loginfo(msgStr)
    self.status_msg_pub.publish(str(msgStr))

  #**********************
  # Setting functions
  def getCapSettings(self):
    return self.CAP_SETTINGS

  def getFactorySettings(self):
    settings = self.getSettings()
    #Apply factory setting overides
    for setting in settings:
      if setting[1] in self.FACTORY_SETTINGS_OVERRIDES:
            setting[2] = self.FACTORY_SETTINGS_OVERRIDES[setting[1]]
            settings = nepi_ros.update_setting_in_settings(setting,settings)
    return settings


  def getSettings(self):
    settings = []
    settings_dict = self.settings_dict # Temp for testing
    if settings_dict is not None:
      for cap_setting in self.cap_settings:
        setting_type = cap_setting[0]
        setting_name = cap_setting[1]
        if setting_name in settings_dict.keys():
          setting_value = settings_dict[setting_name]
          setting = [setting_type,setting_name,str(setting_value)]
          settings.append(setting)
    return settings

  def settingUpdateFunction(self,setting):
    success = False
    setting_str = str(setting)
    if len(setting) == 3:
      setting_type = setting[0]
      setting_name = setting[1]
      [s_name, s_type, data] = nepi_ros.get_data_from_setting(setting)
      #self.publishMsg(type(data))
      if data is not None:
        if setting_name in settings_dict.keys():
          self.settings_dict[setting_name] = setting[2]
          success = True
        else:
          msg = (self.node_name  + " Setting name" + setting_str + " is not supported") 
        if success == True:
          msg = ( self.node_name  + " UPDATED SETTINGS " + setting_str)                  
      else:
        msg = (self.node_name  + " Setting data" + setting_str + " is not valid")
    else:
      msg = (self.node_name  + " Setting " + setting_str + " not correct length")
    return success, msg

  ##########################
  # RBX Interface Functions

  def getStateInd(self):
    return self.state_ind

  def setStateInd(self,state_ind):
    state_last = self.state_current
    set_state_function = globals()[self.RBX_STATE_FUNCTIONS[state_ind]]
    success = set_state_function(self)
    if success:
      self.state_ind = state_ind
      self.state_current = self.RBX_STATES[state_ind]
      self.state_last = state_last
    return success

  def getModeInd(self):
    return self.mode_ind

  def setModeInd(self,mode_ind):
    mode_on_entry = self.mode_current
    set_mode_function = globals()[self.RBX_MODE_FUNCTIONS[mode_ind]]
    success = set_mode_function(self)
    if success:
      if self.RBX_MODES[mode_ind] == "RESUME": 
        if self.RBX_MODES[self.mode_last] != "RESUME":
          self.mode_current = self.mode_last
          self.mode_ind = self.RBX_MODES.index(self.mode_last)
          self.mode_last = mode_on_entry # Don't update last on resume
      else:
        if (mode_ind >= 0 and mode_ind <= (len(self.RBX_MODES)-1)):
          self.mode_ind = mode_ind
          self.mode_current = self.RBX_MODES[mode_ind]
          self.mode_last = mode_on_entry # Don't update last on resume
      #if self.mode_current in self.takeoff_reset_modes:
        #self.takeoff_complete = False
    return success
    
  def getBatteryPercent(self):
    return self.battery_percent

  def setHomeCurrent(self):
    self.sethome_current()

  def setMotorControlRatio(self,motor_ind,speed_ratio):
    pass

  def getMotorControlRatios(self):
    return []

  def setActionInd(self,action_ind):
    set_action_function = globals()[self.RBX_ACTION_FUNCTIONS[action_ind]]
    success = set_action_function(self)
    return success

  def goHome(self):
    self.rtl(self)
    return True

  def goStop(self):
    self.loiter()
    return True

  def gotoPose(self,attitude_enu_degs):
    # Create Setpoint Attitude Message
    attitude_enu_quat = nepi_nav.convert_rpy2quat(attitude_enu_degs)
    orientation_enu_quat = Quaternion()
    orientation_enu_quat.x = attitude_enu_quat[0]
    orientation_enu_quat.y = attitude_enu_quat[1]
    orientation_enu_quat.z = attitude_enu_quat[2]
    orientation_enu_quat.w = attitude_enu_quat[3]
    # Set other setpoint attitude message values
    body_rate = Vector3()
    body_rate.x = 0
    body_rate.y = 0
    body_rate.z = 0
    type_mask = 1|2|4
    thrust_ratio = 0
    attitude_target_msg = AttitudeTarget()
    attitude_target_msg.orientation = orientation_enu_quat
    attitude_target_msg.body_rate = body_rate
    attitude_target_msg.type_mask = type_mask
    attitude_target_msg.thrust = thrust_ratio
    ## Send Setpoint Message
    self.setpoint_attitude_pub.publish(attitude_target_msg) # Publish Setpoint

  def gotoPosition(self,point_enu_m,orientation_enu_deg):
    # Create PoseStamped Setpoint Local ENU Message
    orientation_enu_q = nepi_nav.convert_rpy2quat(orientation_enu_deg)
    orientation_enu_quat = Quaternion()
    orientation_enu_quat.x = orientation_enu_q[0]
    orientation_enu_quat.y = orientation_enu_q[1]
    orientation_enu_quat.z = orientation_enu_q[2]
    orientation_enu_quat.w = orientation_enu_q[3]
    pose_enu=Pose()
    pose_enu.position = point_enu_m
    pose_enu.orientation = orientation_enu_quat
    position_local_target_msg = PoseStamped()
    position_local_target_msg.pose = pose_enu
    ## Send Message and Check for Setpoint Success
    self.setpoint_position_local_pub.publish(position_local_target_msg) # Publish Setpoint

  def gotoLocation(self,geopoint_amsl,orientation_ned_deg):
    # Create GeoPose Setpoint Global AMSL and Yaw NED Message
    orientation_ned_q = nepi_nav.convert_rpy2quat(orientation_ned_deg)
    orientation_ned_quat = Quaternion()
    orientation_ned_quat.x = orientation_ned_q[0]
    orientation_ned_quat.y = orientation_ned_q[1]
    orientation_ned_quat.z = orientation_ned_q[2]
    orientation_ned_quat.w = orientation_ned_q[3]
    geopose_enu=GeoPose()
    geopose_enu.position = geopoint_amsl
    geopose_enu.orientation = orientation_ned_quat
    location_global_target_msg = GeoPoseStamped()
    location_global_target_msg.pose = geopose_enu
    ##############################################
    ## Send Message and Check for Setpoint Success
    ##############################################
    self.setpoint_location_global_pub.publish(location_global_target_msg) # Publish Setpoint

  ##########################
  # Control Ready Check Funcitons

  def manualControlsReady(self):
    ready = False
    if self.mode_ind < len(self.RBX_MODES):
      if self.RBX_MODES[self.mode_ind] == "MANUAL":
        ready = True
    return ready

  def autonomousControlsReady(self):
    ready = False
    if self.RBX_STATES[self.state_ind] == "ARM" and self.RBX_MODES[self.mode_ind] == "GUIDED" and self.takeoff_complete:
      ready = True
    return ready

  ##############################
  # RBX NavPose Topic Publishers
  ### Callback to publish RBX navpose data
  
  def gps_topic_callback(self,navsatfix_msg):
      #Fix Mavros Altitude Error
      if self.rbx_if is None:
        geoid_height_m = 0
      else:
        geoid_height_m = self.rbx_if.current_geoid_height_m
      altitude_wgs84 = navsatfix_msg.altitude - geoid_height_m
      navsatfix_msg.altitude = altitude_wgs84 
      if not rospy.is_shutdown():
        self.rbx_navpose_gps_pub.publish(navsatfix_msg)
      
  ### Callback to publish RBX odom topic
  def odom_topic_callback(self,odom_msg):
      if not rospy.is_shutdown():
        self.rbx_navpose_odom_pub.publish(odom_msg)

  ### Callback to publish RBX heading topic
  def heading_topic_callback(self,heading_msg):
      if not rospy.is_shutdown():
        self.rbx_navpose_heading_pub.publish(heading_msg)


  #######################
  # Mavlink Interface Methods

  ### Callback to get current state message
  def get_state_callback(self,mavlink_state_msg):
    self.mavlink_state = mavlink_state_msg
    # Update state value
    arm_val = mavlink_state_msg.armed
    if arm_val == True:
      self.state_ind=1
    else:
      self.state_ind=0
    self.state_current = self.RBX_STATES[self.state_ind]
    # Update mode value
    mode_val = mavlink_state_msg.mode
    mode_ind=-999
    for ind, mode in enumerate(self.RBX_MODES):
      if mode == mode_val:
        mode_ind=ind
    self.mode_ind=mode_ind 
    if mode_ind >= 0 and mode_ind < len(self.RBX_MODES):
      self.mode_current = self.RBX_MODES[self.mode_ind]
    else:
      self.mode_current = "Undefined"


  ### Function to set mavlink armed state
  def set_mavlink_arm_state(self,arm_value):
    arm_cmd = CommandBoolRequest()
    arm_cmd.value = arm_value
    self.publishMsg("Updating armed")
    self.publishMsg(arm_value)
    time.sleep(1) # Give time for other process to see busy
    self.publishMsg("Waiting for armed value to set to " + str(arm_value))
    while self.mavlink_state.armed != arm_value and not rospy.is_shutdown():
      time.sleep(.25)
      self.arming_client.call(arm_cmd)
      #self.publishMsg("Waiting for armed value to set")
      #self.publishMsg("Set Value: " + str(arm_value))
      #self.publishMsg("Cur Value: " + str(self.mavlink_state.armed))
    self.publishMsg("Armed value set to " + str(arm_value))

  ### Function to set mavlink mode
  def set_mavlink_mode(self,mode_new):
    new_mode = SetModeRequest()
    new_mode.custom_mode = mode_new
    self.publishMsg("Updating mode")
    self.publishMsg(mode_new)
    time.sleep(1) # Give time for other process to see busy
    self.publishMsg("Waiting for mode to set to " + mode_new)
    while self.mavlink_state.mode != mode_new and not rospy.is_shutdown():
      time.sleep(.25)
      self.mode_client.call(new_mode)
      #self.publishMsg("Waiting for mode to set")
      #self.publishMsg("Set Value: " + mode_new)
      #self.publishMsg("Cur Value: " + str(self.mavlink_state.mode))
    self.publishMsg("Mode set to " + mode_new)



  ### Callback to get current mavlink battery message
  def get_mavlink_battery_callback(self,battery_msg):
    self.battery_percent = battery_msg.percentage
 

  #######################
  # Mavlink Ardupilot Interface Methods

  ### Function for switching to arm state
  global arm
  def arm(self):
    self.set_mavlink_arm_state(True)
    success = True
    return success

  ### Function for switching to disarm state
  global disarm
  def disarm(self):
    self.set_mavlink_arm_state(False)
    success = True
    return success

  ## Action Function for setting arm state and sending takeoff command
  global launch
  def launch(self):
    cmd_success = False
    if "guided" in self.RBX_MODE_FUNCTIONS:
      cmd_success = self.setModeInd(self.RBX_MODE_FUNCTIONS.index("guided"))
    if cmd_success:
      if "arm" in self.RBX_STATE_FUNCTIONS:
        cmd_success = self.setStateInd(self.RBX_STATE_FUNCTIONS.index("arm"))
    if cmd_success:
      cmd_success = self.takeoff_action()
    return cmd_success

  ## Function for sending takeoff command
  global takeoff
  def takeoff(self):
    return self.takeoff_action()

  def takeoff_action(self):
    cmd_success = False
    takeoff_height_m = float(self.settings_dict['takeoff_height_m'])
    takeoff_min_pitch_deg = float(self.settings_dict['takeoff_min_pitch_deg'])
    takeoff_error_bound = float(self.settings_dict['takeoff_error_bound_m'])
    takeoff_timeout_s = float(self.settings_dict['takeoff_timeout_s'])
    start_alt = self.rbx_if.current_location_wgs84_geo[2]
    goal_alt = start_alt + takeoff_height_m
    check_interval_s = float(takeoff_timeout_s) / 100
    self.publishMsg("Sending Takeoff Command to altitude to " + str(takeoff_height_m) + " meters")
    time.sleep(1) # VERY IMPORTANT - Sleep a bit between declaring a publisher and using it
    self.takeoff_client(min_pitch=takeoff_min_pitch_deg,altitude=takeoff_height_m)
    check_timer = 0
    check_error = abs(goal_alt - self.rbx_if.current_location_wgs84_geo[2])
    while (check_error > takeoff_error_bound and check_timer < takeoff_timeout_s):
      check_error = abs(goal_alt - self.rbx_if.current_location_wgs84_geo[2])
      time.sleep(check_interval_s)
      check_timer += check_interval_s
    if (check_timer < takeoff_timeout_s):
      cmd_success = True
      self.takeoff_complete = True
      self.publishMsg("Takeoff action completed with error: " + str(check_error) + " meters")
    else:
      self.takeoff_complete = False
      self.publishMsg("Takeoff action failed to complete with error: " + str(check_error) + " meters")
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
    self.set_mavlink_mode('LAND')
    self.publishMsg("Waiting for land process to complete and disarm")
    while self.state_current == "ARMED":
      nepi_ros.sleep(1,10)
    self.publishMsg("Land process complete")
    cmd_success = True
    return cmd_success


  ### Function for sending go home command
  global rtl
  def rtl(self):
    self.set_mavlink_mode('RTL')
    cmd_success = True
    return cmd_success


  ### Function for switching to LOITER mode
  global loiter
  def loiter(self):
    self.set_mavlink_mode('LOITER')
    cmd_success = True
    return cmd_success


  ### Function for switching to Guided mode
  global guided
  def guided(self):
    self.set_mavlink_mode('GUIDED')
    cmd_success = True
    return cmd_success

  ### Function for switching back to current mission
  global resume
  def resume(self):
    # Reset mode to last
    self.publishMsg("Switching mavlink mode from " + self.RBX_MODES[self.mode_current] + " back to " + self.RBX_MODES[self.mode_last])
    self.set_mavlink_mode(self.RBX_MODES[self.mode_last])
    cmd_success = True
    return cmd_success


  ### Function for sending set home current
  global sethome_current
  def sethome_current(self):
    self.publishMsg('Sending mavlink set home current command')
    time.sleep(.1) # VERY IMPORTANT - Sleep a bit between declaring a publisher and using it
    self.set_home_client(current_gps=True)
    cmd_success = True
    return cmd_success



  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    rospy.loginfo("RBX_ARDU: Shutting down: Executing script cleanup actions")


#########################################
# Main
#########################################
if __name__ == '__main__':
  search_name = "mavlink"
  replace_name = "ardupilot"

  rospy.loginfo("Waiting for namespace: " + search_name)
  mavlink_namespace = nepi_ros.wait_for_node(search_name)
  rospy.loginfo("Found namespace: " + mavlink_namespace)
  mavlink_namespace_list = mavlink_namespace.split('/')
  node_name = None
  for name in mavlink_namespace_list:
    if "mavlink" in name:
      node_name = name.replace(search_name,replace_name)
  if node_name is not None:
    rospy.init_node
    rospy.init_node(name = node_name)
    rospy.loginfo("Launching node named: " + node_name)
    node = ArdupilotRBX()  
    #Set up node shutdown
    rospy.on_shutdown(node.cleanup_actions)
    # Spin forever (until object is detected)
    rospy.spin()







