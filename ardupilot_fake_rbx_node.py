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
import copy
import subprocess

from nepi_edge_sdk_base import nepi_ros 
from nepi_edge_sdk_base import nepi_nav
from nepi_edge_sdk_base import nepi_rbx

from nepi_drivers_rbx.rbx_robot_if import ROSRBXRobotIF

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Bool, String, Float32, Float64
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped
from geographic_msgs.msg import GeoPoint, GeoPose, GeoPoseStamped
from mavros_msgs.msg import State, AttitudeTarget
from mavros_msgs.msg import HilGPS, State
from mavros_msgs.srv import CommandHome
from nepi_ros_interfaces.msg import RBXGotoPose, RBXGotoPosition, RBXGotoLocation
from nepi_ros_interfaces.srv import NavPoseQuery, NavPoseQueryRequest


from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL, CommandHome

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, NavSatFix, BatteryState

from nepi_ros_interfaces.msg import AxisControls

sys.path.append("/opt/nepi/ros/lib/nepi_drivers_rbx")

#########################################
# Node Class
#########################################

#class ardupilot_rbx_node(object):
class ArdupilotRBX:
  DEFAULT_NODE_NAME = "ardupilot" # connection port added once discovered

  CAP_SETTINGS = [["Float","takeoff_height_m","0.0","10000.0"],
                  ["Float","takeoff_min_pitch_deg","-90.0","90.0"]]

  FACTORY_SETTINGS_OVERRIDES = dict( takeoff_height_m = "5",
                                      takeoff_min_pitch_deg = "10")

  # RBX State and Mode Dictionaries
  RBX_NAVPOSE_HAS_GPS = True
  RBX_NAVPOSE_HAS_ORIENTATION = False
  RBX_NAVPOSE_HAS_HEADING = False

  RBX_STATES = ["DISARM","ARM"]
  RBX_MODES = ["STABILIZE","LAND","RTL","LOITER","GUIDED","RESUME"]
  RBX_SETUP_ACTIONS = ["TAKEOFF","LAUNCH"]
  RBX_GO_ACTIONS = []

  RBX_STATE_FUNCTIONS = ["disarm","arm"]
  RBX_MODE_FUNCTIONS = ["stabilize","land","rtl","loiter","guided","resume"]
  RBX_SETUP_ACTION_FUNCTIONS = ["takeoff","launch"]  
  RBX_GO_ACTION_FUNCTIONS = []

  # Create shared class variables and thread locks 
  
  device_info_dict = dict(node_name = "",
                          sensor_name = "",
                          identifier = "",
                          serial_number = "",
                          hw_version = "",
                          sw_version = "")
                        
  settings_dict = copy.deepcopy(FACTORY_SETTINGS_OVERRIDES)

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

  home_loc = GeoPoint()
  home_loc.latitude = -999
  home_loc.longitude = -999
  home_loc.altitude = -999
  home_location = home_loc

  stop_triggered = False

  #######################
  ### Node Initialization
  def __init__(self):
    ###################################################
    # Init Ardupilot Node
    rospy.init_node(self.DEFAULT_NODE_NAME)
    self.node_name = rospy.get_name().split("/")[-1]
    rospy.loginfo("RBX_ARDU: Launching node named: " + self.node_name)
    rospy.loginfo("RBX_ARDU: Starting Initialization Processes")
    self.status_msg_pub = rospy.Publisher("~status_msg", String, queue_size=1)


    # Initialize RBX Settings
    self.cap_settings = self.getCapSettings()
    self.publishMsg("CAPS SETTINGS")
    #for setting in self.cap_settings:
        #self.publishMsg(setting)
    self.factory_settings = self.getFactorySettings()
    self.publishMsg("FACTORY SETTINGS")
    #for setting in self.factory_settings:
        #self.publishMsg(setting)

    # Define fake gps namespace
    fake_gps_node = self.node_name.replace("ardupilot","fake_gps")
    FAKE_GPS_NAMESPACE = nepi_ros.get_base_namespace() + fake_gps_node + "/"
    self.publishMsg("Setting up fake_gps at namespace: " + FAKE_GPS_NAMESPACE)
    # Start the an rbx fake gps for this ardupilot node
    base_namespace = nepi_ros.get_base_namespace()
    fgps_node_name = "fake_gps"
    rospy.loginfo("Mavlink_AD: " + "Starting fake gps rbx node: " + fgps_node_name)
    processor_run_cmd = ["rosrun", "nepi_drivers_rbx", "rbx_fake_gps.py",
                          "__name:=" + fgps_node_name, f"__ns:={base_namespace}"]
    fgps_subproc = subprocess.Popen(processor_run_cmd)
    # Start fake gps local publishers
    self.fake_gps_enable_pub = rospy.Publisher(FAKE_GPS_NAMESPACE + "enable", Bool, queue_size=1)
    self.fake_gps_reset_pub = rospy.Publisher(FAKE_GPS_NAMESPACE + "reset", GeoPoint, queue_size=1)
    self.fake_gps_go_stop_pub = rospy.Publisher(FAKE_GPS_NAMESPACE + "go_stop", Empty, queue_size=1)
    self.fake_gps_goto_position_pub = rospy.Publisher(FAKE_GPS_NAMESPACE + "goto_position", Point, queue_size=1)
    self.fake_gps_goto_location_pub = rospy.Publisher(FAKE_GPS_NAMESPACE + "goto_location", GeoPoint, queue_size=1)


    ## Define RBX NavPose Publishers
    NEPI_RBX_NAVPOSE_GPS_TOPIC = FAKE_GPS_NAMESPACE + "gps_fix"
    NEPI_RBX_NAVPOSE_ODOM_TOPIC = None
    NEPI_RBX_NAVPOSE_HEADING_TOPIC = None

    
    time.sleep(1) # Important

    # Launch the IDX interface --  this takes care of initializing all the camera settings from config. file
    self.publishMsg(self.node_name + ": Launching NEPI IDX (ROS) interface...")
    self.device_info_dict["node_name"] = self.node_name
    if self.node_name.find("_") != -1:
        split_name = self.node_name.rsplit('_', 1)
        self.device_info_dict["robot_name"] = split_name[0]
        self.device_info_dict["identifier"] = split_name[1] 
    else:
        self.device_info_dict["robot_name"] = self.node_name
        self.device_info_dict["identifier"] = ""
    
    self.device_info_dict["serial_number"] = ""
    self.device_info_dict["hw_version"] = ""
    self.device_info_dict["sw_version"] = ""
    self.publishMsg(self.device_info_dict)


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
                                  checkStopFunction = self.checkStopFunction,
                                  setup_actions = self.RBX_SETUP_ACTIONS, 
                                  setSetupActionIndFunction = self.setSetupActionInd,
                                  go_actions = self.RBX_GO_ACTIONS, 
                                  setGoActionIndFunction = self.setGoActionInd,
                                  manualControlsReadyFunction = None, #self.manualControlsReady,
                                  getMotorControlRatios=None,
                                  setMotorControlRatio=None,
                                  autonomousControlsReadyFunction = self.autonomousControlsReady,
                                  getHomeFunction=self.getHomeLocation,setHomeFunction=self.setHomeLocation,
                                  goHomeFunction = self.goHome, 
                                  goStopFunction = self.goStop, 
                                  gotoPoseFunction = self.gotoPose,
                                  gotoPositionFunction = self.gotoPosition, 
                                  gotoLocationFunction = self.gotoLocation,
                                  gpsTopic = NEPI_RBX_NAVPOSE_GPS_TOPIC,
                                  odomTopic = NEPI_RBX_NAVPOSE_ODOM_TOPIC,
                                  headingTopic = NEPI_RBX_NAVPOSE_HEADING_TOPIC,
                                  setFakeGPSFunction = self.setFakeGPSFunction
                                )


    self.publishMsg(self.node_name + ": ... IDX interface running")
    #updated the system config from the parameters that have been established
    self.rbx_if.updateFromParamServer()
    time.sleep(1)

    ## Initiation Complete
    self.publishMsg("Initialization Complete")
    #Set up node shutdown
    rospy.on_shutdown(self.cleanup_actions)
    # Spin forever (until object is detected)
    rospy.spin()


  def publishMsg(self,msgStr):
    rospy.loginfo("RBX_ARDU: " + str(msgStr))
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
        if setting_name in self.settings_dict.keys():
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
    
  def checkStopFunction(self):
    triggered = self.stop_triggered
    self.stop_triggered = False # Reset Stop Trigger
    return triggered

  def getBatteryPercent(self):
    return self.battery_percent

  def setHomeLocation(self,geo_point):
    self.set_home_location(geo_point)
    self.fake_gps_reset_pub.publish(geo_point)

  def getHomeLocation(self):
    return self.home_location

  def setFakeGPSFunction(self,fake_gps_enabled):
    self.fake_gps_enable_pub.publish(data = fake_gps_enabled)


  def setMotorControlRatio(self,motor_ind,speed_ratio):
    pass

  def getMotorControlRatios(self):
    return []

  def setSetupActionInd(self,action_ind):
    set_action_function = globals()[self.RBX_SETUP_ACTION_FUNCTIONS[action_ind]]
    success = set_action_function(self)
    return success

  def setGoActionInd(self,action_ind):
    set_action_function = globals()[self.RBX_GO_ACTION_FUNCTIONS[action_ind]]
    success = set_action_function(self)
    return success

  def goStop(self):
    self.stop_triggered = True
    self.fake_gps_go_stop_pub.publish(Empty())
    return True

  def goHome(self):
    self.stop_triggered = True
    nepi_ros.sleep(1,10)
    self.stop_triggered = False
    nepi_ros.sleep(3,30)
    home_loc = self.home_location
    setpoint_location = [home_loc.latitude,home_loc.longitude,home_loc.altitude,-999]
    self.rbx_if.setpoint_location_global_wgs84(setpoint_location)
    self.fake_gps_goto_location_pub.publish(home_loc)
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
    self.fake_gps_goto_position_pub.publish(point_enu_m)

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
    geopoint_wsg84 = nepi_nav.convert_amsl_to_wgs84(geopoint_amsl)
    self.fake_gps_goto_location_pub.publish(geopoint_wsg84)

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


  ### Function to set mavlink armed state
  def set_mavlink_arm_state(self,arm_value):
    last_arm_value = self.mavlink_state.armed
    arm_cmd = CommandBoolRequest()
    arm_cmd.value = arm_value
    self.publishMsg("Updating armed")
    self.publishMsg(arm_value)
    time.sleep(1) # Give time for other process to see busy
    self.publishMsg("Waiting for armed value to set to " + str(arm_value))
    timeout_sec = self.rbx_if.rbx_info.cmd_timeout
    check_interval_s = 0.25
    check_timer = 0
    while self.mavlink_state.armed != arm_value and check_timer < timeout_sec and not rospy.is_shutdown():
      self.mavlink_state.armed = arm_value
      time.sleep(check_interval_s)
      check_timer += check_interval_s
      #self.publishMsg("Waiting for armed value to set")
      #self.publishMsg("Set Value: " + str(arm_value))
      #self.publishMsg("Cur Value: " + str(self.mavlink_state.armed))
    if self.mavlink_state.armed == arm_value:
      self.publishMsg("Armed value set to " + str(arm_value))
      # Reset Home Location on Arming
      if arm_value == True and arm_value != last_arm_value:
        home_loc = GeoPoint()
        home_loc.latitude = self.rbx_if.current_location_wgs84_geo[0]
        home_loc.longitude = self.rbx_if.current_location_wgs84_geo[1]
        home_loc.altitude = self.rbx_if.current_location_wgs84_geo[2]
        self.home_location = home_loc
    else:
      self.publishMsg("Setting Armed value timed-out")
  

  ### Function to set mavlink mode
  def set_mavlink_mode(self,mode_new):
    new_mode = SetModeRequest()
    new_mode.custom_mode = mode_new
    self.publishMsg("Updating mode")
    self.publishMsg(mode_new)
    time.sleep(1) # Give time for other process to see busy
    self.publishMsg("Waiting for mode to set to " + mode_new)
    timeout_sec = self.rbx_if.rbx_info.cmd_timeout
    check_interval_s = 0.25
    check_timer = 0
    while self.mavlink_state.mode != mode_new and check_timer < timeout_sec and not rospy.is_shutdown():
      self.mavlink_state.mode = mode_new
      time.sleep(check_interval_s)
      check_timer += check_interval_s
      #self.publishMsg("Waiting for mode to set")
      #self.publishMsg("Set Value: " + mode_new)
      #self.publishMsg("Cur Value: " + str(self.mavlink_state.mode))
    if self.mavlink_state.mode == mode_new:
      self.publishMsg("Mode set to " + mode_new)
    else:
      self.publishMsg("Setting mode value timed-out")
      
 



  ### Callback to get current mavlink battery message
  def get_mavlink_battery_callback(self,battery_msg):
    self.battery_percent = battery_msg.percentage
 

  #######################
  # Mavlink Ardupilot Interface Methods

  ### Function for switching to arm state
  global arm
  def arm(self):
    success = True
    return success

  ### Function for switching to disarm state
  global disarm
  def disarm(self):
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
    self.rbx_if.update_prev_errors()
    self.rbx_if.update_current_errors( [0,0,0,0,0,0,0] )
    cmd_success = False
    takeoff_height_m = float(self.settings_dict['takeoff_height_m'])
    takeoff_min_pitch_deg = float(self.settings_dict['takeoff_min_pitch_deg'])
    self.publishMsg("Sending Takeoff Command to altitude to " + str(takeoff_height_m) + " meters")
    geo_point = GeoPoint()
    geo_point.latitude = self.rbx_if.current_location_wgs84_geo[0]
    geo_point.longitude = self.rbx_if.current_location_wgs84_geo[1]
    goal_alt = self.rbx_if.current_location_wgs84_geo[2] + takeoff_height_m
    geo_point.altitude = goal_alt
    self.fake_gps_goto_location_pub.publish(geo_point)

    error_bound_m = self.rbx_if.rbx_info.error_bounds.max_distance_error_m
    timeout_sec = self.rbx_if.rbx_info.cmd_timeout
    check_interval_s = float(timeout_sec) / 100
    check_timer = 0
    alt_error = (goal_alt - self.rbx_if.current_location_wgs84_geo[2])
    while (abs(alt_error) > error_bound_m and check_timer < timeout_sec):
      self.rbx_if.update_current_errors( [0,0,alt_error,0,0,0,0] )
      alt_error = (goal_alt - self.rbx_if.current_location_wgs84_geo[2])
      time.sleep(check_interval_s)
      check_timer += check_interval_s
    if (check_timer < timeout_sec):
      cmd_success = True
      self.takeoff_complete = True
      self.publishMsg("Takeoff action completed with error: " + str(alt_error) + " meters")
    else:
      self.takeoff_complete = False
      self.publishMsg("Takeoff action timed-out with error: " + str(alt_error) + " meters")
    return cmd_success

  ### Function for switching to STABILIZE mode
  global stabilize
  def stabilize(self):
    self.fake_gps_go_stop_pub.publish(Empty())
    cmd_success = True
    return cmd_success
      
  ### Function for switching to LAND mode
  global land
  def land(self):
    cmd_success = False
    geo_point = GeoPoint()
    geo_point.latitude = self.rbx_if.current_location_wgs84_geo[0]
    geo_point.longitude = self.rbx_if.current_location_wgs84_geo[1]
    start_alt = self.rbx_if.current_location_wgs84_geo[2]
    goal_alt = 0
    geo_point.altitude = goal_alt
    self.fake_gps_goto_location_pub.publish(geo_point)
    self.publishMsg("Waiting for land process to complete and disarm")
    timeout_sec = self.rbx_if.rbx_info.cmd_timeout
    check_interval_s = float(timeout_sec) / 100
    check_timer = 0
    while (self.state_current == "ARMED" and check_timer < timeout_sec):
      time.sleep(check_interval_s)
      check_timer += check_interval_s
    if self.state_current == "ARMED":
      self.publishMsg("Land process complete")
      cmd_success = True
    else:
      self.publishMsg("Land process timed-out")
    return cmd_success


  ### Function for sending go home command
  global rtl
  def rtl(self):
    cmd_success = False
    self.fake_gps_goto_location_pub.publish(self.home_location)
    error_goal_m = self.rbx_if.rbx_info.error_bounds.max_distance_error_m
    last_loc = self.rbx_if.current_location_wgs84_geo
    timeout_sec = self.rbx_if.rbx_info.cmd_timeout
    check_interval_s = self.rbx_if.rbx_info.error_bounds.min_stabilize_time_s
    check_timer = 0
    stabilized_check = False
    while (stabilized_check is False and check_timer < timeout_sec):
      nepi_ros.sleep(check_interval_s,100)
      check_timer += check_interval_s
      cur_loc = self.rbx_if.current_location_wgs84_geo
      max_distance_error_m = max(abs(np.subtract(cur_loc,last_loc)))
      stabilized_check = max_distance_error_m < error_goal_m
      last_loc = cur_loc
    if stabilized_check:
      self.publishMsg("RTL process complete")
      cmd_success = True
    else:
      self.publishMsg("RTL process timed-out")
    return cmd_success


  ### Function for switching to LOITER mode
  global loiter
  def loiter(self):
    self.fake_gps_go_stop_pub.publish(Empty())
    cmd_success = True
    return cmd_success


  ### Function for switching to Guided mode
  global guided
  def guided(self):
    self.fake_gps_go_stop_pub.publish(Empty())
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


  ### Function for setting home location
  def set_home_location(self,geo_point):
    self.publishMsg('Sending mavlink set home command')
    self.home_location = geo_point
    self.fake_gps_reset_pub.publish(geo_point)
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
  node = ArdupilotRBX()  








