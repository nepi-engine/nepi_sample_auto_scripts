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

from nepi_edge_sdk_base import nepi_ros 
from nepi_edge_sdk_base import nepi_nav
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
                  ["Float","takeoff_min_pitch_deg","-90.0","90.0"]]

  FACTORY_SETTINGS_OVERRIDES = dict( takeoff_height_m = "5",
                                      takeoff_min_pitch_deg = "10")

  
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

  #######################
  ### Node Initialization
  def __init__(self):

    ###################################################
    # Get Mavlink NameSpace
    NEPI_RBX_NAMESPACE = rospy.get_name() + "/rbx/"
    rospy.loginfo("RBX_ARDU: Starting Initialization Processes")
    self.status_msg_pub = rospy.Publisher(NEPI_RBX_NAMESPACE + "status_msg", String, queue_size=1)


    # MAVLINK Fake settings
    self.state_ind=0
    self.state_current = self.RBX_STATES[self.state_ind]
    self.mode_ind=0 
    self.mode_current = self.RBX_MODES[self.mode_ind]
  
    self.publishMsg("Starting State: " + self.state_current)
    self.publishMsg("Starting Mode: " + self.mode_current)

    self.node_name = rospy.get_name().split('/')[-1]
    self.publishMsg(self.node_name + ": ... Connected!")


    # Initialize settings
    self.cap_settings = self.getCapSettings()
    self.publishMsg("CAPS SETTINGS")
    #for setting in self.cap_settings:
        #self.publishMsg(setting)
    self.factory_settings = self.getFactorySettings()
    self.publishMsg("FACTORY SETTINGS")
    #for setting in self.factory_settings:
        #self.publishMsg(setting)
          

    ## Define RBX Fake GPS
    ROBOT_NAMESPACE = rospy.get_name() + "/"
    NEPI_RBX_NAVPOSE_GPS_TOPIC = ROBOT_NAMESPACE + "fake_gps_stream"
    FAKE_GPS_NAMESPACE =  ROBOT_NAMESPACE + "fake_gps/"
    self.publishMsg("Setting up fake_gps at namespace: " + FAKE_GPS_NAMESPACE)
    fake_gps = ArdupilotFakeGPS(robot_namespace = ROBOT_NAMESPACE)
    time.sleep(2) 


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
                                  odomTopic = None, # NEPI_RBX_NAVPOSE_ODOM_TOPIC,
                                  headingTopic = None, #NEPI_RBX_NAVPOSE_HEADING_TOPIC,
                                  fake_gps_namespace = FAKE_GPS_NAMESPACE
                                )

    time.sleep(2) 
    self.publishMsg("Connecting fake gps to rbx robot") 
    fake_gps.connnectToRbx()

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
    #self.setpoint_attitude_pub.publish(attitude_target_msg) # Publish Setpoint

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
    #self.setpoint_position_local_pub.publish(position_local_target_msg) # Publish Setpoint

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
    #self.setpoint_location_global_pub.publish(location_global_target_msg) # Publish Setpoint

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
    self.publishMsg("Updating armed")
    self.publishMsg(arm_value)
    time.sleep(1) # Give time for other process to see busy
    self.publishMsg("Waiting for armed value to set to " + str(arm_value))
    time.sleep(.25)
    if arm_value == False:
      self.state_ind = 0
      self.state_current = "DISARM"
    else:
      self.state_ind = 1
      self.state_current = "ARM"
    self.publishMsg("Armed value set to " + str(arm_value))

  ### Function to set mavlink mode
  def set_mavlink_mode(self,mode_new):
    self.publishMsg("Updating mode")
    self.publishMsg(mode_new)
    time.sleep(1) # Give time for other process to see busy
    self.publishMsg("Waiting for mode to set to " + mode_new)
    self.mode_ind = self.RBX_MODES.index(mode_new)
    self.mode_current = mode_new
    time.sleep(.25)
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
    self.publishMsg("Sending Takeoff Command to altitude to " + str(takeoff_height_m) + " meters")
    #self.takeoff_client(min_pitch=takeoff_min_pitch_deg,altitude=takeoff_height_m)
    start_alt = self.rbx_if.current_location_wgs84_geo[2]
    goal_alt = start_alt + takeoff_height_m
    error_bound_m = self.rbx_if.rbx_info.error_bounds.max_distance_error_m
    timeout_sec = self.rbx_if.rbx_info.cmd_timeout
    check_interval_s = float(timeout_sec) / 100
    check_timer = 0
    alt_error = abs(goal_alt - self.rbx_if.current_location_wgs84_geo[2])
    while (abs(alt_error) > error_bound_m and check_timer < timeout_sec):
      self.rbx_if.update_current_errors( [0,0,alt_error,0,0,0,0] )
      alt_error = (goal_alt - self.rbx_if.current_location_wgs84_geo[2])
      time.sleep(check_interval_s)
      check_timer += check_interval_s
    self.rbx_if.update_current_errors( [0,0,0,0,0,0,0] )
    self.rbx_if.update_prev_errors( [0,0,alt_error,0,0,0,0] )
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
    self.set_mavlink_mode('STABILIZE')
    cmd_success = True
    return cmd_success
      
  ### Function for switching to LAND mode
  global land
  def land(self):
    cmd_success = False
    self.set_mavlink_mode('LAND')
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
    self.set_mavlink_mode('RTL')
    error_goal_m = self.rbx_if.rbx_info.error_bounds.max_distance_error_m
    last_loc = self.rbx_if.current_location_wgs84_geo
    timeout_sec = self.rbx_if.rbx_info.cmd_timeout
    check_interval_s = self.rbx_if.rbx_info.error_bounds.min_stabilize_time_s
    check_timer = 0
    stabilized_check = False
    while (stabilized_check and check_timer < timeout_sec):
      nepi_ros.sleep(check_interval_s,100)
      check_timer += check_interval_s
      cur_loc = self.rbx_if.current_location_wgs84_geo
      max_distance_error_m = max(abs(numpy.subtract(cur_loc,last_loc)))
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
# Fake GPS Class
#########################################

class ArdupilotFakeGPS():

  # ROS namespace setup
  NEPI_BASE_NAMESPACE = nepi_ros.get_base_namespace()

  #Homeup Location
  # [Lat, Long, Altitude_WGS84]
  FAKE_GPS_START_GEOPOINT_WGS84 = [46.6540828,-122.3187578,0.0]

  #GPS Setup
  SAT_COUNT = 20
  GPS_PUB_RATE_HZ = 20

  #GPS Simulation Position Update Controls
  #Adjust these settings for smoother xyz movements
  MOVE_UPDATE_TIME_SEC_PER_METER=1

  PRINT_LOCATION_1HZ = False
   
  #######################
  ### Node Initialization
  def __init__(self,robot_namespace):

    self.rbx_caps = None
    self.fake_gps_enabled = True

    rospy.loginfo("RBX_FAKE_GPS: Starting Initialization Processes")
    ## Initialize Class Variables
    # RBX State and Mode Dictionaries
    self.rbx_cap_modes = []
    self.rbx_cap_actions = []
    self.current_location_wgs84_geo = None
    self.current_heading_deg = None
    self.current_home_wgs84_geo = None
    self.navpose_update_interval = 0.1
    self.fake_gps_ready = True
    self.gps_publish_interval_sec=1.0/self.GPS_PUB_RATE_HZ
    self.takeoff_m = 0
    # Initialize Current Location
    self.current_location_wgs84_geo=GeoPoint()
    self.current_location_wgs84_geo.latitude = self.FAKE_GPS_START_GEOPOINT_WGS84[0]
    self.current_location_wgs84_geo.longitude = self.FAKE_GPS_START_GEOPOINT_WGS84[1]
    self.current_location_wgs84_geo.altitude = self.FAKE_GPS_START_GEOPOINT_WGS84[2]
    self.current_home_wgs84_geo=GeoPoint()
    self.current_home_wgs84_geo.latitude = self.current_location_wgs84_geo.latitude
    self.current_home_wgs84_geo.longitude = self.current_location_wgs84_geo.longitude
    self.current_home_wgs84_geo.altitude = self.current_location_wgs84_geo.altitude

    rospy.loginfo("RBX_FAKE_GPS: Start Home GEO Location: " + str(self.current_home_wgs84_geo))

    self.nepi_nav_service_name = nepi_ros.get_base_namespace() + "nav_pose_query"

    # Fake GPS Publish Topic
    self.fake_gps_pub = rospy.Publisher(robot_namespace + "fake_gps_stream", NavSatFix, queue_size=1)

    # Start update heading callback 
    rospy.Timer(rospy.Duration(self.navpose_update_interval), self.update_current_heading_callback)
    while self.current_heading_deg is None and not rospy.is_shutdown():
      rospy.loginfo("RBX_FAKE_GPS: Waiting for current heading from navpose service call")
      nepi_ros.sleep(1,10)
      rospy.Timer(rospy.Duration(self.gps_publish_interval_sec), self.fake_gps_pub_callback)

   # Setup RBX driver interfaces
    self.robot_namespace = robot_namespace
    self.rbx_namespace = robot_namespace + "rbx/"
    self.fake_gps_namespace = self.robot_namespace + "fake_gps/"

    rospy.loginfo("RBX_FAKE_GPS: Starting fake gps subscribers with namespace: " + self.fake_gps_namespace)

         
    # Create Fake GPS controls subscribers
    rospy.Subscriber(self.fake_gps_namespace + "enable",Bool, self.fakeGPSEnableCb)
    rospy.Subscriber(self.fake_gps_namespace + "reset",Empty, self.fakeGPSResetCb)
    rospy.Subscriber(self.fake_gps_namespace + "goto_position", RBXGotoPosition, self.fakeGPSGoPosCB)
    rospy.Subscriber(self.fake_gps_namespace + "goto_location", RBXGotoLocation, self.fakeGPSGoLocCB)
    rospy.Subscriber(self.fake_gps_namespace + "set_home", GeoPoint, self.fakeGPSSetHomeCB)
    rospy.Subscriber(self.fake_gps_namespace + "go_home", Empty, self.fakeGPSGoHomeCB)
    rospy.Subscriber(self.fake_gps_namespace + "set_home_current", Empty, self.fakeGPSSetHomeCurCB)
    rospy.Subscriber(self.fake_gps_namespace + "set_mode", UInt8, self.fakeGPSModeCB)
    rospy.Subscriber(self.fake_gps_namespace + "go_action", UInt8, self.fakeGPSActionCB)

    self.status_pub = rospy.Publisher(self.fake_gps_namespace + "status", Bool, queue_size=1, latch = True)
 

    ## Initiation Complete
    rospy.loginfo("RBX_FAKE_GPS: Initialization Complete")

  def connnectToRbx(self):
    # NEPI RBX Caps Service
    caps_topic = self.rbx_namespace + "capabilities_query"
    self.rbx_caps = nepi_rbx.get_capabilities(self,caps_topic)
    self.rbx_cap_states = eval(self.rbx_caps.state_options)
    self.rbx_cap_modes = eval(self.rbx_caps.mode_options)
    self.rbx_cap_actions = eval(self.rbx_caps.action_options)
    # NEPI RBX Settings Status Subscriber Topic
    rospy.Subscriber(self.robot_namespace + "settings_status", String, self.rbxSettingsCB)
    self.force_settings_pub = rospy.Publisher(self.robot_namespace + "publish_settings",Empty, queue_size=1)
    time.sleep(1)
    self.force_settings_pub.publish(Empty())
    rospy.loginfo("RBX_Fake_GPS: Connecting to rbx namespace: " + self.rbx_namespace)




  #######################
  ### Node Methods


  ### Setup a regular Send Fake GPS callback using current geo point value
  def fake_gps_pub_callback(self,timer):
    if self.fake_gps_ready:
      self.publishFakeGPS()

  def publishFakeGPS(self):
    if self.fake_gps_enabled:
      navsatfix = NavSatFix()
      navsatfix.latitude = self.current_location_wgs84_geo.latitude
      navsatfix.longitude = self.current_location_wgs84_geo.longitude
      navsatfix.altitude = self.current_location_wgs84_geo.altitude
      #rospy.loginfo(navsatfix)
      if not rospy.is_shutdown():
        self.fake_gps_pub.publish(navsatfix)


  ### function to simulate move to new global geo position
  def move(self,geopoint_msg):
    rospy.loginfo("")
    rospy.loginfo('***********************')
    loc = self.current_location_wgs84_geo
    rospy.loginfo("RBX_FAKE_GPS: Fake GPS Moving FROM: " + str(loc.latitude) + ", " + str(loc.longitude) + ", " + str(loc.altitude)) 
    loc = geopoint_msg
    rospy.loginfo("RBX_FAKE_GPS: T0: " + str(loc.latitude) + ", " + str(loc.longitude) + ", " + str(loc.altitude)) 
    org_geo=np.array([self.current_location_wgs84_geo.latitude, \
                      self.current_location_wgs84_geo.longitude, self.current_location_wgs84_geo.altitude])
    cur_geo = org_geo
    new_geo=np.array([geopoint_msg.latitude, geopoint_msg.longitude, geopoint_msg.altitude])
    for ind, val in enumerate(new_geo):
      if new_geo[ind] == -999.0: # Use current
        new_geo[ind]=org_geo[ind]
    delta_geo = new_geo - org_geo
    #rospy.loginfo("RBX_FAKE_GPS: TO:")
    #rospy.loginfo(delta_geo)
    
    move_dist_m = nepi_nav.distance_geopoints(org_geo,new_geo)
    if move_dist_m > 0:
      move_time = self.MOVE_UPDATE_TIME_SEC_PER_METER * move_dist_m
      move_steps = move_time * self.GPS_PUB_RATE_HZ
      stp_interval_sec = float(move_time)/float(move_steps)
      rospy.loginfo("RBX_FAKE_GPS: Moving " + "%.2f" % move_dist_m + " meters in " + "%.2f" % move_time + " seconds")
      rospy.loginfo("RBX_FAKE_GPS: with " + "%.2f" % move_steps + " steps")

      ramp=np.hanning(move_steps)
      ramp=ramp**2
      ramp_norm=ramp/np.sum(ramp)
      step_norm=np.zeros(len(ramp_norm))
      for ind, val in enumerate(ramp_norm):
        step_norm[ind]=np.sum(ramp_norm[0:ind])
      
      rospy.loginfo_timer = 0
      for ind, val in enumerate(step_norm):
        time.sleep(stp_interval_sec)
        #rospy.loginfo_timer = rospy.loginfo_timer + stp_interval_sec
        cur_geo_step = delta_geo * val
        cur_geo = org_geo + cur_geo_step
        self.current_location_wgs84_geo.latitude = cur_geo[0]
        self.current_location_wgs84_geo.longitude = cur_geo[1]
        self.current_location_wgs84_geo.altitude = cur_geo[2]
        #if rospy.loginfo_timer > 0.5:
          #rospy.loginfo("RBX_FAKE_GPS: ")
          #rospy.loginfo("RBX_FAKE_GPS: Updated to")
          #rospy.loginfo(self.current_location_wgs84_geo)
          #current_error_m = nepi_nav.distance_geopoints(cur_geo,new_geo)
          #rospy.loginfo("RBX_FAKE_GPS: Current move error : " + "%.2f" % (current_error_m) + " meters")
          #rospy.loginfo_timer=0
    current_error_m = nepi_nav.distance_geopoints(cur_geo,new_geo)
    rospy.loginfo("RBX_FAKE_GPS: Move Error: " + "%.2f" % (current_error_m) + " meters")

    rospy.loginfo("RBX_FAKE_GPS: FAKE GPS Move Complete")
    rospy.loginfo('***********************')



  ### Callback to rospy.loginfo the current fake geopoint position at slower rate
  def print_callback(self,timer):
    if (self.checkEnable()):
      rospy.loginfo('')
      rospy.loginfo("RBX_FAKE_GPS: Current WGS84 Geo Location ")
      rospy.loginfo(self.current_location_wgs84_geo)


  #######################
  # NEPI NavPose Interfaces

  ### Setup a regular background navpose get and update heading data
  def update_current_heading_callback(self,timer):
    # Get current NEPI NavPose data from NEPI ROS nav_pose_query service call
    try:
      get_navpose_service = rospy.ServiceProxy(self.nepi_nav_service_name, NavPoseQuery)
      nav_pose_response = get_navpose_service(NavPoseQueryRequest())
      self.current_heading_deg = nav_pose_response.nav_pose.heading.heading
      #rospy.loginfo('')
      #rospy.loginfo("RBX_FAKE_GPS: Update current heading to: " + "%.2f" % (self.current_heading_deg))
    except Exception as e:
      rospy.loginfo("RBX_FAKE_GPS: navpose service call failed: " + str(e))


  #######################
  # NEPI Fake GPS Interfaces

    ### Callback to set fake gps enable
  def fakeGPSEnableCb(self,msg):
    rospy.loginfo("RBX_FAKE_GPS: Received set fake gps enable message: " + str(msg.data))
    self.fake_gps_enabled = msg.data
    self.status_pub.publish(self.fake_gps_enabled)

  def fakeGPSResetCb(self,msg):
    if self.fake_gps_enabled:
      rospy.loginfo("Received Fake GPS Reset Msg")
      success = self.reset_gps()
      if success:
        rospy.loginfo("RBX_FAKE_GPS: Reset Complete")
      return success

  ### Function to reset gps and wait for position ned x,y to reset
  def reset_gps(self):
    if self.fake_gps_enabled:
      self.fake_gps_ready = False
      time.sleep(1)
      new_location = GeoPoint()
      new_location.latitude = self.current_home_wgs84_geo.latitude
      new_location.longitude = self.current_home_wgs84_geo.longitude
      new_location.altitude = self.current_home_wgs84_geo.altitude
      self.current_location_wgs84_geo = new_location
      #rospy.loginfo("RBX_FAKE_GPS: Waiting for GPS to reset") 
      for i in range(90):
        self.publishFakeGPS()
        time.sleep(.1)
      self.fake_gps_ready = True
      return True


  ### Callback to set home
  def fakeGPSSetHomeCB(self,geo_msg):
    if self.fake_gps_enabled:
      rospy.loginfo("RBX_FAKE_GPS: Received set home message")
      #rospy.loginfo(geo_msg)
      self.current_home_wgs84_geo = geo_msg


  ### Callback to set home current
  def fakeGPSSetHomeCurCB(self,empty_msg):
    if self.fake_gps_enabled:
      rospy.loginfo("RBX_FAKE_GPS: Received set home current message")
      new_location = GeoPoint()
      new_location.latitude = self.current_location_wgs84_geo.latitude
      new_location.longitude = self.current_location_wgs84_geo.longitude
      new_location.altitude = self.current_location_wgs84_geo.altitude
      self.current_home_wgs84_geo = self.current_location_wgs84_geo

  ### Callback to go home
  def fakeGPSGoHomeCB(self,empty_msg):
    if self.fake_gps_enabled:
      rospy.loginfo("RBX_FAKE_GPS: Received go home message")
      self.move(self.current_home_wgs84_geo)

  ### Function to monitor RBX GoTo Position Command Topics
  def fakeGPSGoPosCB(self,position_cmd_msg):
    if self.fake_gps_enabled:
      rospy.loginfo("RBX_FAKE_GPS: Recieved GoTo Position Message")
      #rospy.loginfo(position_cmd_msg)
      new_position = [position_cmd_msg.x_meters,position_cmd_msg.y_meters,position_cmd_msg.z_meters,position_cmd_msg.yaw_deg]
      rospy.loginfo("RBX_FAKE_GPS: Sending Fake GPS Setpoint Position Update")
      new_geopoint_wgs84=nepi_nav.get_geopoint_at_body_point(self.current_location_wgs84_geo, \
                                                    self.current_heading_deg, new_position)    
      self.move(new_geopoint_wgs84)


  ### Function to monitor RBX GoTo Location Command Topics
  def fakeGPSGoLocCB(self,location_cmd_msg):
    if self.fake_gps_enabled:
      rospy.loginfo("RBX_FAKE_GPS: Recieved GoTo Location Message")
      #rospy.loginfo(location_cmd_msg)
      new_location = [location_cmd_msg.lat,location_cmd_msg.long,location_cmd_msg.altitude_meters,location_cmd_msg.yaw_deg]
      new_geopoint_wgs84=GeoPoint()
      new_geopoint_wgs84.latitude = new_location[0]
      new_geopoint_wgs84.longitude = new_location[1]
      new_geopoint_wgs84.altitude = new_location[2]
      self.move(new_geopoint_wgs84)


  ### Callback to set mode
  def fakeGPSModeCB(self,mode_msg):
    if self.fake_gps_enabled:
      rospy.loginfo("RBX_FAKE_GPS: Recieved Set Mode Message")
      #rospy.loginfo(mode_msg)
      mode_ind = mode_msg.data
      if mode_ind < 0 or mode_ind > (len(self.rbx_cap_modes)-1):
        rospy.loginfo("RBX_FAKE_GPS: No matching rbx mode found")
      else:
        set_mode_function = globals()[self.rbx_cap_modes[mode_ind]]
        set_mode_function(self)

  ### Callback to set mode
  def fakeGPSActionCB(self,action_msg):
    if self.fake_gps_enabled:
      rospy.loginfo("RBX_FAKE_GPS: Recieved Go Action Message")
      #rospy.loginfo(action_msg)
      action_ind = action_msg.data
      print(action_ind)
      print((len(self.rbx_cap_actions)-1))
      if action_ind < 0 or action_ind > (len(self.rbx_cap_actions)-1):
        rospy.loginfo("RBX_FAKE_GPS: No matching rbx action found")
      else:
        set_action_function = globals()[self.rbx_cap_actions[action_ind]]
        set_action_function(self)


  #######################
  # Mavlink Ardupilot Interface Methods
    
  ### update Takeoff height from RBX settings status message
  def rbxSettingsCB(self,settingsMsg):
    rospy.loginfo("RBX_FAKE_GPS: Fake GPS recievied settings status message: " + settingsMsg.data)
    settings = nepi_ros.parse_settings_msg_data(settingsMsg.data)
    setting = nepi_ros.get_setting_from_settings('takeoff_height_m',settings)
    #rospy.loginfo(setting)
    if setting is not None:
      [s_name,s_type, data]= nepi_ros.get_data_from_setting(setting)    
      if data is not None:
        self.takeoff_m = data

  ## Function for sending takeoff command

  global LAUNCH
  def LAUNCH(self):
    print(self.current_location_wgs84_geo.altitude)
    print(self.takeoff_m)
    new_alt_m = self.current_location_wgs84_geo.altitude + self.takeoff_m
    print(new_alt_m)
    new_geopoint_wgs84=GeoPoint()
    new_geopoint_wgs84.latitude = self.current_location_wgs84_geo.latitude
    new_geopoint_wgs84.longitude = self.current_location_wgs84_geo.longitude
    new_geopoint_wgs84.altitude = new_alt_m
    self.move(new_geopoint_wgs84)

  global TAKEOFF
  def TAKEOFF(self):
    print(self.current_location_wgs84_geo.altitude)
    print(self.takeoff_m)
    new_alt_m = self.current_location_wgs84_geo.altitude + self.takeoff_m
    print(new_alt_m)
    new_geopoint_wgs84=GeoPoint()
    new_geopoint_wgs84.latitude = self.current_location_wgs84_geo.latitude
    new_geopoint_wgs84.longitude = self.current_location_wgs84_geo.longitude
    new_geopoint_wgs84.altitude = new_alt_m
    self.move(new_geopoint_wgs84)

  ### Function for switching to STABILIZE mode
  global STABILIZE
  def STABILIZE(self):
    rospy.loginfo("RBX_FAKE_GPS: ")
      
  ### Function for switching to LAND mode
  global LAND
  def LAND(self):
    new_geopoint_wgs84=GeoPoint()
    new_geopoint_wgs84.latitude = self.current_location_wgs84_geo.latitude
    new_geopoint_wgs84.longitude = self.current_location_wgs84_geo.longitude
    new_geopoint_wgs84.altitude = 0
    self.move(new_geopoint_wgs84)
    
  ### Function for sending go home command
  global RTL
  def RTL(self):
    self.move(self.current_home_wgs84_geo)

  ### Function for switching to LOITER mode
  global LOITER
  def LOITER(self):
    rospy.loginfo("RBX_FAKE_GPS: ")

  ### Function for switching back to GUIDED mode
  global GUIDED
  def GUIDED(self):
    rospy.loginfo("RBX_FAKE_GPS: ")

  ### Function for switching back to current mission
  global RESUME
  def RESUME(self):
    rospy.loginfo("RBX_FAKE_GPS: ")


#########################################
# Main
#########################################
if __name__ == '__main__':
  node_name = "ardupilot_fake"
  rospy.init_node
  rospy.init_node(name = node_name)
  rospy.loginfo("Launching node named: " + node_name)
  node = ArdupilotRBX()  
  #Set up node shutdown
  rospy.on_shutdown(node.cleanup_actions)
  # Spin forever (until object is detected)
  rospy.spin()







