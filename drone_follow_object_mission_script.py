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
### Expects Classifier to be running ###
# 1) Monitors AI detector output for specfic target class 
# 3) Changes system to Loiter mode on detection
# 4) Sends NEPI snapshot event trigger
# 5) Waits to achieve waits set time to complete snapshot events
# 6) Sets system back to original mode
# 6) Delays, then waits for next detection

# Requires the following additional scripts are running
# a) ai_detector_config_script.py
# (Optional) Some Snapshot Action Automation Script like the following
#   b)snapshot_trigger_save_to_disk_action_script.py
# These scripts are available for download at:
# [link text](https://github.com/numurus-nepi/nepi_sample_auto_scripts)

import rospy
import sys
import time
import math
from nepi_edge_sdk_base import nepi_ros 
from nepi_edge_sdk_base import nepi_msg
from nepi_edge_sdk_base import nepi_rbx

from std_msgs.msg import Empty,Bool, String, UInt8, Int8, Float32, Float64
from geographic_msgs.msg import GeoPoint
from nepi_ros_interfaces.msg import RBXInfo, RBXStatus, AxisControls, RBXErrorBounds, RBXGotoErrors, RBXMotorControl, \
     RBXGotoPose, RBXGotoPosition, RBXGotoLocation, SettingUpdate
from nepi_ros_interfaces.srv import RBXCapabilitiesQuery, RBXCapabilitiesQueryResponse
from sensor_msgs.msg import NavSatFix, Image
from nepi_ros_interfaces.msg import TargetLocalization, TargetLocalizations

#########################################
# USER SETTINGS - Edit as Necessary 
#########################################
#RBX Robot Name
RBX_ROBOT_NAME = "ardupilot"

# Robot Settings Overides
###################
TAKEOFF_HEIGHT_M = 10.0
# Ignore Yaw Control
IGNORE_YAW_CONTROL = True

###!!!!!!!! Set Automation action parameters !!!!!!!!
TARGET_TO_FOLLOW = "chair" # Either a target class name (will follow first found of that class) or specific target_id
TARGET_OFFSET_GOAL_M = 0.1 # How close to set setpoint to target
TRIGGER_RESET_DELAY_S = 5 # Time between detect/move checks 

# Set Home Poistion
ENABLE_FAKE_GPS = True
SET_HOME = True
HOME_LOCATION = [47.6540828,-122.3187578,0.0]

# Goto Error Settings
GOTO_MAX_ERROR_M = 2.0 # Goal reached when all translation move errors are less than this value
GOTO_MAX_ERROR_DEG = 2.0 # Goal reached when all rotation move errors are less than this value
GOTO_STABILIZED_SEC = 1.0 # Window of time that setpoint error values must be good before proceeding

# CMD Timeout Values
CMD_STATE_TIMEOUT_SEC = 5
CMD_MODE_TIMEOUT_SEC = 5
CMD_ACTION_TIMEOUT_SEC = 20
CMD_GOTO_TIMEOUT_SEC = 20

#########################################
# Node Class
#########################################




class drone_follow_object_mission(object):

  rbx_settings = []
  rbx_info = RBXInfo()
  rbx_status = RBXStatus()

  #######################
  ### Node Initialization
  DEFAULT_NODE_NAME = "drone_follow_object_mission" # Can be overwitten by luanch command
  def __init__(self):
    #### APP NODE INIT SETUP ####
    nepi_ros.init_node(name= self.DEFAULT_NODE_NAME)
    self.node_name = nepi_ros.get_node_name()
    self.base_namespace = nepi_ros.get_base_namespace()
    nepi_msg.createMsgPublishers(self)
    nepi_msg.publishMsgInfo(self,"Starting Initialization Processes")
    ##############################
    nepi_msg.publishMsgInfo(self,"Waiting for namespace containing: " + RBX_ROBOT_NAME)
    robot_namespace = nepi_ros.wait_for_node(RBX_ROBOT_NAME)
    robot_namespace = robot_namespace + "/"
    nepi_msg.publishMsgInfo(self,"Found namespace: " + robot_namespace)
    rbx_namespace = (robot_namespace + "rbx/")
    nepi_msg.publishMsgInfo(self,"Using rbx namesapce " + rbx_namespace)
    nepi_rbx.rbx_initialize(self,rbx_namespace)
    time.sleep(1)

    #### publishers used below are defined in nepi_rbx.initialize() helper function call above

    # Apply Takeoff Height setting overide
    th_setting = nepi_ros.get_setting_from_settings('takeoff_height_m',self.rbx_settings)
    th_setting[2] = str(TAKEOFF_HEIGHT_M)
    th_update_msg = nepi_ros.create_update_msg_from_setting(th_setting)
    self.rbx_setting_update_pub.publish(th_update_msg) 
    nepi_ros.sleep(2,10)
    settings_str = str(self.rbx_settings)
    nepi_msg.publishMsgInfo(self,"Updated settings:" + settings_str)


    # Mission Action Topics (If Required)
    self.base_namespace = nepi_ros.get_base_namespace()
    SNAPSHOT_TRIGGER_TOPIC = self.base_namespace + "snapshot_trigger"
    self.snapshot_trigger_pub = rospy.Publisher(SNAPSHOT_TRIGGER_TOPIC, Empty, queue_size = 1)

    # Setup Fake GPS if Enabled   
    if ENABLE_FAKE_GPS:
      nepi_msg.publishMsgInfo(self,"DRONE_INSPECT: Enabled Fake GPS")
      self.rbx_enable_fake_gps_pub.publish(ENABLE_FAKE_GPS)
      time.sleep(2)
    if SET_HOME:
      nepi_msg.publishMsgInfo(self,"DRONE_INSPECT: Upating RBX Home Location")
      new_home_geo = GeoPoint()
      new_home_geo.latitude = HOME_LOCATION[0]
      new_home_geo.longitude = HOME_LOCATION[1]
      new_home_geo.altitude = HOME_LOCATION[2]
      self.rbx_set_home_pub.publish(new_home_geo)
      if ENABLE_FAKE_GPS:
      	nepi_ros.sleep(15,100) # Give system time to stabilize on new gps location




    ###########################     
    # Sutup AI 3d targeting 
    ###########################
    # Wait for AI targeting detection topic and subscribe to it
    AI_TARGETING_TOPIC = "app_ai_targeting/target_localizations"
    nepi_msg.publishMsgInfo(self,"Waiting for topic: " + AI_TARGETING_TOPIC)
    ai_targeting_topic_name = nepi_ros.wait_for_topic(AI_TARGETING_TOPIC)

    ## Initiation Complete
    nepi_msg.publishMsgInfo(self,"Initialization Complete")
    nepi_msg.publishMsgInfo(self,"Waiting for AI Object Detection")  

    ###########################
    ## Start Mission
    ###########################
    # Run pre-mission processes
    self.pre_mission_actions()
    # Start misson processes
    nepi_msg.publishMsgInfo(self,"Starting move to object callback")
    rospy.Subscriber(ai_targeting_topic_name, TargetLocalizations, self.move_to_object_callback, queue_size = 1)

    ##############################
    ## Initiation Complete
    nepi_msg.publishMsgInfo(self," Initialization Complete")
    # Spin forever (until object is detected)
    rospy.spin()
    ##############################
 
  #######################
  ### RBX Settings, Info, and Status Callbacks
  def rbx_settings_callback(self, msg):
    self.rbx_settings = nepi_ros.parse_settings_msg_data(msg.data)


  def rbx_info_callback(self, msg):
    self.rbx_info = msg


  def rbx_status_callback(self, msg):
    self.rbx_status = msg

  #######################
  ### Node Methods    

 ## Function for custom pre-mission actions
  def pre_mission_actions(self):
    ###########################
    # Start Your Custom Actions
    ###########################
    success = True
    # Set Mode to Guided
    success = nepi_rbx.set_rbx_mode(self,"GUIDED",timeout_sec =CMD_MODE_TIMEOUT_SEC)
    # Arm System
    success = nepi_rbx.set_rbx_state(self,"ARM",timeout_sec = CMD_STATE_TIMEOUT_SEC)
    # Send Takeoff Command
    success=nepi_rbx.setup_rbx_action(self,"TAKEOFF",timeout_sec =CMD_ACTION_TIMEOUT_SEC)
    time.sleep(1)
    error_str = str(self.rbx_status.errors_current)
    if success:
      nepi_msg.publishMsgInfo(self,"DRONE_INSPECT: Takeoff completed with errors: " + error_str )
    else:
      nepi_msg.publishMsgInfo(self,"DRONE_INSPECT: Takeoff failed with errors: " + error_str )
    nepi_ros.sleep(2,10)
    ###########################
    # Stop Your Custom Actions
    ###########################
    print("Pre-Mission Actions Complete")
    return success


  ## Function for custom mission actions
  def mission_actions(self):
    ###########################
    # Start Your Custom Actions
    ###########################
    self.snapshot_trigger_pub.publish(Empty)
    success = True
    #########################################

    ###########################
    # Stop Your Custom Actions
    ###########################
    nepi_msg.publishMsgInfo(self,"Mission Actions Complete")
    return success

  ## Function for custom post-mission actions
  def post_mission_actions(self):
    ###########################
    # Start Your Custom Actions
    ###########################
    success = True
    #success = nepi_rbx.set_rbx_mode(self,"LAND", timeout_sec = CMD_MODE_TIMEOUT_SEC) # Uncomment to change to Land mode
    #success = nepi_rbx.set_rbx_mode(self,"LOITER", timeout_sec = CMD_MODE_TIMEOUT_SEC) # Uncomment to change to Loiter mode
    success = nepi_rbx.set_rbx_mode(self,"RTL", timeout_sec = CMD_MODE_TIMEOUT_SEC) # Uncomment to change to home mode
    #success = nepi_rbx.set_rbx_mode(self,"RESUME", timeout_sec = CMD_MODE_TIMEOUT_SEC) # Uncomment to return to last mode
    nepi_ros.sleep(1,10)
    ###########################
    # Stop Your Custom Actions
    ###########################
    print("Post-Mission Actions Complete")
    return success

  #######################
  # AI Detection Functions
  
    ### Simple callback to get image height and width
  def ai_image_callback(self,img_msg):
    # This is just to get the image size for ratio purposes
    if (self.img_height == 0 and self.img_width == 0):
      nepi_msg.publishMsgInfo(self,"Initial input image received. Size = " + str(img_msg.width) + "x" + str(img_msg.height))
      self.img_height = img_msg.height
      self.img_width = img_msg.width

  # Action upon detection and targeting for object of interest 
  def move_to_object_callback(self,targets_data_msg):
    #nepi_msg.publishMsgInfo(self,"Recieved target data message")
    #nepi_msg.publishMsgInfo(self,target_data_msg)
    # Check for the object of interest and take appropriate actions
    for target_data_msg in targets_data_msg.target_localizations:
      target_class = target_data_msg.Class
      target_range_m = target_data_msg.range_m # [x,y,z]
      target_yaw_d = target_data_msg.azimuth_deg  # dz
      target_pitch_d = target_data_msg.elevation_deg # dy
      if target_class == TARGET_TO_FOLLOW and  target_range_m != -999:
        nepi_msg.publishMsgInfo(self,"Detected a " + TARGET_TO_FOLLOW + "with valid range")
        setpoint_range_m = target_range_m - TARGET_OFFSET_GOAL_M
        sp_x_m = setpoint_range_m * math.cos(math.radians(target_yaw_d))  # X is Forward
        sp_y_m = setpoint_range_m * math.sin(math.radians(target_yaw_d)) # Y is Right
        sp_z_m = - setpoint_range_m * math.sin(math.radians(target_pitch_d)) # Z is Down
        sp_yaw_d = target_yaw_d
        if IGNORE_YAW_CONTROL:
          sp_yaw_d = -999
        setpoint_position_body_m = [sp_x_m,sp_y_m,sp_z_m,sp_yaw_d]
        rospy.logwarn(setpoint_position_body_m)
        # Send poisition update
        nepi_msg.publishMsgInfo(self,"Sending setpoint position body command")
        nepi_msg.publishMsgInfo(self,setpoint_position_body_m)
        success = nepi_rbx.goto_rbx_position(self,setpoint_position_body_m)
        error_str = str(self.rbx_status.errors_current)
        if success:
          nepi_msg.publishMsgInfo(self,"Goto Position completed with errors: " + error_str )
        else:
          nepi_msg.publishMsgInfo(self,"Goto Position failed with errors: " + error_str )
        nepi_ros.sleep(2,10)
        #########################################
        # Run Mission Actions
        #nepi_msg.publishMsgInfo(self,"Starting Mission Actions")
        #success = self.mission_actions()
        ##########################################
  ##        nepi_msg.publishMsgInfo(self,"Switching back to original mode")
  ##        nepi_rbx.set_rbx_mode(self,"RESUME")
        nepi_msg.publishMsgInfo(self,"Delaying next trigger for " + str(TRIGGER_RESET_DELAY_S) + " secs")
        nepi_ros.sleep(TRIGGER_RESET_DELAY_S,100)
        nepi_msg.publishMsgInfo(self,"Waiting for next " + TARGET_TO_FOLLOW + " detection")
      else:
        nepi_msg.publishMsgInfo(self,"Target range value invalid, skipping actions")
        time.sleep(1)


  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    nepi_msg.publishMsgInfo(self,"Shutting down: Executing script cleanup actions")

#########################################
# Main
#########################################
if __name__ == '__main__':
  drone_follow_object_mission()


  



