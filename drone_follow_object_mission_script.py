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
import math
from nepi_edge_sdk_base import nepi_ros 
from nepi_edge_sdk_base import nepi_rbx

from std_msgs.msg import Empty,Bool, String, UInt8, Int8, Float32, Float64
from geographic_msgs.msg import GeoPoint
from nepi_ros_interfaces.msg import RBXInfo, RBXStatus, AxisControls, RBXErrorBounds, RBXGotoErrors, RBXMotorControl, \
     RBXGotoPose, RBXGotoPosition, RBXGotoLocation, SettingUpdate
from nepi_ros_interfaces.srv import RBXCapabilitiesQuery, RBXCapabilitiesQueryResponse
from sensor_msgs.msg import NavSatFix, Image
from nepi_ros_interfaces.msg import TargetLocalization

#########################################
# USER SETTINGS - Edit as Necessary 
#########################################
#RBX Robot Name
RBX_ROBOT_NAME = "ardupilot"

# Robot Settings Overides
###################
TAKEOFF_HEIGHT_M = 0.0
# Ignore Yaw Control
IGNORE_YAW_CONTROL = True

###!!!!!!!! Set Automation action parameters !!!!!!!!
OBJ_LABEL_OF_INTEREST = "chair"
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
CMD_STATE_timeout_secEC = 5
CMD_MODE_timeout_secEC = 5
CMD_ACTION_timeout_secEC = 15
CMD_GOTO_timeout_secEC = 15

#########################################
# Node Class
#########################################




class drone_follow_object_mission(object):
  NEPI_BASE_NAMESPACE = nepi_ros.get_base_namespace()

  rbx_settings = []
  rbx_info = RBXInfo()
  rbx_status = RBXStatus()


  #######################
  ### Node Initialization
  def __init__(self):
    rospy.loginfo("DRONE_FOLLOW: Starting Initialization Processes")
    rospy.loginfo("DRONE_FOLLOW: Waiting for namespace containing: " + RBX_ROBOT_NAME)
    robot_namespace = nepi_ros.wait_for_node(RBX_ROBOT_NAME)
    robot_namespace = robot_namespace + "/"
    rospy.loginfo("DRONE_FOLLOW: Found namespace: " + robot_namespace)
    rbx_namespace = (robot_namespace + "rbx/")
    rospy.loginfo("DRONE_FOLLOW: Using rbx namesapce " + rbx_namespace)
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
    rospy.loginfo("DRONE_FOLLOW: Udated settings:" + settings_str)

    # Setup Fake GPS if Enabled   
    if ENABLE_FAKE_GPS:
      rospy.loginfo("DRONE_INSPECT: Enabled Fake GPS")
      self.rbx_enable_fake_gps_pub.publish(ENABLE_FAKE_GPS)
      time.sleep(2)
    if SET_HOME:
      rospy.loginfo("DRONE_INSPECT: Upating RBX Home Location")
      new_home_geo = GeoPoint()
      new_home_geo.latitude = HOME_LOCATION[0]
      new_home_geo.longitude = HOME_LOCATION[1]
      new_home_geo.altitude = HOME_LOCATION[2]
      self.rbx_set_home_pub.publish(new_home_geo)
      nepi_ros.sleep(15,100) # Give system time to stabilize on new gps location

    ###########################     
    # Sutup AI 3d targeting 
    ###########################
    # Wait for AI targeting detection topic and subscribe to it
    AI_TARGETING_TOPIC = "targeting/targeting_data"
    rospy.loginfo("DRONE_FOLLOW: Waiting for topic: " + AI_TARGETING_TOPIC)
    ai_targeting_topic_name = nepi_ros.wait_for_topic(AI_TARGETING_TOPIC)

    ## Initiation Complete
    rospy.loginfo("DRONE_FOLLOW: Initialization Complete")
    rospy.loginfo("DRONE_FOLLOW: Waiting for AI Object Detection")  

    ###########################
    ## Start Mission
    ###########################
    # Run pre-mission processes
    self.pre_mission_actions()
    # Start misson processes
    rospy.loginfo("DRONE_FOLLOW: Starting move to object callback")
    rospy.Subscriber(ai_targeting_topic_name, TargetLocalization, self.move_to_object_callback, queue_size = 1)
 
 
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
    success = nepi_rbx.set_rbx_mode(self,"GUIDED",timeout_sec = CMD_MODE_timeout_secEC)
    # Arm System
    success = nepi_rbx.set_rbx_state(self,"ARM",timeout_sec = CMD_STATE_timeout_secEC)
    # Send Takeoff Command
    success=nepi_rbx.go_rbx_setup_action(self,"TAKEOFF",timeout_sec = CMD_ACTION_timeout_secEC)
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
    success = True
    #########################################

    ###########################
    # Stop Your Custom Actions
    ###########################
    rospy.loginfo("DRONE_FOLLOW: Mission Actions Complete")
    return success

  ## Function for custom post-mission actions
  def post_mission_actions(self):
    ###########################
    # Start Your Custom Actions
    ###########################
    success = True
    #success = nepi_rbx.set_rbx_mode(self,"LAND", timeout_sec = CMD_MODE_timeout_secEC) # Uncomment to change to Land mode
    #success = nepi_rbx.set_rbx_mode(self,"LOITER", timeout_sec = CMD_MODE_timeout_secEC) # Uncomment to change to Loiter mode
    success = nepi_rbx.set_rbx_mode(self,"RTL", timeout_sec = CMD_MODE_timeout_secEC) # Uncomment to change to home mode
    #success = nepi_rbx.set_rbx_mode(self,"RESUME", timeout_sec = CMD_MODE_timeout_secEC) # Uncomment to return to last mode
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
      rospy.loginfo("DRONE_FOLLOW: Initial input image received. Size = " + str(img_msg.width) + "x" + str(img_msg.height))
      self.img_height = img_msg.height
      self.img_width = img_msg.width

  # Action upon detection and targeting for object of interest 
  def move_to_object_callback(self,target_data_msg):
    #rospy.loginfo("DRONE_FOLLOW: Recieved target data message")
    #rospy.loginfo(target_data_msg)
    # Check for the object of interest and take appropriate actions
    target_class = target_data_msg.name
    target_range_m = target_data_msg.range_m # [x,y,z]
    target_yaw_d = target_data_msg.azimuth_deg  # dz
    target_pitch_d = target_data_msg.elevation_deg # dy
    if target_class == OBJ_LABEL_OF_INTEREST and  target_range_m != -999:
      rospy.loginfo("DRONE_FOLLOW: Detected a " + OBJ_LABEL_OF_INTEREST + "with valid range")
      setpoint_range_m = target_range_m - TARGET_OFFSET_GOAL_M
      sp_x_m = setpoint_range_m * math.cos(math.radians(target_yaw_d))  # X is Forward
      sp_y_m = - setpoint_range_m * math.sin(math.radians(target_yaw_d)) # Y is Right
      sp_z_m = - setpoint_range_m * math.sin(math.radians(target_pitch_d)) # Z is Down
      sp_yaw_d = target_yaw_d
      if IGNORE_YAW_CONTROL:
        sp_yaw_d = -999
      setpoint_position_body_m = [sp_x_m,sp_y_m,sp_z_m,sp_yaw_d]
      # Send poisition update
      rospy.loginfo("DRONE_FOLLOW: Sending setpoint position body command")
      rospy.loginfo(setpoint_position_body_m)
      success = nepi_rbx.goto_rbx_position(self,setpoint_position_body_m)
      #########################################
      # Run Mission Actions
      #rospy.loginfo("DRONE_FOLLOW: Starting Mission Actions")
      #success = self.mission_actions()
      ##########################################
##        rospy.loginfo("DRONE_FOLLOW: Switching back to original mode")
##        nepi_rbx.set_rbx_mode(self,"RESUME")
      rospy.loginfo("DRONE_FOLLOW: Delaying next trigger for " + str(TRIGGER_RESET_DELAY_S) + " secs")
      nepi_ros.sleep(TRIGGER_RESET_DELAY_S,100)
      rospy.loginfo("DRONE_FOLLOW: Waiting for next " + OBJ_LABEL_OF_INTEREST + " detection")
    else:
      rospy.loginfo("DRONE_FOLLOW: Target range value invalid, skipping actions")
      time.sleep(1)


  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    rospy.loginfo("DRONE_FOLLOW: Shutting down: Executing script cleanup actions")

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
  rospy.loginfo("DRONE_FOLLOW: Launching node named: " + node_name)
  node_class = eval(node_name)
  node = node_class()
  #Set up node shutdown
  rospy.on_shutdown(node.cleanup_actions)
  # Spin forever (until object is detected)
  rospy.spin()


  



