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


# Sample NEPI Automation Script.
# Uses onboard ROS python and mavros libraries to
# 1) Subscribes to NEPI nav_pose_current heading, orientation, position, location topics
# 2) Runs pre-mission processes
# 3) Runs mission setpoint command processes
# 4) Runs mission setpoint action processes
# 5) Runs post-mission processes

# Requires the following additional scripts are running
# a) navpose_publish_process_script.py
# b) mavros_setpoint_control_script.py
# c) mavros_navpose_config_script.py
# d) (Optional) MAVROS_fake_gps_config_script.py if a real GPS fix is not available
# These scripts are available for download at:
# [link text](https://github.com/numurus-nepi/nepi_sample_auto_scripts)

import rospy
import time
import numpy as np
import math
import tf

from std_msgs.msg import Empty, Bool, String, Float64, Float64MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped
from geographic_msgs.msg import GeoPoint, GeoPose, GeoPoseStamped
from mavros_msgs.msg import State, AttitudeTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL, CommandHome

#####################################################################################
# SETUP - Edit as Necessary ##################################
##########################################


# Setpoint Position Global Settings
###################################################
# setpoint_location is [LAT, LONG, ALT_WGS84, YAW_NED_DEGREES]
# Altitude is specified as meters above the WGS-84 and converted to AMSL before sending
# Yaw is specified in NED frame degrees 0-360 or +-180 
#####################################################
SETPOINT_LOCATION_GLOBAL = [47.6541208,-122.3186620, 10, -999] # [Lat, Long, Alt WGS84, Yaw NED Frame], Enter -999 to use current value
SETPOINT_CORNERS_GLOBAL =  [[47.65412620,-122.31881480, -999, -999],[47.65402050,-122.31875320, -999, -999],[47.65391570,-122.31883630, -999, -999],[47.65397990,-122.31912330, -999, -999]]

# Setpoint Position Local Body Settings
###################################################
# setpoint_position is [X_BODY_METERS, Y_BODY_METERS, Z_BODY_METERS, YEW_BODY_DEGREES]
# Local Body Position Setpoint Function use these body relative x,y,z,yaw conventions
# x+ axis is forward
# y+ axis is right
# z+ axis is down
# Only yaw orientation updated
# yaw+ clockwise, yaw- counter clockwise from x axis (0 degrees faces x+ and rotates positive using right hand rule around z+ axis down)
#####################################################
SETPOINT_POSITION_BODY = [10,5,0,0] # [X, Y, Z, YAW] Offset in xyz meters yaw body +- 180 (+Z is Down). Use 0 value for no change

# Setpoint Attitude NED Settings
###################################################
# setpoint_attitudeInp is [ROLL_NED_DEG, PITCH_NED_DEG, YEW_NED_DEGREES]
###################################################
SETPOINT_ATTITUDE_NED = [-999,30,-999] # Roll, Pitch, Yaw Degrees: Enter -999 to use current value


# SETPOINT ACTION SETTINGS
SNAPSHOT_EVENT_WAIT_SEC = 5.0 # Time to wait for snapshot event to complete


# FAKE GPS SETTINGS
###################################################
# The Fake GPS Sim automation script is available at
# https://github.com/numurus-nepi/nepi_sample_auto_scripts
#####################################################
MAVROS_FAKE_GPS_SIM_SUPPORT = True # Set True if running "MAVROS_fake_gps_sim_auto_script.py"
FAKE_GPS_HOME_GEOPOINT_WGS84 = [47.6541271,-122.3189492,0.0] # [Lat, Long, Altitude_WGS84], Use -999 values to use current
### Currently, the geoid_height for your home location must be set in the navpose_get_and_publsih_auto_script.py auto script
### Plan is to automate this in future updates. For now, you can use this link to get your geoid height value
### for the current Lat Long location: [link text](https://geodesy.noaa.gov/GEOID/GEOID18/computation.html)



# ROS namespace setup
NEPI_BASE_NAMESPACE = "/nepi/s2x/"
MAVROS_NAMESPACE = NEPI_BASE_NAMESPACE + "mavlink/"
MAVROS_CONTROLS_NAMESPACE = MAVROS_NAMESPACE + "controls/"

# Setpoint Action Topics
SNAPSHOT_TOPIC = NEPI_BASE_NAMESPACE + "snapshot_event"



# NEPI MAVROS Control Publisher Topics
MAVROS_CONTROL_ATTITUDE_TOPIC = MAVROS_CONTROLS_NAMESPACE + "setpoint_command_attitude"
MAVROS_CONTROL_POSITION_TOPIC = MAVROS_CONTROLS_NAMESPACE + "setpoint_command_position"
MAVROS_CONTROL_LOCATION_TOPIC = MAVROS_CONTROLS_NAMESPACE + "setpoint_command_location"
# NEPI MAVROS Control Subscriber Topics
MAVROS_CONTROL_PROCESS_COMPLETE_TOPIC = MAVROS_CONTROLS_NAMESPACE + "setpoint_complete_status"




### Setup NEPI NavPose Settings Topic Namespaces
NEPI_NAVPOSE_SET_GPS_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose_mgr/set_gps_fix_topic"
NEPI_NAVPOSE_SET_HEADING_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose_mgr/set_heading_topic"
NEPI_NAVPOSE_SET_ORIENTATION_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose_mgr/set_orientation_topic"
NEPI_NAVPOSE_ENABLE_GPS_CLOCK_SYNC_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose_mgr/enable_gps_clock_sync"


# MAVROS Fake GPS Publish Topics
MAVROS_FAKE_GPS_GOTO_GEOPOINT_TOPIC = MAVROS_NAMESPACE + "fake_gps/goto_geopoint_wgs84"
MAVROS_FAKE_GPS_RESET_GEOPOINT_TOPIC = MAVROS_NAMESPACE + "fake_gps/reset_geopoint_wgs84"


#####################################################################################
# Globals
#####################################################################################
setpoint_command_attitude_pub = rospy.Publisher(MAVROS_CONTROL_ATTITUDE_TOPIC, Float64MultiArray, queue_size=1)
setpoint_command_position_pub = rospy.Publisher(MAVROS_CONTROL_POSITION_TOPIC, Float64MultiArray, queue_size=1)
setpoint_command_location_pub = rospy.Publisher(MAVROS_CONTROL_LOCATION_TOPIC, Float64MultiArray, queue_size=1)

snapshot_trigger_pub = rospy.Publisher(SNAPSHOT_TOPIC, Empty, queue_size = 1)


current_state = None
original_state = None
current_home = None

setpoint_complete_status = None
               
#####################################################################################
# Methods
#####################################################################################

### System Initialization processes
def initialize_actions():
  global current_state
  global original_state
  global current_home
  global setpoint_complete_status
  ## Start Get Update State Subscriber Callback
  # Wait for topic
  print("Waiting for topic: " + MAVROS_STATE_TOPIC)
  wait_for_topic(MAVROS_STATE_TOPIC, 'mavros_msgs/State')
  print("Starting state scubscriber callback")
  rospy.Subscriber(MAVROS_STATE_TOPIC, State, get_state_callback)
  while current_state is None and not rospy.is_shutdown():
    print("Waiting for current state to publish")
    time.sleep(0.1)
  print(current_state)
  original_state = current_state
  # Wait for setpoint controls status to publish
  print("Waiting for topic: " + MAVROS_CONTROL_PROCESS_COMPLETE_TOPIC)
  wait_for_topic(MAVROS_CONTROL_PROCESS_COMPLETE_TOPIC, 'std_msgs/Bool')
  print("Starting setpoint controls status scubscriber callback")
  # Start setpoint control process complete monitor
  print("Starting mavros setpoint control subscriber topics")
  rospy.Subscriber(MAVROS_CONTROL_PROCESS_COMPLETE_TOPIC, Bool, setpoint_complete_status_callback)
  while setpoint_complete_status is None and not rospy.is_shutdown():
    print("Waiting for setpoint complete status to publish")
    print(setpoint_complete_status)
    time.sleep(0.1)
  ##############################
  # Update Global Location source
  set_gps_pub = rospy.Publisher(NEPI_NAVPOSE_SET_GPS_TOPIC, String, queue_size=1)
  time.sleep(.1) # Wait between creating and using publisher
  set_gps_pub.publish(MAVROS_SOURCE_GPS_TOPIC)
  print("GPS Topic Set to: " + MAVROS_SOURCE_GPS_TOPIC)
  ##############################
  # Update Orientation source
  set_orientation_pub = rospy.Publisher(NEPI_NAVPOSE_SET_ORIENTATION_TOPIC, String, queue_size=1)
  time.sleep(.1) # Wait between creating and using publisher
  set_orientation_pub.publish(MAVROS_SOURCE_ODOM_TOPIC)
  print("Orientation Topic Set to: " + MAVROS_SOURCE_ODOM_TOPIC)
  ##############################
  # Update Heading source
  set_heading_pub = rospy.Publisher(NEPI_NAVPOSE_SET_HEADING_TOPIC, String, queue_size=1)
  time.sleep(.1) # Wait between creating and using publisher
  set_heading_pub.publish(MAVROS_SOURCE_HEADING_TOPIC)
  print("Heading Topic Set to: " + MAVROS_SOURCE_HEADING_TOPIC)
  ##############################
  # Sync NEPI clock to GPS timestamp
  set_gps_timesync_pub = rospy.Publisher(NEPI_NAVPOSE_ENABLE_GPS_CLOCK_SYNC_TOPIC, Bool, queue_size=1)
  time.sleep(.1) # Wait between creating and using publisher
  set_gps_timesync_pub.publish(data=True)
  print("GPS Clock Sync Topic Set ")
  print("Completed Initialization")

## Function for custom post setpoint operations
def pre_mission_actions():
  ###########################
  # Start Your Custom Actions
  ###########################
  # Update Home Position
  sethome_current()
  # Set Mode to Guided
  update_mode('GUIDED')
  # Arm System
  update_armed(True)
  # Send Takeoff Command
  takeoff_height = 10.0
  takeoff(takeoff_height)
  ###########################
  # Stop Your Custom Actions
  ###########################
  print("Pre-Mission Actions Complete")

## Function for custom setpoint operations
def setpoint_actions():
  ###########################
  # Start Your Custom Actions
  ###########################
  ## Change Vehicle Mode to Guided
  print("Sending snapshot event trigger")
  snapshot(5)
  ###########################
  # Stop Your Custom Actions
  ###########################
  print("Setpoing Actions Complete")

  
## Function for custom post setpoint operations
def post_mission_actions():
  ###########################
  # Start Your Custom Actions
  ###########################
  #land() # Uncomment to change to Land mode
  #loiter() # Uncomment to change to Loiter mode
  rtl() # Uncomment to change to Home mode
  #continue_mission() # Uncomment to return to pre setpoint state
  time.sleep(1)
  ###########################
  # Stop Your Custom Actions
  ###########################
  print("Post-Mission Actions Complete")
  

### Callback to update setpoint process status value
def setpoint_complete_status_callback(status_msg):
  global setpoint_complete_status
  setpoint_complete_status = status_msg.data

### Function to call Setpoint Location Global control
def setpoint_location_global(setpoint_data):
  global setpoint_command_location_pub
  # Send Setpoint Location Command
  wait_for_setpoint_clear()
  print("Starting Setpoint Location Global Process")
  setpoint_location_msg = create_setpoint_message(setpoint_data)
  setpoint_command_location_pub.publish(setpoint_location_msg)
  wait_for_setpoint_started()
  wait_for_setpoint_clear()

### Function to call Setpoint Position Body control
def setpoint_position_body(setpoint_data):
  global setpoint_command_location_pub
  # Send Setpoint Position Command
  wait_for_setpoint_clear()
  print("Starting Setpoint Position Body Process")
  setpoint_position_msg = create_setpoint_message(setpoint_data)
  setpoint_command_position_pub.publish(setpoint_position_msg)
  wait_for_setpoint_started()
  wait_for_setpoint_clear()

### Function to call Setpoint Attititude NED control
def setpoint_attitude_ned(setpoint_data):
  global setpoint_command_attitude_pub
  # Send Setpoint Attitude Command
  wait_for_setpoint_clear()
  print("Starting Setpoint Attitude NED Process")
  setpoint_attitude_msg = create_setpoint_message(setpoint_data)
  setpoint_command_attitude_pub.publish(setpoint_attitude_msg)
  wait_for_setpoint_started()
  wait_for_setpoint_clear()
  
### Function to wait for setpoint control process to complete
def wait_for_setpoint_clear():
  global setpoint_complete_status
  while setpoint_complete_status is not True and not rospy.is_shutdown():
    print("Waiting for current setpoint control process to complete")
    print(setpoint_complete_status)
    time.sleep(1)

### Function to wait for setpoint control process to complete
def wait_for_setpoint_started():
  global setpoint_complete_status
  while setpoint_complete_status is not False and not rospy.is_shutdown():
    print("Waiting for current setpoint control process to start")
    print(setpoint_complete_status)
    time.sleep(1)

### Function to wait for topic to exist
def wait_for_topic(topic_name,message_name):
  topic_in_list = False
  while topic_in_list is False and not rospy.is_shutdown():
    topic_list=rospy.get_published_topics(namespace='/')
    topic_to_connect=[topic_name, message_name]
    if topic_to_connect not in topic_list:
      time.sleep(.1)
    else:
      topic_in_list = True


### Function to send snapshot event trigger and wait for completion
def snapshot(wait_sec):
  global snapshot_trigger_pub
  snapshot_trigger_pub.publish(Empty())
  time.sleep(wait_sec)

### Function for creating setpoint messages
def create_setpoint_message(setpoint):
  print(setpoint)
  setpoint_msg = Float64MultiArray()
  setpoint_data=[]
  for ind in range(len(setpoint)):
    setpoint_data.append(float(setpoint[ind]))
  print(setpoint_data)
  setpoint_msg.data = setpoint_data
  print("")
  print("Setpoint Message Created")
  print(setpoint_msg)
  return setpoint_msg


### Cleanup processes on node shutdown
def cleanup_actions():
  print("Shutting down: Executing script cleanup actions")

  
### Script Entrypoint
def startNode():
  rospy.loginfo("Starting MAVROS Setpoint Mission Target automation script")
  rospy.init_node("mavros_setpoint_mission_auto_script")
  #initialize system including pan scan process
  initialize_actions()
  #########################################
  # Run Pre-Mission Custom Actions
  print("Starting Pre-Setpoint Actions")
  pre_mission_actions()
  #########################################
  # Start Mission
  #########################################
  # Send Setpoint Location Command
  print("Starting Setpoint Location Global Process")
  setpoint_location_global(SETPOINT_LOCATION_GLOBAL)
  ##########################################
  # Send Setpoint Position Command
  print("Starting Setpoint Position Local Process")
  setpoint_position_body(SETPOINT_POSITION_BODY)
  #########################################
  # Send Setpoint Attitude Command
  print("Sending Setpoint Attitude Control Message")
  setpoint_attitude_ned(SETPOINT_ATTITUDE_NED)
  #########################################
  # Run Setpoint Actions
  print("Starting Setpoint Actions")
  setpoint_actions()
 #########################################
  # Send Setpoint Location Loop Command
  for ind in range(4):
    # Send Setpoint Location Command
    print("Starting Setpoint Location Corners Process")
    setpoint_location_global(SETPOINT_CORNERS_GLOBAL[ind])
    # Run Setpoint Actions
    print("Starting Setpoint Actions")
    setpoint_actions()
  #########################################
  # End Mission
  #########################################
  # Run Post-Mission Actions
  print("Starting Post-Setpoint Actions")
  post_mission_actions()
  #########################################
  # Mission Complete, Shutting Down
  print("Shutting Mission Restarting in 20 Seconds")
  time.sleep(20)
  rospy.signal_shutdown("Mission Complete, Shutting Down")
  #########################################
  # Run cleanup actions on rospy shutdown
  rospy.on_shutdown(cleanup_actions)
  # Spin forever
  rospy.spin()


#####################################################################################
# Main
#####################################################################################

if __name__ == '__main__':
  startNode()

