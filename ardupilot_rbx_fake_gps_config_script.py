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
# Uses onboard ROS and MAVROS libraries to
# 1) Publishes a fake GPS MAVLink Message 
# Provides two ROS control topics
# a) goto_geopoint_wgs84 - Simulates move to new geopoint
# b) reset_geopoint_wgs84 - Resets GPS and global x,y NED home position at new geopoint
# c) reset_current - Resets GPS and global x,y NED home position at current geopoint
# d) subscribe to RBX command topics and apply position moves
# Fake GPS control messages take a GeoPoint with WGS84 Height for Atlitude

# Requires the following additional scripts are running
# a) None
# These scripts are available for download at:
# [link text](https://github.com/numurus-nepi/nepi_sample_auto_scripts)

###################################################
### These Parameters Must Be Configured First to allow MAVLINK GPS source
### There maybe better configuration options, lots of knobs to play with
#GPS_TYPE = 14
#GPS_DELAY_MS = 1
#EK3_POS_I_GATE = 300
#EK3_POSNE_M_NSE = 5
#EK3_SRC_OPTIONS = 0
#EK3_SRC1_POSXY = 3
#EK3_SRC1_POSZ = 3
#EK3_SRC1_VELXY = 3
#EK3_SRC1_VELZ = 3
#EK3_SRC1_YAW = 1
#BARO_OPTION = 1  (This was required for proper barometer reading on Pixhawk)
#####################################################


import rospy
import time
import numpy as np
import math
import random

from std_msgs.msg import Empty, UInt8, Int8, Float64, Float64MultiArray, Header
from mavros_msgs.msg import HilGPS, State
from geometry_msgs.msg import Point
from geographic_msgs.msg import GeoPoint
from mavros_msgs.srv import CommandHome
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped
from nepi_ros_interfaces.srv import NavPoseQuery, NavPoseQueryRequest

###################################################
# SETUP - Edit as Necessary 
##########################################

###################################################
# RBX State and Mode Dictionaries
RBX_MODE_FUNCTIONS = ["stabilize","land","rtl","loiter","guided","resume"]
RBX_ACTION_FUNCTIONS = ["takeoff"]
TAKEOFF_ALT_M = 10

#Startup Location
# [Lat, Long, Altitude_WGS84]
FAKE_GPS_START_GEOPOINT_WGS84 = [47.6540828,-122.3187578,0.0]

#GPS Setup
SAT_COUNT = 20
GPS_PUB_RATE_HZ = 50

#GPS Simulation Position Update Controls
#Adjust these settings for smoother xyz movements
MOVE_UPDATE_TIME_SEC=10
MOVE_UPDATE_STEPS = GPS_PUB_RATE_HZ*MOVE_UPDATE_TIME_SEC # Number of moves to reach new position

PRINT_LOCATION_1HZ = False # Print current location at 1Hz

# ROS namespace setup
NEPI_BASE_NAMESPACE = "/nepi/s2x/"
NEPI_NAVPOSE_SERVICE_NAME = NEPI_BASE_NAMESPACE + "nav_pose_query"
NEPI_RBX_NAMESPACE = NEPI_BASE_NAMESPACE + "ardupilot/rbx/"


MAVLINK_NAMESPACE = NEPI_BASE_NAMESPACE + "mavlink/"

# MAVLINK Fake GPS Publish Topic
MAVLINK_HILGPS_TOPIC = MAVLINK_NAMESPACE + "hil/gps"

# MAVLINK Fake GPS Control Topic
MAVLINK_FAKE_GPS_GOTO_GEOPOINT_TOPIC = MAVLINK_NAMESPACE + "fake_gps/goto_geopoint_wgs84"
MAVLINK_FAKE_GPS_RESET_GEOPOINT_TOPIC = MAVLINK_NAMESPACE + "fake_gps/reset_geopoint_wgs84"
MAVLINK_FAKE_GPS_RESET_CURRENT_TOPIC = MAVLINK_NAMESPACE + "fake_gps/reset_current"

# MAVLINK Fake GPS RBX Subscriber Topic
NEPI_RBX_GOTO_POSITION_TOPIC = NEPI_RBX_NAMESPACE + "goto_position"
NEPI_RBX_GOTO_LOCATION_TOPIC = NEPI_RBX_NAMESPACE + "goto_location"
NEPI_RBX_SET_HOME_CURRENT_TOPIC = NEPI_RBX_NAMESPACE + "set_home_current"
NEPI_RBX_SET_MODE_TOPIC = NEPI_RBX_NAMESPACE + "set_mode"  # Int to Defined Dictionary RBX_MODES
NEPI_RBX_SET_ACTION_TOPIC = NEPI_RBX_NAMESPACE + "set_action"  # Int to Defined Dictionary RBX_MODES


#####################################################################################
# Globals
#####################################################################################
mavlink_fake_gps_mavlink_pub = rospy.Publisher(MAVLINK_HILGPS_TOPIC, HilGPS, queue_size=1)
current_location_wgs84_geo = None
current_heading_deg = None
current_home = None
navpose_update_interval = 0.1
fake_gps_enabled = True
gps_publish_interval_sec=1.0/GPS_PUB_RATE_HZ
#####################################################################################
# Methods
##########f###########################################################################

### System Initialization processes
def initialize_actions():
  global current_location_wgs84_geo
  global current_heading_deg
  global current_home
  # Initialize Current Location
  current_location_wgs84_geo=GeoPoint()
  current_location_wgs84_geo.latitude = FAKE_GPS_START_GEOPOINT_WGS84[0]
  current_location_wgs84_geo.longitude =FAKE_GPS_START_GEOPOINT_WGS84[1]
  current_location_wgs84_geo.altitude = FAKE_GPS_START_GEOPOINT_WGS84[2]
  current_home_wgs84_geo=GeoPoint()
  current_home_wgs84_geo.latitude = FAKE_GPS_START_GEOPOINT_WGS84[0]
  current_home_wgs84_geo.longitude =FAKE_GPS_START_GEOPOINT_WGS84[1]
  current_home_wgs84_geo.altitude = FAKE_GPS_START_GEOPOINT_WGS84[2]
  current_home = current_home_wgs84_geo
  #Start fake gps subscribers
  rospy.Subscriber(MAVLINK_FAKE_GPS_GOTO_GEOPOINT_TOPIC, GeoPoint, mavlink_fake_gps_goto_geopoint_callback)
  rospy.Subscriber(MAVLINK_FAKE_GPS_RESET_GEOPOINT_TOPIC, GeoPoint, mavlink_fake_gps_reset_geopoint_callback)
  rospy.Subscriber(MAVLINK_FAKE_GPS_RESET_CURRENT_TOPIC, Empty, mavlink_fake_gps_reset_current_callback)

  #Start fake gps rbx control subscribers
  rospy.Subscriber(NEPI_RBX_GOTO_POSITION_TOPIC, Float64MultiArray, rbx_goto_position_callback)
  rospy.Subscriber(NEPI_RBX_GOTO_LOCATION_TOPIC, Float64MultiArray, rbx_goto_location_callback)
  rospy.Subscriber(NEPI_RBX_SET_HOME_CURRENT_TOPIC, Empty, rbx_set_home_current_callback)
  rospy.Subscriber(NEPI_RBX_SET_MODE_TOPIC, UInt8, rbx_set_mode_callback)
  rospy.Subscriber(NEPI_RBX_SET_ACTION_TOPIC, UInt8, rbx_set_action_callback)
  
  # Start Print Callback if Enabled
  if PRINT_LOCATION_1HZ:
    rospy.Timer(rospy.Duration(1.0), mavlink_fake_gps_print_callback)
  #####
  rospy.Timer(rospy.Duration(navpose_update_interval), update_current_heading_callback)
  while current_heading_deg is None and not rospy.is_shutdown():
    print("Waiting for current heading from navpose service call")
    time.sleep(0.1)
  print("Initialization Complete")

#######################
# mavlink Interface Methods

### Setup a regular Send Fake GPS callback using current geo point value
def mavlink_fake_gps_pub_callback(timer):
  global mavlink_fake_gps_mavlink_pub
  global current_location_wgs84_geo
  global fake_gps_enabled
  if fake_gps_enabled:
    hilgps=HilGPS()
    hilgps.header = Header(stamp=rospy.Time.now(), frame_id="mavlink_fake_gps")
    hilgps.fix_type=3
    hilgps.geo.latitude=current_location_wgs84_geo.latitude
    hilgps.geo.longitude=current_location_wgs84_geo.longitude
    hilgps.geo.altitude=current_location_wgs84_geo.altitude
    hilgps.satellites_visible=SAT_COUNT
    ##print("Created new HilGPS message")
    ##print(hilgps)
    # Create and publish Fake GPS Publisher
    hilgps.header = Header(stamp=rospy.Time.now(), frame_id="mavlink_fake_gps")
    if not rospy.is_shutdown():
      mavlink_fake_gps_mavlink_pub.publish(hilgps)


### Callback to simulate move to new global geo position
def mavlink_fake_gps_goto_geopoint_callback(geopoint_msg):
  global current_location_wgs84_geo
  print('')
  print('***********************')
  print("Recieved Fake GPS Goto Message")
  print('***********************')
  print('')
  print(geopoint_msg)
  mavlink_fake_gps_move(geopoint_msg)


### function to simulate move to new global geo position
def mavlink_fake_gps_move(geopoint_msg):
  global current_location_wgs84_geo
  print('***********************')
  print("Moving FROM, TO, DELTA")
  print('***********************')
  print(current_location_wgs84_geo)
  print('')
  print(geopoint_msg)
  org_geo=np.array([current_location_wgs84_geo.latitude, current_location_wgs84_geo.longitude, current_location_wgs84_geo.altitude])
  new_geo=np.array([geopoint_msg.latitude, geopoint_msg.longitude, geopoint_msg.altitude])
  for ind, val in enumerate(new_geo):
    if new_geo[ind] == -999.0: # Use current
      new_geo[ind]=org_geo[ind]
  delta_geo = new_geo - org_geo
  print('')
  print(delta_geo)
  ramp=np.hanning(MOVE_UPDATE_STEPS)
  ramp=ramp**2
  ramp_norm=ramp/np.sum(ramp)
  step_norm=np.zeros(len(ramp_norm))
  for ind, val in enumerate(ramp_norm):
    step_norm[ind]=np.sum(ramp_norm[0:ind])
  stp_interval_sec = float(MOVE_UPDATE_TIME_SEC)/float(MOVE_UPDATE_STEPS)
  print_timer = 0
  for ind, val in enumerate(step_norm):
    time.sleep(stp_interval_sec)
    print_timer = print_timer + stp_interval_sec
    cur_geo_step = delta_geo * val
    cur_geo = org_geo + cur_geo_step
    current_location_wgs84_geo.latitude = cur_geo[0]
    current_location_wgs84_geo.longitude = cur_geo[1]
    current_location_wgs84_geo.altitude = cur_geo[2]
    if print_timer > 0.5:
      print("")
      print("Updated to")
      print(current_location_wgs84_geo)
      print_timer=0
  print('')
  print('***********************')
  print("Moving Complete")
  print('***********************')



### Callback to Reset GPS position and global x,y NED home position
def mavlink_fake_gps_reset_geopoint_callback(geopoint_msg):
  global current_location_wgs84_geo
  print('')
  print('***********************')
  print("Recieved Fake GPS Reset GeoPoint Message")
  print('***********************')
  print(geopoint_msg)
  org_geo=np.array([current_location_wgs84_geo.latitude, current_location_wgs84_geo.longitude, current_location_wgs84_geo.altitude])
  new_geo=np.array([geopoint_msg.latitude, geopoint_msg.longitude, geopoint_msg.altitude])
  for ind, val in enumerate(new_geo):
    if new_geo[ind] == -999.0: # Use current
      new_geo[ind]=org_geo[ind]
  current_location_wgs84_geo.latitude=new_geo[0]
  current_location_wgs84_geo.longitude=new_geo[1]
  current_location_wgs84_geo.altitude=new_geo[2]
  time.sleep(1)
  print("Reseting GPS with new location")
  reset_gps()
  print("Reset Complete")


  ### Callback to Reset GPS position and global x,y NED home position
def mavlink_fake_gps_reset_current_callback(empty_msg):
  print('')
  print('***********************')
  print("Recieved Fake GPS Reset Current Message")
  print('***********************')
  print("Reseting GPS with current location")
  reset_gps()
  print("Reset Complete")

### Function to reset gps and wait for position ned x,y to reset
def reset_gps():
  global fake_gps_enabled
  fake_gps_enabled = False
  print("Waiting for GPS to reset")  
  fake_gps_enabled = False
  time.sleep(MOVE_UPDATE_TIME_SEC)
  fake_gps_enabled = True


### Callback to print the current fake geopoint position at slower rate
def mavlink_fake_gps_print_callback(timer):
  global current_location_wgs84_geo
  print('')
  print("Current WGS84 Geo Location ")
  print(current_location_wgs84_geo)

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


#######################
# NEPI NavPose Interface Methods

### Setup a regular background navpose get and update heading data
def update_current_heading_callback(timer):
  global current_heading_deg
  # Get current NEPI NavPose data from NEPI ROS nav_pose_query service call
  try:
    get_navpose_service = rospy.ServiceProxy(NEPI_NAVPOSE_SERVICE_NAME, NavPoseQuery)
    nav_pose_response = get_navpose_service(NavPoseQueryRequest())
    current_heading_deg = nav_pose_response.nav_pose.heading.heading
  except Exception as e:
    print("navpose service call failed: " + str(e))

#######################
# RBX Driver Interface Methods

### Function to monitor RBX GoTo Position Command Topics
def rbx_goto_position_callback(position_cmd_msg):
  global current_location_wgs84_geo
  global current_heading_deg
  print("*******************************")
  print("Recieved GoTo Position Message")
  print("")
  print(position_cmd_msg)
  new_position = position_cmd_msg.data
  print("Sending Fake GPS Setpoint Position Update")
  new_geopoint_wgs84=get_geopoint_at_body_point(current_location_wgs84_geo, current_heading_deg, new_position)    
  mavlink_fake_gps_move(new_geopoint_wgs84)


### Function to monitor RBX GoTo Location Command Topics
def rbx_goto_location_callback(location_cmd_msg):
  print("*******************************")
  print("Recieved GoTo Location Message")
  print("")
  print(location_cmd_msg)
  new_location = location_cmd_msg.data
  new_geopoint_wgs84=GeoPoint()
  new_geopoint_wgs84.latitude = new_location[0]
  new_geopoint_wgs84.longitude = new_location[1]
  new_geopoint_wgs84.altitude = new_location[2]
  mavlink_fake_gps_move(new_geopoint_wgs84)

### Callback to set mode
def rbx_set_home_current_callback(set_home_msg):
  global current_location_wgs84_geo
  global current_home
  print("Recieved Set Home Message")
  current_home =  current_location_wgs84_geo

### Callback to set mode
def rbx_set_mode_callback(mode_msg):
  print("Recieved Set Mode Message")
  print(mode_msg)
  mode_ind = mode_msg.data
  if mode_ind < 0 or mode_ind > (len(RBX_MODE_FUNCTIONS)-1):
    print("No matching rbx mode found")
  else:
    set_mode_function = globals()[RBX_MODE_FUNCTIONS[mode_ind]]
    set_mode_function()

### Callback to set mode
def rbx_set_action_callback(action_msg):
  print("Recieved Set Action Message")
  print(action_msg)
  action_ind = action_msg.data
  if action_ind < 0 or action_ind > (len(RBX_ACTION_FUNCTIONS)-1):
    print("No matching rbx action found")
  else:
    set_action_function = globals()[RBX_ACTION_FUNCTIONS[action_ind]]
    set_action_function() 


#######################
# Mavlink Ardupilot Interface Methods
  
## Function for sending takeoff command
def takeoff():
  global current_location_wgs84_geo
  new_alt_m = current_location_wgs84_geo.altitude + TAKEOFF_ALT_M
  new_geopoint_wgs84=GeoPoint()
  new_geopoint_wgs84.latitude = current_location_wgs84_geo.latitude
  new_geopoint_wgs84.longitude = current_location_wgs84_geo.longitude
  new_geopoint_wgs84.altitude = new_alt_m
  mavlink_fake_gps_move(new_geopoint_wgs84)

### Function for switching to STABILIZE mode
def stabilize():
  print("")
    
### Function for switching to LAND mode
def land():
  global current_location_wgs84_geo
  new_geopoint_wgs84=GeoPoint()
  new_geopoint_wgs84.latitude = current_location_wgs84_geo.latitude
  new_geopoint_wgs84.longitude = current_location_wgs84_geo.longitude
  new_geopoint_wgs84.altitude = 0
  mavlink_fake_gps_move(new_geopoint_wgs84)
  
### Function for sending go home command
def rtl():
  global current_home
  mavlink_fake_gps_move(current_home)

### Function for switching to LOITER mode
def loiter():
  print("")

### Function for switching back to GUIDED mode
def guided():
  print("")

### Function for switching back to current mission
def resume():
  print("")

### Function for sending set home current
def sethome_current():
  print("")

#######################
# Process Functions

### Function to get new latlong at body relative point
def get_geopoint_at_body_point(cur_geopoint_geo, cur_bearing_deg, point_body_m):
  # cur_geopoint_geo is list [Lat,Long,Alt] with Alt passed through
  earth_radius_km = 6378.137
  earth_circ_m = np.float64(2 * math.pi * earth_radius_km*1000)
  # Calculate bearing in NED frame
  point_bearing_ned_deg = cur_bearing_deg + math.degrees(math.atan2(point_body_m[1],point_body_m[0]))
  point_bearing_ned_rad = math.radians(point_bearing_ned_deg)
  # Calculate distances NED frame
  delta_body_m = math.sqrt(point_body_m[0]**2+point_body_m[1]**2)
  delta_x_ned_m = delta_body_m * math.cos(point_bearing_ned_rad) # north:pos,south:neg
  delta_y_ned_m = delta_body_m * math.sin(point_bearing_ned_rad) # east:pos,west:neg
  # Calculate New Lat Position
  cur_lat = cur_geopoint_geo.latitude
  m_per_lat = np.float64(earth_circ_m/360)
  delta_lat = delta_x_ned_m / m_per_lat
  new_lat = cur_lat + delta_lat
  # Calculate New Long Position
  cur_long = cur_geopoint_geo.longitude
  m_per_long = m_per_lat * math.cos(math.radians(cur_lat)) 
  delta_long = delta_y_ned_m / m_per_long
  new_long = cur_long + delta_long
  # Return New Geo Position
  new_geopoint_geo=GeoPoint()
  new_geopoint_geo.latitude = new_lat
  new_geopoint_geo.longitude = new_long
  new_geopoint_geo.altitude = cur_geopoint_geo.altitude
  return  new_geopoint_geo


#######################
# StartNode and Cleanup Functions

### Cleanup processes on node shutdown
def cleanup_actions():
  print("Shutting down: Executing script cleanup actions")


### Script Entrypoint
def startNode():
  global gps_publish_interval_sec
  rospy.loginfo("Starting Ardupilot RBX Fake GPS Process Script")
  rospy.init_node("ardupilot_rbx_fake_gps_process_script")
  #initialize system including pan scan process
  initialize_actions()
  #Start fake gps publish and print callbacks
  print("Starting fake gps publishing to " + MAVLINK_HILGPS_TOPIC)
  rospy.Timer(rospy.Duration(gps_publish_interval_sec), mavlink_fake_gps_pub_callback)
  # run cleanup actions on shutdown
  rospy.on_shutdown(cleanup_actions)
  # Spin forever
  rospy.spin()

#####################################################################################
# Main
#####################################################################################

if __name__ == '__main__':
  startNode()
