#!/usr/bin/env python

__author__ = "Jason Seawall"
__copyright__ = "Copyright 2023, Numurus LLC"
__email__ = "nepi@numurus.com"
__credits__ = ["Jason Seawall", "Josh Maximoff"]

__license__ = "GPL"
__version__ = "2.0.4.0"

# Sample NEPI Automation Script.
# Uses onboard ROS python and mavros libraries to
# 1) Subscribes to setpoint command topics
#   a) setpoint_command_attitude
#   b) setpoint_command_position
#   c) setpoint_command_location
# 2) Processes mavros setpoint control commands for the folloing topics when called
#   a) setpoint_location_global_wgs84
#   b) setpoint_position_local_body
#   c) setpoint_attitude_ned
# 3)Publishes setpoint command status tipic

# Requires the following additional scripts are running
# a) navpose_get_and_publish_auto_script.py
# b) (Optional) mavros_navpose_config_auto_script to automate conecting NEPI NavPose with MAVLink
# c) (Optional) MAVROS_fake_gps_sim_auto_script.py if a real GPS fix is not available
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



# GENERAL Setpoint Settings
###################################################
SETPOINT_MAX_ERROR_M = 2.0 # Goal reached when all values within this error value
SETPOINT_MAX_ERROR_DEG = 2.0 # Goal reached when all values within this error value
SETPOINT_STABILIZED_WINDOW_SEC = 1.0 # Window of time that setpoint error values must be good before proceeding

# FAKE GPA SETTINGS
###################################################
# The Fake GPS Sim automation script is available at
# https://github.com/numurus-nepi/nepi_sample_auto_scripts
#####################################################
MAVROS_FAKE_GPS_SIM_SUPPORT = True # Set True if running "MAVROS_fake_gps_sim_auto_script.py"

# ROS namespace setup
NEPI_BASE_NAMESPACE = "/nepi/s2x/"
MAVROS_NAMESPACE = NEPI_BASE_NAMESPACE + "pixhawk_mavlink/"
MAVROS_CONTROLS_NAMESPACE = MAVROS_NAMESPACE + "controls/"
MAVROS_FAKE_GPS_NAMESPACE = MAVROS_NAMESPACE + "fake_gps/"

# NavPose Heading, Oreanation, Location, and Position Subscriber Topics
NAVPOSE_CURRENT_HEADING_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose_current/heading_deg"
NAVPOSE_CURRENT_ORIENTATION_NED_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose_current/orientation_ned_degs"
NAVPOSE_CURRENT_ORIENTATION_ENU_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose_current/orientation_enu_degs"
NAVPOSE_CURRENT_POSITION_NED_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose_current/position_ned_m"
NAVPOSE_CURRENT_POSITION_ENU_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose_current/position_enu_m"
NAVPOSE_CURRENT_LOCATION_AMSL_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose_current/location_amsl_geo"
NAVPOSE_CURRENT_LOCATION_WGS84_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose_current/location_wgs84_geo"
NAVPOSE_CURRENT_GEOID_HEIGHT_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose_current/geoid_height_m"

# MAVROS Setpoint Control Topics
MAVROS_SETPOINT_ATTITUDE_TOPIC = MAVROS_NAMESPACE + "setpoint_raw/attitude"
MAVROS_SETPOINT_POSITION_LOCAL_TOPIC = MAVROS_NAMESPACE + "setpoint_position/local"
MAVROS_SETPOINT_LOCATION_GLOBAL_TOPIC = MAVROS_NAMESPACE + "setpoint_position/global"

# NEPI MAVROS Control Subscriber Topics
MAVROS_CONTROL_ATTITUDE_TOPIC = MAVROS_CONTROLS_NAMESPACE + "setpoint_command_attitude"
MAVROS_CONTROL_POSITION_TOPIC = MAVROS_CONTROLS_NAMESPACE + "setpoint_command_position"
MAVROS_CONTROL_LOCATION_TOPIC = MAVROS_CONTROLS_NAMESPACE + "setpoint_command_location"
# NEPI MAVROS Control Publish Topics
MAVROS_CONTROL_PROCESS_COMPLETE_TOPIC = MAVROS_CONTROLS_NAMESPACE + "setpoint_complete_status"
MAVROS_CONTROL_PROCESS_COMPLETE_RATE_HZ = 10

# MAVROS Fake GPS Publish Topics
MAVROS_FAKE_GPS_GOTO_GEOPOINT_TOPIC = MAVROS_NAMESPACE + "fake_gps/goto_geopoint_wgs84"
MAVROS_FAKE_GPS_RESET_GEOPOINT_TOPIC = MAVROS_NAMESPACE + "fake_gps/reset_geopoint_wgs84"


#####################################################################################
# Globals
#####################################################################################
setpoint_process_complete_pub = rospy.Publisher(MAVROS_CONTROL_PROCESS_COMPLETE_TOPIC, Bool, queue_size=1)

current_heading_deg = None
current_orientation_enu_degs = None
current_orientation_ned_degs = None
current_position_enu_m = None
current_position_ned_m = None
current_location_amsl_geo = None
current_location_wgs84_geo = None
current_geoid_height_m = None

setpoint_complete_status = True
setpoint_process_complete_pub_interval = float(1.0)/float(MAVROS_CONTROL_PROCESS_COMPLETE_RATE_HZ)
                  
#####################################################################################
# Methods
#####################################################################################

### System Initialization processes
def initialize_actions():
  global current_heading_deg
  global current_orientation_enu_degs
  global current_orientation_ned_degs
  global current_position_enu_m
  global current_position_ned_m
  global current_location_amsl_geo
  global current_location_wgs84_geo
  global current_geoid_height_m
  ## Start Get Update State Subscriber Callback
  ## Start Update Heading Callback
  print("Starting heading subscriber callback")
  # Wait for topic
  print("Waiting for topic: " + NAVPOSE_CURRENT_HEADING_TOPIC)
  wait_for_topic(NAVPOSE_CURRENT_HEADING_TOPIC, 'std_msgs/Float64')
  rospy.Subscriber(NAVPOSE_CURRENT_HEADING_TOPIC, Float64, get_heading_callback)
  while current_heading_deg is None and not rospy.is_shutdown():
    print("Waiting for current heading to set")
    time.sleep(0.5)
  ## Start Update Orientation NED Callback
  # Wait for topic
  print("Waiting for topic: " + NAVPOSE_CURRENT_ORIENTATION_NED_TOPIC)
  wait_for_topic(NAVPOSE_CURRENT_ORIENTATION_NED_TOPIC, 'std_msgs/Float64MultiArray')
  print("Starting orientation ned subscriber callback")
  rospy.Subscriber(NAVPOSE_CURRENT_ORIENTATION_NED_TOPIC, Float64MultiArray, get_orientation_ned_callback)
  while current_orientation_ned_degs is None and not rospy.is_shutdown():
    print("Waiting for current orientaiton ned to publish")
    time.sleep(0.5)
  ## Start Update Orientation ENU Callback
  # Wait for topic
  print("Waiting for topic: " + NAVPOSE_CURRENT_ORIENTATION_ENU_TOPIC)
  wait_for_topic(NAVPOSE_CURRENT_ORIENTATION_ENU_TOPIC, 'std_msgs/Float64MultiArray')
  print("Starting orientation enu subscriber callback")
  rospy.Subscriber(NAVPOSE_CURRENT_ORIENTATION_ENU_TOPIC, Float64MultiArray, get_orientation_enu_callback)
  while current_orientation_enu_degs is None and not rospy.is_shutdown():
    print("Waiting for current orientaiton enu to publish")
    time.sleep(0.5)
  ## Start Update Position NED Callback
  # Wait for topic
  print("Waiting for topic: " + NAVPOSE_CURRENT_POSITION_NED_TOPIC)
  wait_for_topic(NAVPOSE_CURRENT_POSITION_NED_TOPIC, 'std_msgs/Float64MultiArray')
  print("Starting orientation ned subscriber callback")
  rospy.Subscriber(NAVPOSE_CURRENT_POSITION_NED_TOPIC, Float64MultiArray, get_position_ned_callback)
  while current_position_ned_m is None and not rospy.is_shutdown():
    print("Waiting for current position ned to publish")
    time.sleep(0.5)
  ## Start Update Position ENU Callback
  # Wait for topic
  print("Waiting for topic: " + NAVPOSE_CURRENT_POSITION_ENU_TOPIC)
  wait_for_topic(NAVPOSE_CURRENT_POSITION_ENU_TOPIC, 'std_msgs/Float64MultiArray')
  print("Starting orientation enu subscriber callback")
  rospy.Subscriber(NAVPOSE_CURRENT_POSITION_ENU_TOPIC, Float64MultiArray, get_position_enu_callback)
  while current_position_enu_m is None and not rospy.is_shutdown():
    print("Waiting for current position enu to publish")
    time.sleep(0.5)
  ## Start Update Global Location AMSL Callback
  # Wait for topic
  print("Waiting for topic: " + NAVPOSE_CURRENT_LOCATION_AMSL_TOPIC)
  wait_for_topic(NAVPOSE_CURRENT_LOCATION_AMSL_TOPIC, 'std_msgs/Float64MultiArray')
  print("Starting global location subscriber callback")
  rospy.Subscriber(NAVPOSE_CURRENT_LOCATION_AMSL_TOPIC, Float64MultiArray, get_location_amsl_callback)
  while current_location_amsl_geo is None and not rospy.is_shutdown():
    print("Waiting for current location amsl height to publish")
    time.sleep(0.5)
  ## Start Update Global Location WGS84 Callback
  # Wait for topic
  print("Waiting for topic: " + NAVPOSE_CURRENT_LOCATION_WGS84_TOPIC)
  wait_for_topic(NAVPOSE_CURRENT_LOCATION_WGS84_TOPIC, 'std_msgs/Float64MultiArray')
  print("Starting global location subscriber callback")
  rospy.Subscriber(NAVPOSE_CURRENT_LOCATION_WGS84_TOPIC, Float64MultiArray, get_location_wgs84_callback)
  while current_location_wgs84_geo is None and not rospy.is_shutdown():
    print("Waiting for current location wsg84 to publish")
    time.sleep(0.5)
  ## Start Update Geoid Height Callback
  # Wait for topic
  print("Waiting for topic: " + NAVPOSE_CURRENT_GEOID_HEIGHT_TOPIC)
  wait_for_topic(NAVPOSE_CURRENT_GEOID_HEIGHT_TOPIC, 'std_msgs/Float64')
  print("Starting geoid height subscriber callback")
  rospy.Subscriber(NAVPOSE_CURRENT_GEOID_HEIGHT_TOPIC, Float64, get_geoid_height_callback)
  while current_geoid_height_m is None and not rospy.is_shutdown():
    print("Waiting for current geoid height to publish")
    time.sleep(0.5)




### Function to set and check setpoint position global geopoint and yaw command
###################################################
# Input is [LAT, LONG, ALT_WGS84, YAW_NED_DEGREES]
# Converted to AMSL Altitude and ENU Yaw berore sending
# Altitude is specified as meters above the WGS-84 and converted to AMSL before sending
# Yaw is specified in NED frame degrees 0-360 or +-180 
#####################################################
def setpoint_location_global_wgs84(setpoint_location =[-999,-999,-999,-999]):
  # setpoint_location is [LAT, LONG, ALT_WGS84, YEW_NED_DEGREES 0-360 or +-180]
  # Use value -999 to use current value
  global current_orientation_ned_degs
  global current_geoid_height_m
  global current_location_wgs84_geo
  global current_heading_deg
  global setpoint_complete_status
  setpoint_complete_status = False
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
  ## Send Fake location update if enabled
  ##############################################
  if MAVROS_FAKE_GPS_SIM_SUPPORT:
    print("Sending Fake GPS Goto Setpoint Position")
    fake_gps_goto_geopoint(new_geopoint_wgs84)
  ##############################################
  ## Send Message and Check for Setpoint Success
  ##############################################
  setpoint_location_global_pub = rospy.Publisher(MAVROS_SETPOINT_LOCATION_GLOBAL_TOPIC, GeoPoseStamped, queue_size=1)
  time.sleep(1) # VERY IMPORTANT - Sleep a bit between declaring a publisher and using it
  print("Sending MAVROS Setpoint Position Local Command at 50 Hz and")
  print(" checking for Setpoint Reached")
  setpoint_location_global_geopoint_reached = False
  setpoint_location_global_yaw_reached = False 
  time_sec=0
  print('')
  print("Waiting for Position Local Setpoint to complete")
  geopoint_errors = [] # Initialize running list of errors
  yaw_errors = [] # Initialize running list of errors
  while setpoint_location_global_geopoint_reached is False or setpoint_location_global_yaw_reached is False and not rospy.is_shutdown(): # Wait for setpoint goal to be set
    time.sleep(0.02) # update setpoint position at 50 Hz
    time_sec=time_sec+0.02 # Increment print message timer
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
      if time_sec > SETPOINT_STABILIZED_WINDOW_SEC:
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
        if max_geopoint_errors < SETPOINT_MAX_ERROR_M:
          print('')
          print("Location Setpoint Reached")
          setpoint_location_global_geopoint_reached = True
      else:
        geopoint_errors.append(max_geopoint_error_m) # append last
    # Check for setpoint position yaw goal
    if  setpoint_location_global_yaw_reached is False:
      if time_sec > SETPOINT_STABILIZED_WINDOW_SEC:
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
        if max_yaw_errors < SETPOINT_MAX_ERROR_DEG:
          print('')
          print("Yaw Setpoint Reached")
          setpoint_location_global_yaw_reached = True
      else:
        yaw_errors.append(max_yaw_ned_error_deg) # append last
    # Reset print timer if past
    if time_sec > 1:
      time_sec=0 # Reset print timer
  print("************************")
  print("Setpoint Reached")
  setpoint_complete_status = True





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
def setpoint_position_local_body(setpoint_position = [0,0,0,0]):
  # setpoint_position is [X_BODY_METERS, Y_BODY_METERS, Z_BODY_METERS, YEW_BODY_DEGREES]
  # use value 0 for no change
  global current_orientation_ned_degs
  global current_position_ned_m
  global current_location_wgs84_geo
  global current_heading_deg
  global setpoint_position_local_pub
  global setpoint_complete_status
  setpoint_complete_status = False
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
  new_yaw_ned_deg = convert_yaw_body2ned(new_yaw_body_deg,start_heading_deg)
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
  ## Send Fake location update if enabled
  ##############################################
  if MAVROS_FAKE_GPS_SIM_SUPPORT:
    print("Sending Fake GPS Setpoint Position Update")
    new_geopoint_wgs84=get_geopoint_at_body_point(start_geopoint_wgs84, start_yaw_ned_deg, new_point_body_m)    
    fake_gps_goto_geopoint(new_geopoint_wgs84)
  ##############################################
  ## Send Message and Check for Setpoint Success
  ##############################################
  setpoint_position_local_pub = rospy.Publisher(MAVROS_SETPOINT_POSITION_LOCAL_TOPIC, PoseStamped, queue_size=1)
  time.sleep(1) # VERY IMPORTANT - Sleep a bit between declaring a publisher and using it
  print('')
  print("Sending Setpoint Position Local Command at 50 Hz and")
  print("Waiting for Attitude Setpoint to complete")
  setpoint_position_local_point_reached = False
  setpoint_position_local_yaw_reached = False
  time_sec=0
  point_errors = [] # Initialize running list of errors
  yaw_errors = [] # Initialize running list of errors
  while setpoint_position_local_point_reached is False or setpoint_position_local_yaw_reached is False and not rospy.is_shutdown():  # Wait for setpoint goal to be set
    time.sleep(0.02) # update setpoint position at 50 Hz
    time_sec=time_sec+0.02 # Increment print message timer
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
      if time_sec > SETPOINT_STABILIZED_WINDOW_SEC:
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
        if max_point_errors < SETPOINT_MAX_ERROR_M:
          print('')
          print("Position Setpoint Reached")
          setpoint_position_local_point_reached = True
      else:
        point_errors.append(max_point_ned_error_m) # append last
    # Check for setpoint position yaw point goal
    if  setpoint_position_local_yaw_reached is False:
      if time_sec > SETPOINT_STABILIZED_WINDOW_SEC:
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
        if max_yaw_errors < SETPOINT_MAX_ERROR_DEG:
          print('')
          print("Yaw Setpoint Reached")
          setpoint_position_local_yaw_reached = True
      else:
        yaw_errors.append(max_yaw_ned_error_deg) # append last
    # Reset print timer if past
    if time_sec > SETPOINT_STABILIZED_WINDOW_SEC:
      time_sec=0 # Reset print timer
  print("************************")
  print("Setpoint Reached")
  setpoint_complete_status = True



### Function to set and check setpoint attitude NED command
###################################################
# Input is [ROLL_NED_DEG, PITCH_NED_DEG, YEW_NED_DEGREES]
# Converted to ENU befor sending message
###################################################
def setpoint_attitude_ned(setpoint_attitude):
  # setpoint_attitude is [ROLL_NED_DEG, PITCH_NED_DEG, YEW_NED_DEGREES]
  # Use value -999 to use current value
  global current_orientation_enu_degs
  global current_orientation_ned_degs
  global current_heading_degs
  global setpoint_complete_status
  setpoint_complete_status = False
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
  setpoint_attitude_pub = rospy.Publisher(MAVROS_SETPOINT_ATTITUDE_TOPIC, AttitudeTarget, queue_size=1)
  time.sleep(1) # VERY IMPORTANT - Sleep a bit between declaring a publisher and using it  
  print('')
  print("Sending Setpoint Attitude Command at 50 Hz and")
  print("Waiting for Attitude Setpoint to complete")
  setpoint_attitude_reached = False
  time_sec=0
  attitude_errors = [] # Initialize running list of errors
  while setpoint_attitude_reached is False and not rospy.is_shutdown():  # Wait for setpoint goal to be set
    time.sleep(0.02) # update setpoint position at 50 Hz
    time_sec=time_sec+0.02 # Increment print message timer
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
      if time_sec > SETPOINT_STABILIZED_WINDOW_SEC:
        print(time_sec)
        print(SETPOINT_STABILIZED_WINDOW_SEC)
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
        if max_attitude_errors < SETPOINT_MAX_ERROR_DEG:
          print('')
          print("Attitude Setpoint Reached")
          setpoint_attitude_reached = True
      else:
        attitude_errors.append(max_attutude_error_deg) # append last
    # Reset print timer if past
    if time_sec > SETPOINT_STABILIZED_WINDOW_SEC:
      time_sec=0 # Reset print timer
  print("************************")
  print("Setpoint Attitude Reached")
  setpoint_complete_status = True
  


### Callback to get current heading
def get_heading_callback(heading_msg):
  global current_heading_deg
  current_heading_deg = heading_msg.data

### Callback to get current NED orientation
def get_orientation_ned_callback(orientation_ned_msg):
  global current_orientation_ned_degs
  current_orientation_ned_degs = list(orientation_ned_msg.data)

### Callback to get current ENU orientation
def get_orientation_enu_callback(orientation_enu_msg):
  global current_orientation_enu_degs
  current_orientation_enu_degs = list(orientation_enu_msg.data)

### Callback to get current NED position
def get_position_ned_callback(position_ned_msg):
  global current_position_ned_m
  current_position_ned_m = list(position_ned_msg.data)

### Callback to get current ENU position
def get_position_enu_callback(position_enu_msg):
  global current_position_enu_m
  current_position_enu_m = list(position_enu_msg.data)

### Callback to get current location amsl
def get_location_amsl_callback(location_amsl_msg):
  global current_location_amsl_geo
  current_location_amsl_geo = list(location_amsl_msg.data)

### Callback to get current location wgs84
def get_location_wgs84_callback(location_wgs84_msg):
  global current_location_wgs84_geo
  current_location_wgs84_geo = list(location_wgs84_msg.data)

### Callback to get current geoid height
def get_geoid_height_callback(geoid_height_msg):
  global current_geoid_height_m
  current_geoid_height_m = geoid_height_msg.data

### Function for simulating Fake GPS movement to new geopoint
def fake_gps_goto_geopoint(goto_geopoint_wgs84):
  global current_location_wgs84_geo
  # Send mavlink set home command and message
  new_geopoint_wgs84 = list(goto_geopoint_wgs84)
  for ind, val in enumerate(new_geopoint_wgs84):
    if new_geopoint_wgs84[ind] == -999.0: # Use current
      new_geopoint_wgs84[ind]=current_location_wgs84_geo[ind]
  print("Sending Fake GPS move command to:")
  fake_gps_goto_geopoint_pub = rospy.Publisher(MAVROS_FAKE_GPS_GOTO_GEOPOINT_TOPIC, GeoPoint, queue_size=1)
  time.sleep(.1)
  print(new_geopoint_wgs84)
  geopoint_msg=GeoPoint()
  geopoint_msg.latitude = new_geopoint_wgs84[0]
  geopoint_msg.longitude = new_geopoint_wgs84[1]
  geopoint_msg.altitude = new_geopoint_wgs84[2]
  fake_gps_goto_geopoint_pub.publish(geopoint_msg)


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
  cur_lat = cur_geopoint_geo[0]
  m_per_lat = np.float64(earth_circ_m/360)
  delta_lat = delta_x_ned_m / m_per_lat
  new_lat = cur_lat + delta_lat
  # Calculate New Long Position
  cur_long = cur_geopoint_geo[1]
  m_per_long = m_per_lat * math.cos(math.radians(cur_lat)) 
  delta_long = delta_y_ned_m / m_per_long
  # Return New Geo Position
  new_long = cur_long + delta_long
  new_geopoint_geo=[new_lat,new_long,cur_geopoint_geo[2]]
  return  new_geopoint_geo




### Callback to set setpoint command status publisher
def setpoint_process_complete_pub_callback(timer):
  global setpoint_complete_status
  global setpoint_process_complete_pub
  if not rospy.is_shutdown():
    setpoint_process_complete_pub.publish(data=setpoint_complete_status)


### Callback to set setpoint command status subscriber
def setpoint_command_location_callback(location_cmd_msg):
  global setpoint_complete_status
  print("*******************************")
  print("Recieved Setpoint Command Location Message")
  print("")
  print(location_cmd_msg)
  setpoint_data=list(location_cmd_msg.data)
  if len(setpoint_data) is not 4:
    print("Ignoring this Request")
    print("Messge is wrong length. Should be float list of size 4")
    print("[Lat,Long,Alt,Yaw]")
  else:
    if setpoint_complete_status is False:
      print("Another Setpoint Command Process is Active")
      print("Ignoring this Request")
    else:
      setpoint_location_global_wgs84(setpoint_data)


### Callback to start mavros setpoint position process
def setpoint_command_position_callback(position_cmd_msg):
  global setpoint_complete_status
  print("*******************************")
  print("Recieved Setpoint Command Position Message")
  print("")
  print(position_cmd_msg)
  setpoint_data=list(position_cmd_msg.data)
  if len(setpoint_data) is not 4:
    print("Ignoring this Request")
    print("Messge is wrong length. Should be float list of size 4")
    print("[X,Y,Z,Yaw]")
  else:    
    if setpoint_complete_status is False:
      print("Another Setpoint Command Process is Active")
      print("Ignoring this Request")
    else:
      setpoint_position_local_body(setpoint_data)


### Callback to start mavros setpoint attitidue process
def setpoint_command_attitude_callback(attitude_cmd_msg):
  global setpoint_complete_status
  print("*******************************")
  print("Recieved Setpoint Command Attitude Message")
  print("")
  print(attitude_cmd_msg)
  setpoint_data=list(attitude_cmd_msg.data)
  if len(setpoint_data) is not 3:
    print("Ignoring this Request")
    print("Messge is wrong length. Should be float list of size 3")
    print("[Roll,Pitch,Yaw]")
  else:
    if setpoint_complete_status is False:
      print("Another Setpoint Command Process is Active")
      print("Ignoring this Request")
    else:
      setpoint_attitude_ned(setpoint_data)



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


### Cleanup processes on node shutdown
def cleanup_actions():
  print("Shutting down: Executing script cleanup actions")

  
### Script Entrypoint
def startNode():
  rospy.loginfo("Starting MAVROS Setpoint Controls automation script")
  rospy.init_node("mavros_setpoint_controls_auto_script")
  #initialize system including pan scan process
  initialize_actions()
  print("Starting mavros setpoint control publisher and subscriber topics")
  rospy.Timer(rospy.Duration(setpoint_process_complete_pub_interval), setpoint_process_complete_pub_callback)
  rospy.Subscriber(MAVROS_CONTROL_ATTITUDE_TOPIC, Float64MultiArray, setpoint_command_attitude_callback)
  rospy.Subscriber(MAVROS_CONTROL_POSITION_TOPIC, Float64MultiArray, setpoint_command_position_callback)
  rospy.Subscriber(MAVROS_CONTROL_LOCATION_TOPIC, Float64MultiArray, setpoint_command_location_callback)
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

