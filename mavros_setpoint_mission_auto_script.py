#!/usr/bin/env python

# Sample NEPI Automation Script.
# Uses onboard ROS python and mavros libraries to
# 1) Subscribes to NEPI nav_pose_current heading, orientation, position, location topics
# 2) Runs pre-mission processes
# 3) Runs mission setpoint command processes
# 4) Runs mission setpoint action processes
# 5) Runs post-mission processes

# Requires the following additional scripts are running
# a) mavros_navpose_config_and_publish_auto_script.py
# b) (Optional) MAVROS_fake_gps_sim_auto_script.py if a real GPS fix is not available
# These scripts are available for download at:
# [link text](https://github.com/numurus-nepi/nepi_sample_auto_scripts)

import rospy
import time
import numpy as np
import math
import tf

from std_msgs.msg import Empty, String, Float64, Float64MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped
from geographic_msgs.msg import GeoPoint, GeoPoseStamped
from mavros_msgs.msg import State, AttitudeTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL, CommandHome

#####################################################################################
# SETUP - Edit as Necessary ##################################
##########################################

# Home Settings
###################################################
## Specified as [Lat, Long, Altitude_Meters] or -999 values to use current
## Altitude is specified as meters in WGS-84 Ellipsoid height
######################################################
HOME_POSITION = [47.6541207,-122.3187998,0.0] # [Lat, Long, Altitude_Meters]. Replace any value with -999 to use current global val
#HOME_POSITION = [-999,-999,-999] # us to update home to current position
MODE_UPDATE_MAX_POSITION_ERROR_M = 2.0 # Goal used to check normal mode position changes like land and home

# Setpoint Attitude Settings
###################################################
SETPOINT_ATTITUDE_NED_DEGS = [-999,30,90] # Roll, Pitch, Yaw Degrees: Enter -999 to use current value
SETPOINT_ATTITUDE_MAX_ERROR_DEG = 2.0 # Goal reached when all values within this error value
SETPOINT_ATTITUDE_THRUST = 0.2

# Setpoint Position Local Settings
###################################################
# Local Position Updates X,Y,Z use these vehicle relative conventions
# x+ axis is forward
# y+ axis is right
# z+ axis is down ***
# Only yaw orientation updated
#####################################################
SETPOINT_POSITION_LOCAL_POINT_M = [10,5,0] # X, Y, Z Offset in Meters (+Z is Down)
SETPOINT_POSITION_LOCAL_YAW_DEG = 80 # Yaw Angle in Degrees, Enter -999 to use current value
SETPOINT_POSITION_LOCAL_MAX_ERROR_M = 2.0 # Goal reached when all values within this error value
SETPOINT_POSITION_LOCAL_MAX_ERROR_DEG = 2.0 # Goal reached when all values within this error value

# Setpoint Position Global Settings
###################################################
# Accepts a list of global [Lat,Long,Altitude_M,Global_Heading_Deg]
# Accepts a Mission Planner Absolute "waypoints file" with 8th column used for Global_Heading_Deg value
# Altitude is specified as meters above the WGS-84 and converted to AMSL before sending
# Heading is specified in global heading degrees and conveted to local pose
#####################################################
SETPOINT_POSITION_GLOBAL_WAYPOINTS = [10,5,0,80] # List of [Lat,Long,Altitude,Global_Heading_Deg] waypoints
SETPOINT_POSITION_GLOBAL_WAYPOINTS_FILE = '/mnt/nepi_storage/missions/NumurusCorners.waypoints'
SETPOINT_POSITION_GLOBAL_MAX_ERROR_M = 2.0 # Goal reached when all values within this error value
SETPOINT_POSITION_GLOBAL_MAX_ERROR_DEG = 2.0 # Goal reached when all values within this error value

# SOME ACTION SETTINGS
SNAPSHOT_EVENT_WAIT_SEC = 5.0 # Time to wait for snapshot event to complete


# FAKE GPA SETTINGS
###################################################
# The Fake GPS Sim automation script is available at
# https://github.com/numurus-nepi/nepi_sample_auto_scripts
#####################################################
MAVROS_FAKE_GPS_SIM_SUPPORT = True # Set True if running "MAVROS_fake_gps_sim_auto_script.py"
MAVROS_FAKE_GPS_UPDATE_TIME_SEC=10 # Match setting in fake gps sim automation script

# ROS namespace setup
BASE_NAMESPACE = "/nepi/s2x/"
MAVROS_NAMESPACE = BASE_NAMESPACE + "pixhawk_mavlink/"
MAVROS_CONTROLS_NAMESPACE = MAVROS_NAMESPACE + "backseat_controls/"
MAVROS_FAKE_GPS_NAMESPACE = MAVROS_NAMESPACE + "fake_gps/"

# NavPose Heading, Oreanation, Location, and Position Subscriber Topics
NAVPOSE_CURRENT_HEADING_TOPIC = BASE_NAMESPACE + "nav_pose_current/heading_deg"
NAVPOSE_CURRENT_ORIENTATION_NED_TOPIC = BASE_NAMESPACE + "nav_pose_current/orientation_ned_degs"
NAVPOSE_CURRENT_ORIENTATION_ENU_TOPIC = BASE_NAMESPACE + "nav_pose_current/orientation_enu_degs"
NAVPOSE_CURRENT_POSITION_NED_TOPIC = BASE_NAMESPACE + "nav_pose_current/position_ned_m"
NAVPOSE_CURRENT_POSITION_ENU_TOPIC = BASE_NAMESPACE + "nav_pose_current/position_enu_m"
NAVPOSE_CURRENT_LOCATION_AMSL_TOPIC = BASE_NAMESPACE + "nav_pose_current/location_amsl_geo"
NAVPOSE_CURRENT_LOCATION_WGS84_TOPIC = BASE_NAMESPACE + "nav_pose_current/location_wgs84_geo"
NAVPOSE_CURRENT_GEOID_HEIGHT_TOPIC = BASE_NAMESPACE + "nav_pose_current/geoid_height_m"

# MAVROS Subscriber Topics
MAVROS_STATE_TOPIC = MAVROS_NAMESPACE + "state"

# MAVROS Required Services
MAVROS_SET_HOME_SERVICE = MAVROS_NAMESPACE + "cmd/set_home"
MAVROS_SET_MODE_SERVICE = MAVROS_NAMESPACE + "set_mode"
MAVROS_ARMING_SERVICE = MAVROS_NAMESPACE + "cmd/arming"
MAVROS_TAKEOFF_SERVICE = MAVROS_NAMESPACE + "cmd/takeoff"

# MAVROS Setpoint Control Topics
MAVROS_SETPOINT_ATTITUDE_TOPIC = MAVROS_NAMESPACE + "setpoint_raw/attitude"
MAVROS_SETPOINT_POSITION_LOCAL_TOPIC = MAVROS_NAMESPACE + "setpoint_position/local"
MAVROS_SETPOINT_POSITION_GLOBAL_TOPIC = MAVROS_NAMESPACE + "setpoint_position/global"

# Setpoint Action Topics
SNAPSHOT_TOPIC = BASE_NAMESPACE + "snapshot_event"

# MAVROS Fake GPS Publish Topics
MAVROS_FAKE_GPS_SETHOME_TOPIC = MAVROS_FAKE_GPS_NAMESPACE + "sethome"
MAVROS_FAKE_GPS_TAKEOFF_TOPIC = MAVROS_FAKE_GPS_NAMESPACE + "takeoff"
MAVROS_FAKE_GPS_UPDATE_GLOBAL_TOPIC = MAVROS_FAKE_GPS_NAMESPACE + "update_global"
MAVROS_FAKE_GPS_GOTO_LOCAL_TOPIC = MAVROS_FAKE_GPS_NAMESPACE + "goto_local"
MAVROS_FAKE_GPS_GOTO_GLOBAL_TOPIC = MAVROS_FAKE_GPS_NAMESPACE + "goto_global"
MAVROS_FAKE_GPS_GOHOME_TOPIC = MAVROS_FAKE_GPS_NAMESPACE + "gohome"
MAVROS_FAKE_GPS_LAND_TOPIC = MAVROS_FAKE_GPS_NAMESPACE + "land"

#####################################################################################
# Globals
#####################################################################################
mavros_fake_gps_sethome_pub = rospy.Publisher(MAVROS_FAKE_GPS_SETHOME_TOPIC, GeoPoint, queue_size=1)
mavros_fake_gps_takeoff_pub = rospy.Publisher(MAVROS_FAKE_GPS_TAKEOFF_TOPIC, Float64, queue_size=1)
mavros_fake_gps_goto_local_pub = rospy.Publisher(MAVROS_FAKE_GPS_GOTO_LOCAL_TOPIC, Point, queue_size=1)
mavros_fake_gps_goto_global_pub = rospy.Publisher(MAVROS_FAKE_GPS_GOTO_GLOBAL_TOPIC, GeoPoint, queue_size=1)
mavros_fake_gps_gohome_pub = rospy.Publisher(MAVROS_FAKE_GPS_GOHOME_TOPIC, Empty, queue_size=1)
mavros_fake_gps_land_pub = rospy.Publisher(MAVROS_FAKE_GPS_LAND_TOPIC, Empty, queue_size=1)
snapshot_trigger_pub = rospy.Publisher(SNAPSHOT_TOPIC, Empty, queue_size = 1)

current_state = None
original_state = None
current_home = None
current_heading_deg = None
current_orientation_enu_degs = None
current_orientation_ned_degs = None
current_position_enu_meters = None
current_position_ned_meters = None
current_location_amsl_geo = None
current_location_wgs84_geo = None
current_geoid_height_m = None
                  
#####################################################################################
# Methods
#####################################################################################

### System Initialization processes
def initialize_actions():
  global current_state
  global original_state
  global current_heading_deg
  global current_orientation_enu_degs
  global current_orientation_ned_degs
  global current_position_enu_meters
  global current_position_ned_meters
  global current_location_amsl_geo
  global current_location_wgs84_geo
  global current_geoid_height_m
  ## Start Get Update State Subscriber Callback
  print("Starting state scubscriber callback")
  print(MAVROS_STATE_TOPIC)
  rospy.Subscriber(MAVROS_STATE_TOPIC, State, get_state_callback)
  while current_state is None:
    print("Waiting for current state to set")
    time.sleep(0.1)
  print(current_state)
  original_state = current_state
  ## Start Update Heading Callback
  print("Starting heading subscriber callback")
  print(NAVPOSE_CURRENT_HEADING_TOPIC)
  rospy.Subscriber(NAVPOSE_CURRENT_HEADING_TOPIC, Float64, get_heading_callback)
  while current_heading_deg is None:
    print("Waiting for current heading to set")
    time.sleep(0.5)
  ## Start Update Orientation NED Callback
  print("Starting orientation ned subscriber callback")
  print(NAVPOSE_CURRENT_ORIENTATION_NED_TOPIC)
  rospy.Subscriber(NAVPOSE_CURRENT_ORIENTATION_NED_TOPIC, Float64MultiArray, get_orientation_ned_callback)
  while current_orientation_ned_degs is None:
    print("Waiting for current orientaiton ned to set")
    time.sleep(0.5)
  ## Start Update Orientation ENU Callback
  print("Starting orientation enu subscriber callback")
  print(NAVPOSE_CURRENT_ORIENTATION_ENU_TOPIC)
  rospy.Subscriber(NAVPOSE_CURRENT_ORIENTATION_ENU_TOPIC, Float64MultiArray, get_orientation_enu_callback)
  while current_orientation_enu_degs is None:
    print("Waiting for current orientaiton enu to set")
    time.sleep(0.5)
  ## Start Update Position NED Callback
  print("Starting orientation ned subscriber callback")
  print(NAVPOSE_CURRENT_POSITION_NED_TOPIC)
  rospy.Subscriber(NAVPOSE_CURRENT_POSITION_NED_TOPIC, Float64MultiArray, get_position_ned_callback)
  while current_position_ned_meters is None:
    print("Waiting for current position ned to set")
    time.sleep(0.5)
  ## Start Update Position ENU Callback
  print("Starting orientation enu subscriber callback")
  print(NAVPOSE_CURRENT_POSITION_ENU_TOPIC)
  rospy.Subscriber(NAVPOSE_CURRENT_POSITION_ENU_TOPIC, Float64MultiArray, get_position_enu_callback)
  while current_position_enu_meters is None:
    print("Waiting for current position enuto set")
    time.sleep(0.5)
  ## Start Update Global Location AMSL Callback
  print("Starting global location subscriber callback")
  print(NAVPOSE_CURRENT_LOCATION_AMSL_TOPIC)
  rospy.Subscriber(NAVPOSE_CURRENT_LOCATION_AMSL_TOPIC, Float64MultiArray, get_location_amsl_callback)
  while current_location_amsl_geo is None:
    print("Waiting for current location amsl height to set")
    time.sleep(0.5)
  ## Start Update Global Location WGS84 Callback
  print("Starting global location subscriber callback")
  print(NAVPOSE_CURRENT_LOCATION_WGS84_TOPIC)
  rospy.Subscriber(NAVPOSE_CURRENT_LOCATION_WGS84_TOPIC, Float64MultiArray, get_location_wgs84_callback)
  while current_location_wgs84_geo is None:
    print("Waiting for current location wsg84 to set")
    time.sleep(0.5)
  ## Start Update Geoid Height Callback
  print("Starting geoid height subscriber callback")
  print(NAVPOSE_CURRENT_GEOID_HEIGHT_TOPIC)
  rospy.Subscriber(NAVPOSE_CURRENT_GEOID_HEIGHT_TOPIC, Float64, get_geoid_height_callback)
  while current_geoid_height_m is None:
    print("Waiting for current geoid height to set")
    time.sleep(0.5)
  print("Completed Initialization")



### Function to set and check attitude setpoint command
def setpoint_attitude_ned_command(attitude_ned_degs, thrust, max_error_deg):
  global current_orientation_enu_degs
  global current_orientation_ned_degs
  print('')
  print("Starting Setpoint Attitude Create-Send-Check Process")
  ##############################################
  # Create Setpoint Message
  ##############################################
  print("Creating Message")
  print('')
  print("Attitude Input NED Degrees")
  print(" Roll, Pitch, Yaw")
  print(["%.2f" % attitude_ned_degs[0],"%.2f" % attitude_ned_degs[1],"%.2f" % attitude_ned_degs[2]])
  print('')
  print("Attitude Current NED Degrees")
  print(" Roll, Pitch, Yaw")
  print(["%.2f" % current_orientation_ned_degs[0],"%.2f" % current_orientation_ned_degs[1],"%.2f" % current_orientation_ned_degs[2]])
  # Capture current attitude and heading
  cur_attitude_ned_degs=list(current_orientation_ned_degs)
  # Set new attitude in degrees
  new_attitude_ned_degs=list(current_orientation_ned_degs) # Initialize with current
  for ind in range(3): # Overwrite current with new if set and valid
    if attitude_ned_degs[ind] != -999:
      new_attitude_ned_degs[ind]=attitude_ned_degs[ind]
  print('')
  print("Attitude Goal NED Degrees")
  print(" Roll, Pitch, Yaw")
  print(["%.2f" % new_attitude_ned_degs[0],"%.2f" % new_attitude_ned_degs[1],"%.2f" % new_attitude_ned_degs[2]])
  # Convert to ENU and create setpoint attitude goal in quaternion
  yaw_enu_deg = -attitude_ned_degs[2]-90
  if yaw_enu_deg < -180:
    yaw_enu_deg = 360 + yaw_enu_deg
  elif yaw_enu_deg > 180:
    yaw_enu_deg = yaw_enu_deg - 360
  attitude_sp_enu_degs = [attitude_ned_degs[0],attitude_ned_degs[1],yaw_enu_deg]
  print('')
  print("Attitude Goal ENU Degrees")
  print(" Roll, Pitch, Yaw")
  print(["%.2f" % attitude_sp_enu_degs[0],"%.2f" % attitude_sp_enu_degs[1],"%.2f" % attitude_sp_enu_degs[2]])
  attitude_sp_enu_quat = convert_rpy2quat(attitude_sp_enu_degs)
  new_orientation = Quaternion()
  new_orientation.x = attitude_sp_enu_quat[0]
  new_orientation.y = attitude_sp_enu_quat[1]
  new_orientation.z = attitude_sp_enu_quat[2]
  new_orientation.w = attitude_sp_enu_quat[3]
  # Set other setpoint attitude message values
  body_rate = Vector3()
  body_rate.x = 0
  body_rate.y = 0
  body_rate.z = 0
  type_mask = 1|2|4
  # Create Setpoint Attitude Message
  attitude_target_msg = AttitudeTarget()
  attitude_target_msg.orientation = new_orientation
  attitude_target_msg.body_rate = body_rate
  attitude_target_msg.type_mask = type_mask
  attitude_target_msg.thrust = thrust
  print('')
  print("Setpoint Attitude Message")
  print(attitude_target_msg)
  ##############################################
  ## Send Setpoint Message
  ##############################################
  print("Sending Message")
  setpoint_attitude_pub = rospy.Publisher(MAVROS_SETPOINT_ATTITUDE_TOPIC, AttitudeTarget, queue_size=1)
  setpoint_attitude_pub.publish(attitude_target_msg)
  ##############################################
  ## Check for Setpoint Success
  ##############################################
  print("Checking for Setpoint Reached")
  setpoint_attitude_reached = False
  while setpoint_attitude_reached is False:  # Wait for attitude goal to be set
    # Calculate setpoint attitude errors
    errors_deg = np.array(new_attitude_ned_degs) - np.array(current_orientation_ned_degs)
    for ind, val in enumerate(attitude_ned_degs):
      if val == -999.0: #Ignore
        errors_deg[ind]=0.0
    # Print some information
    print('')
    print("Current Attitude Degrees")
    print(" Roll, Pitch, Yaw")
    print(["%.2f" % current_orientation_ned_degs[0],"%.2f" % current_orientation_ned_degs[1],"%.2f" % current_orientation_ned_degs[2]])
    print('')
    print("Current Goal Degrees")
    print(["%.2f" % new_attitude_ned_degs[0],"%.2f" % new_attitude_ned_degs[1],"%.2f" % new_attitude_ned_degs[2]])
    print('')
    print("Current Attitude Errors")
    print(["%.3f" % errors_deg[0],"%.3f" % errors_deg[1],"%.3f" % errors_deg[2]])
    # Check for setpoint attitude goal
    max_error = max(abs(errors_deg))
    if  max_error > max_error_deg:
      print("Waiting for Attitude Setpoint to complete")
      time.sleep(1)
    else:
      print("Attitude Setpoint Reached")
      print('')
      setpoint_attitude_reached = True



##### Function to set and check position local setpoint command
##def setpoint_position_local(point_m =[0,0,0], yaw_deg =-999, max_error_m =1.0, max_error_deg =2.0, yaw_is_heading =True):
##  global poisiton_current_geopoint
##  global current_orientation_ned_degs
##  global current_heading_deg
##  global mavros_fake_gps_land_pub
##  global setpoint_position_local_pub
##  print('')
##  print("Starting Setpoint Position Local Create-Send-Check Process")
##  ##############################################
##  # Create Setpoint Message
##  ##############################################
##  print("Creating Message")
##  # Capture current oreantation and heading
##  cur_attitude_ned_deg=current_orientation_ned_degs 
##  cur_heading_deg=current_heading_deg
##  # Set new position in meters
##  new_position=Position()
##  new_position.x = point_m[0]
##  new_position.y = point_m[1]
##  new_position.z = point_m[2]
##  point_sp_m = point_m # Initialize Value for Check
##  print('')
##  print("Postion Goal Degrees")
##  print(" X, Y, Z")
##  print(["%.2f" % new_position.x,"%.2f" % new_position.y,"%.2f" % new_position.z])
##  # Set new yaw orienation in degrees
##  new_attitude_ned_deg=cur_attitude_ned_deg # Initialize with current
##  if yaw_deg != -999: # Keep current if -999
##    if yaw_is_heading: # Convert global heading goal to yaw is set
##      new_attitude_ned_deg[2] = cur_heading_deg+cur_attitude_ned_deg[2]-yaw_deg
##  heading_sp_deg = new_attitude_ned_deg[2] # Initialize Value for Check
##  print('')
##  print("Orientation Goal Degrees")
##  print(" Roll, Pitch, Yaw")
##  print(["%.2f" % new_attitude_ned_deg[0],"%.2f" % new_attitude_ned_deg[1],"%.2f" % new_attitude_ned_deg[2]])
##  ## Set new yaw orienation in quaternion
##  new_attitude_quat = convert_rpy2quat(new_attitude_ned_deg)
##  new_orientation = Quaternion()
##  new_orientation.x = new_attitude_quat[0]
##  new_orientation.y = new_attitude_quat[1]
##  new_orientation.z = new_attitude_quat[2]
##  new_orientation.w = new_attitude_quat[3]
##  ## Create Setpoint Attitude Message
##  position_local_target_msg = Pose()
##  position_local_target_msg.position = new_position
##  position_local_target_msg.orientation = new_orientation
##  print('')
##  print("Setpoint Position Local Message")
##  print(position_local_target_msg)
##  ##############################################
##  ## Send Setpoint Message
##  ##############################################
##  print("Sending Message")
##  setpoint_position_local_pub = rospy.Publisher(MAVROS_SETPOINT_POSITION_LOCAL_TOPIC, PoseStamped, queue_size=1)
##  setpoint_positon_local_pub.publish(position_local_target_msg)
##  if MAVROS_FAKE_GPS_SIM_SUPPORT:
##  ##############################################
##  ## Check for Setpoint Success
##  ##############################################
##  print("Checking for Setpoint Reached")
##  setpoint_attitude_reached = False
##  while setpoint_attitude_reached is False:  # Wait for attitude goal to be set
##    # Calculate setpoint attitude errors
##    pose = odometry_msg.pose.pose.orientation
##    xyzw_pose=(pose.x,pose.y,pose.z,pose.w)
##    attitude_ned_deg = convert_quat2rpy(xyzw_pose)
##    errors_deg = np.array(attitude_sp_deg) - np.array(attitude_ned_deg)
##    # Print some information
##    print('')
##    print("Current Attitude Degrees")
##    print(" Roll, Pitch, Yaw")
##    print(["%.2f" % attitude_ned_deg[0],"%.2f" % attitude_ned_deg[1],"%.2f" % attitude_ned_deg[2]])
##    print('')
##    print("Current Goal Degrees")
##    print(["%.2f" % attitude_sp_deg[0],"%.2f" % attitude_sp_deg[1],"%.2f" % attitude_sp_deg[2]])
##    print("Current Attitude Errors")
##    print(["%.3f" % errors_deg[0],"%.3f" % errors_deg[1],"%.3f" % errors_deg[2]])
##    # Check for setpoint attitude goal
##    max_error_deg = np.amax(np.abs(errors_deg))
##    if  max_error_deg > SETPOINT_ATTITUDE_MAX_ERROR_DEG:
##      print("Waiting for Attitude Setpoint to complete")
##      time.sleep(1)
##    else:
##      print("Attitude Setpoint Reached")
##      setpoint_attitude_reached = True

##setpoint_position_global_pub = rospy.Publisher(MAVROS_SETPOINT_POSITION_GLOBAL_TOPIC, GeoPoseStamped, queue_size=1)





## Function for custom post setpoint operations
def pre_setpoint_custom_actions():
  ###########################
  # Start Your Custom Actions
  ###########################
  # Update Home Position
  set_home_position(HOME_POSITION)
  # Set Mode to Guided
  update_mode('GUIDED')
  # Arm System
  update_armed(True)
  # Send Takeoff Command
  takeoff(10)
  ###########################
  # Stop Your Custom Actions
  ###########################

## Function for sending set_home command
def set_home_position(home_pos):
  global current_home
  global current_location_amsl_geo
  global mavros_fake_gps_sethome_pub
  # Send mavlink set home command and message
  new_home = list(home_pos)
  for ind, val in enumerate(new_home):
    if new_home[ind] == -999.0: # Use current
      new_home[ind]=current_location_amsl_geo[ind]
  #new_home[2] = new_home[2] - HOME_GEOID_HEIGHT # Change from WGS-84 to AMSL
  print('Sending mavlink set home command')
  print(new_home)
  set_home_client = rospy.ServiceProxy(MAVROS_SET_HOME_SERVICE, CommandHome)
  set_home_client(latitude=new_home[0],longitude=new_home[1],altitude=new_home[2])
  current_home = list(new_home)
  # Update fake gps home position if enabled
  if MAVROS_FAKE_GPS_SIM_SUPPORT:
    home_geopoint=GeoPoint()
    home_geopoint.latitude=home_pos[0]
    home_geopoint.longitude=home_pos[1]
    home_geopoint.altitude=home_pos[2]
    print('Sending fake gps set home command')
    print(home_geopoint)
    mavros_fake_gps_sethome_pub.publish(home_geopoint)
  # Wait for updates to set
  print("Waiting for set home poistion process to complete")
  time.sleep(1)

## Function for sending set_home command
def set_current_home_gps():
  global mavros_fake_gps_sethome_pub
  global current_location_amsl_geopoint
  print("Sending Set Home Current GPS Command")
  set_home_client = rospy.ServiceProxy(MAVROS_SET_HOME_SERVICE, CommandHome)
  takeoff_client(current_gps=True)
  if MAVROS_FAKE_GPS_SIM_SUPPORT:
    mavros_fake_gps_sethome_pub.publish(current_location_amsl_geopoint)
  print("Waiting for takeoff process to complete")
  time.sleep(1)


### Function to set mode
def update_mode(mode_new):
  global current_state
  global mode_client
  print('gothere')
  new_mode = SetModeRequest()
  new_mode.custom_mode = mode_new
  print("Updating mode")
  print(mode_new)
  mode_client = rospy.ServiceProxy(MAVROS_SET_MODE_SERVICE, SetMode)
  while current_state.mode != mode_new:
    mode_client.call(new_mode)
    print("Waiting for mode to set")
    time.sleep(.1)

### Function to set armed state
def update_armed(armed_new):
  global current_state
  global arming_client
  arm_cmd = CommandBoolRequest()
  arm_cmd.value = armed_new
  print("Updating armed")
  print(armed_new)
  arming_client = rospy.ServiceProxy(MAVROS_ARMING_SERVICE, CommandBool)
  while current_state.armed != armed_new:
    arming_client.call(arm_cmd)
    print("Waiting for armed value to set")
    time.sleep(.1)

## Function for sending takeoff command
def takeoff(takeoff_alt_m):
  global mavros_fake_gps_takeoff_pub
  global current_location_amsl_geo
  cur_alt_goal_m = current_location_amsl_geo[2]+takeoff_alt_m
  print("Sending Takeoff Command")
  takeoff_client = rospy.ServiceProxy(MAVROS_TAKEOFF_SERVICE, CommandTOL)
  takeoff_client(altitude=takeoff_alt_m)
  if MAVROS_FAKE_GPS_SIM_SUPPORT:
    print("Waiting for fake gps takeoff process to complete")
    mavros_fake_gps_takeoff_pub.publish(data=takeoff_alt_m)
    time.sleep(MAVROS_FAKE_GPS_UPDATE_TIME_SEC)
  tkoff_alt_error = abs(takeoff_alt_m)
  while tkoff_alt_error > MODE_UPDATE_MAX_POSITION_ERROR_M:
    print("Waiting for takeoff process to complete")
    tkoff_alt_error = abs(cur_alt_goal_m-current_location_amsl_geo[2])
    time.sleep(1)


## Function for custom setpoint operations
def at_setpoint_custom_actions():
  ###########################
  # Start Your Custom Actions
  ###########################
  ## Change Vehicle Mode to Guided
  print("Sending snapshot event trigger")
  snapshot(5)
  ###########################
  # Stop Your Custom Actions
  ###########################

### Function to send snapshot event trigger and wait for completion
def snapshot(wait_sec):
  global snapshot_trigger_pub
  snapshot_trigger_pub.publish(Empty())
  time.sleep(wait_sec)


  
## Function for custom post setpoint operations
def post_setpoint_custom_actions():
  ###########################
  # Start Your Custom Actions
  ###########################
  land() # Uncomment to change to Land mode
  #loiter() # Uncomment to change to Loiter mode
  #home() # Uncomment to change to Home mode
  #continue_mission() # Uncomment to return to pre setpoint state
  time.sleep(1)
  ###########################
  # Stop Your Custom Actions
  ###########################
  # Shutdown
  rospy.signal_shutdown("Setpoint Reached")



### Function for switching to LAND mode
def land():
  global current_state
  global mavros_fake_gps_land_pub
  update_mode('LAND')
  if MAVROS_FAKE_GPS_SIM_SUPPORT:
    print("Waiting for fake gps land process to complete")
    mavros_fake_gps_land_pub.publish(Empty())
    time.sleep(MAVROS_FAKE_GPS_UPDATE_TIME_SEC)
  print("Waiting for drone to land and disarm")
  while current_state.armed == True:
    time.sleep(1)

### Function for switching to LOITER mode
def loiter():
  update_mode('LOITER')

### Function for switching to HOME mode
def home():
  global current_state
  global current_home
  global current_location_amsl_geo
  global mavros_fake_gps_gohome_pub
  cur_alt_goal_m = list(current_location_amsl_geo)
  update_mode('HOME')
  if MAVROS_FAKE_GPS_SIM_SUPPORT:
    print("Waiting for fake gps home process to complete")
    mavros_fake_gps_gohome_pub.publish(Empty())
    time.sleep(MAVROS_FAKE_GPS_UPDATE_TIME_SEC)
  print("Waiting for drone to reach home")
  home_error = distance_geopoints(current_location_amsl_geo,current_home)
  while home_error > MODE_UPDATE_MAX_POSITION_ERROR_M:
    print("Waiting for home process to complete")
    home_error = distance_geopoints(current_location_amsl_geo,current_home)
    time.sleep(1)

### Function for switching back to current mission
def continue_mission():
  global original_state
  mode_org = original_state.mode.upper()
  update_mode(mode_org)

### Callback to get current state
def get_state_callback(state_msg):
  global current_state
  current_state = state_msg

### Callback to get current heading
def get_heading_callback(heading_msg):
  global current_heading_deg
  current_heading_deg = heading_msg.data

### Callback to get current NED orienation
def get_orientation_ned_callback(orientation_ned_msg):
  global current_orientation_ned_degs
  current_orientation_ned_degs = orientation_ned_msg.data

### Callback to get current ENU orienation
def get_orientation_enu_callback(orientation_enu_msg):
  global current_orientation_enu_degs
  current_orientation_enu_degs = orientation_enu_msg.data

### Callback to get current NED position
def get_position_ned_callback(position_ned_msg):
  global current_position_ned_meters
  current_position_ned_meters = position_ned_msg.data

### Callback to get current ENU position
def get_position_enu_callback(position_enu_msg):
  global current_position_enu_meters
  current_position_enu_meters = position_enu_msg.data

### Callback to get current location amsl
def get_location_amsl_callback(location_amsl_msg):
  global current_location_amsl_geo
  current_location_amsl_geo = location_amsl_msg.data

### Callback to get current location wgs84
def get_location_wgs84_callback(location_wgs84_msg):
  global current_location_wgs84_geo
  current_location_wgs84_geo = location_wgs84_msg.data

  ### Callback to get current geoid height
def get_geoid_height_callback(geoid_height_msg):
  global current_geoid_height_m
  current_geoid_height_m = geoid_height_msg.data

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

### Function to Estimate Distance Between Two GEO Locations
def distance_geopoints(geopoint1,geopoint2):
  lon1 = radians(geopoint1[0])
  lon2 = radians(geopoint2[0])
  lat1 = radians(geopoint1[1])
  lat2 = radians(geopoint2[0])
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


### Cleanup processes on node shutdown
def cleanup_actions():
  time.sleep(.1)

  
### Script Entrypoint
def startNode():
  rospy.loginfo("Starting MAVROS Set Attitude Target automation script")
  rospy.init_node("mavros_set_attitude_target_auto_script")
  #initialize system including pan scan process
  initialize_actions()
  #########################################
  # Execute custom pre-setpoint actions
  print("Starting Pre-Setpoint Actions")
  pre_setpoint_custom_actions()
  #########################################
  # Send Setpoint Attitude Command
  sp_att_ned_degs = SETPOINT_ATTITUDE_NED_DEGS
  sp_att_thrust = SETPOINT_ATTITUDE_THRUST
  sp_att_max_error_deg = SETPOINT_ATTITUDE_MAX_ERROR_DEG
  print("Starting Setpoint Attitude Call")
  setpoint_attitude_ned_command(sp_att_ned_degs, sp_att_thrust, sp_att_max_error_deg)
  #########################################
  # Execute custom setpoint actions
  print("Starting Setpoint Actions")
  #at_setpoint_custom_actions()
  #########################################
  # Send Setpoint Position Local Command
  #print("Starting Setpoint Attitude Call")
  #setpoint_attitude(sp_att_attitude_ned_deg,sp_att_thrust,sp_att_max_error_deg,sp_att_yaw_is_heading)
  #########################################
  # Execute custom setpoint actions
  print("Starting Setpoint Actions")
  at_setpoint_custom_actions()
  #########################################
  ####### End Setpoint Attitude Send and Monitor Processes
  # Run Custom Post-Setpoint Actions
  print("Starting Post-Setpoint Actions")
  post_setpoint_custom_actions()
  # Run cleanup actions on rospy shutdown
  rospy.on_shutdown(cleanup_actions)
  # Spin forever
  rospy.spin()


#####################################################################################
# Main
#####################################################################################

if __name__ == '__main__':
  startNode()

