#!/usr/bin/env python

__author__ = "Jason Seawall"
__copyright__ = "Copyright 2023, Numurus LLC"
__email__ = "nepi@numurus.com"
__credits__ = ["Jason Seawall", "Josh Maximoff"]

__license__ = "GPL"
__version__ = "2.0.4.0"

# Sample NEPI RBX Driver Script.

import rospy
import time
import numpy as np
import math
import tf
import pygeodesy


from std_msgs.msg import Empty, Int8, UInt8, Bool, String, Float32, Float64, Float64MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, BatteryState
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped
from geographic_msgs.msg import GeoPoint, GeoPose, GeoPoseStamped
from mavros_msgs.msg import State, AttitudeTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL, CommandHome
from nepi_ros_interfaces.srv import NavPoseQuery, NavPoseQueryRequest
from pygeodesy.ellipsoidalKarney import LatLon

try:
  print(['Loading Geoids Database from: ' + GEOID_DATABASE_FILE])
  ginterpolator = pygeodesy.GeoidKarney(GEOID_DATABASE_FILE)
  USE_FALLBACK_GEOID_HEIGHT = False
except:
  print('Geoids database failed to import')
  def ginterpolator(single_position):
    return 0


#####################################################################################
# SETUP - Edit as Necessary ##################################
##########################################

###################################################
# RBX State and Mode Dictionaries
RBX_STATES = ["DISARM","ARM"]
RBX_MODES = ["STABILIZE","TAKEOFF","LAND","RTL","LOITER","GUIDED"] 

###################################################
# RBX Initialization Values
GOTO_TRAN_SPEED_RATIO = 0.5
GOTO_ROT_SPEED_RATIO = 0.5
GOTO_MAX_ERROR_M = 2.0 # Goal reached when all translation move errors are less than this value
GOTO_MAX_ERROR_DEG = 2.0 # Goal reached when all rotation move errors are less than this value
GOTO_STABILIZED_SEC = 1.0 # Window of time that setpoint error values must be good before proceeding

###################################################
# ROS namespace setup
NEPI_BASE_NAMESPACE = "/nepi/s2x/"
NEPI_NAVPOSE_SERVICE_NAME = NEPI_BASE_NAMESPACE + "nav_pose_query"
NEPI_RBX_NAMESPACE = NEPI_BASE_NAMESPACE + "mavlink/rbx/"


# NEPI MAVLINK RBX Driver Capabilities Publish Topics
NEPI_RBX_CAPABILITIES_RATE_HZ = 1
NEPI_RBX_CAPABILITIES_CONTROLS_TOPIC = NEPI_RBX_NAMESPACE + "control_support"
NEPI_RBX_CAPABILITIES_FEEDBACK_TOPIC = NEPI_RBX_NAMESPACE + "feedack_support"
NEPI_RBX_CAPABILITIES_STATES_TOPIC = NEPI_RBX_NAMESPACE + "state_options"
NEPI_RBX_CAPABILITIES_MODES_TOPIC = NEPI_RBX_NAMESPACE + "mode_options"

# NEPI MAVLINK RBX Driver Status Publish Topics
NEPI_RBX_STATUS_RATE_HZ = 10
NEPI_RBX_STATUS_STATE_TOPIC = NEPI_RBX_NAMESPACE + "state"  # Int to Defined Dictionary RBX_STATES
NEPI_RBX_STATUS_MODE_TOPIC = NEPI_RBX_NAMESPACE + "mode" # Int to Defined Dictionary RBX_MODES
NEPI_RBX_STATUS_BATTERY_TOPIC = NEPI_RBX_NAMESPACE + "battery" # Float Ratio 0-1
NEPI_RBX_STATUS_GOTO_STATUS_TOPIC = NEPI_RBX_NAMESPACE + "goto_status" # Bool, True if goto is complete or no active goto process
NEPI_RBX_STATUS_GOTO_SPEEDS_TOPIC = NEPI_RBX_NAMESPACE + "goto_speeds" # Floats [Translation_Ratio,Rotation_Ratio]
NEPI_RBX_STATUS_GOTO_GOALS_TOPIC = NEPI_RBX_NAMESPACE + "goto_goals" # Floats [Max_Meters,Max_Degrees,Stabilize_Time_Sec]
NEPI_RBX_STATUS_GOTO_ERRORS_TOPIC = NEPI_RBX_NAMESPACE + "goto_errors" # Floats [X_Meters,Y_Meters,Z_Meters,Heading_Degrees,Roll_Degrees,Pitch_Degrees,Yaw_Degrees]

# NEPI MAVLINK RBX Driver NavPose Publish Topics
NEPI_RBX_NAVPOSE_GPS_TOPIC = NEPI_RBX_NAMESPACE + "gps_fix"
NEPI_RBX_NAVPOSE_ODOM_TOPIC = NEPI_RBX_NAMESPACE + "odom"
NEPI_RBX_NAVPOSE_HEADING_TOPIC = NEPI_RBX_NAMESPACE + "heading"

# NEPI MAVLINK RBX Driver Settings Subscriber Topics
NEPI_RBX_SET_STATE_TOPIC = NEPI_RBX_NAMESPACE + "set_state" # Int to Defined Dictionary RBX_STATES
NEPI_RBX_SET_MODE_TOPIC = NEPI_RBX_NAMESPACE + "set_mode"  # Int to Defined Dictionary RBX_MODES
NEPI_RBX_SET_HOME_CURRENT_TOPIC = NEPI_RBX_NAMESPACE + "set_home_current" # Emplty
NEPI_RBX_SET_GOTO_SPEEDS_TOPIC = NEPI_RBX_NAMESPACE + "set_goto_speeds" # Float [Translation_Ratio,Rotation_Ratio]
NEPI_RBX_SET_GOTO_GOALS_TOPIC = NEPI_RBX_NAMESPACEE + "set_goto_goals" # Float [Max_Meters,Max_Degrees,Stabilize_Time_Sec]

# NEPI MAVLINK RBX Driver Control Subscriber Topics
NEPI_RBX_GOTO_POSE_TOPIC = NEPI_RBX_NAMESPACE + "goto_pose" # Ignored if any active goto processes
NEPI_RBX_GOTO_POSITION_TOPIC = NEPI_RBX_NAMESPACE + "goto_position" # Ignored if any active goto processes
NEPI_RBX_GOTO_LOCATION_TOPIC = NEPI_RBX_NAMESPACE + "goto_location" # Ignored if any active goto processes
NEPI_RBX_GO_HOME_TOPIC = NEPI_RBX_NAMESPACE + "go_home" # Aborts any active goto processes
NEPI_RBX_HOLD_STATION_TOPIC = NEPI_RBX_NAMESPACE + "go_hold" # Aborts any active goto processes

###################################################
MAVLINK_NAMESPACE = NEPI_BASE_NAMESPACE + "mavlink/"

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


#####################################################################################
# Globals
#####################################################################################
rbx_capabilities_states_pub = rospy.Publisher(NEPI_RBX_CAPABILITIES_STATES_TOPIC, String, queue_size=1)
rbx_capabilities_mode_pub = rospy.Publisher(NEPI_RBX_CAPABILITIES_MODES_TOPIC, String, queue_size=1)

rbx_status_state_pub = rospy.Publisher(NEPI_RBX_STATUS_STATE_TOPIC, Int8, queue_size=1)
rbx_status_mode_pub = rospy.Publisher(NEPI_RBX_STATUS_MODE_TOPIC, Int8, queue_size=1)
rbx_status_battery_pub = rospy.Publisher(NEPI_RBX_STATUS_BATTERY_TOPIC, UInt8, queue_size=1)
rbx_status_goto_status_pub = rospy.Publisher(NEPI_RBX_STATUS_GOTO_STATUS_TOPIC, Bool, queue_size=1)
rbx_status_goto_speeds_pub = rospy.Publisher(NEPI_RBX_STATUS_GOTO_SPEEDS_TOPIC, Float64MultiArray, queue_size=1)
rbx_status_goto_goals_pub = rospy.Publisher(NEPI_RBX_STATUS_GOTO_GOALS_TOPIC, Float64MultiArray, queue_size=1)
rbx_status_goto_errors_pub = rospy.Publisher(NEPI_RBX_STATUS_GOTO_ERRORS_TOPIC, Float64MultiArray, queue_size=1)

rbx_navpose_gps_pub = rospy.Publisher(NEPI_RBX_NAVPOSE_GPS_TOPIC, NavSatFix, queue_size=1)
rbx_navpose_odom_pub = rospy.Publisher(NEPI_RBX_NAVPOSE_ODOM_TOPIC, Odometry, queue_size=1)
rbx_navpose_heading_pub = rospy.Publisher(NEPI_RBX_NAVPOSE_HEADING_TOPIC, Float64, queue_size=1)

current_heading_deg = None
current_orientation_enu_degs = None
current_orientation_ned_degs = None
current_position_enu_m = None
current_position_ned_m = None
current_location_amsl_geo = None
current_location_wgs84_geo = None
current_geoid_height_m = None

current_rbx_state = None
current_rbx_mode = None
current_rbx_battery = 0
current_rbx_goto_status = True
current_rbx_goto_speeds = [GOTO_TRAN_SPEED_RATIO,GOTO_ROT_SPEED_RATIO]
current_rbx_goto_goals = [GOTO_MAX_ERROR_M,GOTO_MAX_ERROR_DEG,GOTO_STABILIZED_SEC]
current_rbx_goto_errors = [0,0,0,0,0,0]

orig_rbx_state = None
orig_rbx_mode = None

rbx_capabilities_pub_interval = float(1.0)/float(NEPI_RBX_CAPABILITIES_RATE_HZ)
rbx_status_pub_interval = float(1.0)/float(NEPI_RBX_STATUS_RATE_HZ)
update_navpose_interval = 0.1 # 10 Hz

rbx_state_functions = ["disarm","arm"]
rbx_mode_functions = ["stabilize","takeoff","land","rtl","loiter","guided"]

                
#####################################################################################
# Methods
#####################################################################################

### System Initialization processes
def initialize_actions():
  global current_rbx_state
  global current_rbx_mode
  global current_rbx_battery
  global orig_rbx_state
  global orig_rbx_mode
  print("Starting Initialization Process")
  # Wait for MAVLink State topic to publish then subscribe
  print("Waiting for topic: " + MAVLINK_STATE_TOPIC)
  wait_for_topic(MAVLINK_STATE_TOPIC, 'mavros_msgs/State')
  print("Starting state scubscriber callback")
  rospy.Subscriber(MAVLINK_STATE_TOPIC, State, get_state_callback)
  while current_rbx_state is None and not rospy.is_shutdown():
    print("Waiting for current state message to publish")
    time.sleep(0.1)
  orig_rbx_state = current_rbx_state
  orig_rbx_mode = current_rbx_mode
  # Subscribe to MAVLink Battery topic
  rospy.Subscriber(MAVLINK_BATTERY_TOPIC, BatteryState, get_battery_callback)
  # Start NavPose Data Updater
  rospy.Timer(rospy.Duration(update_navpose_interval), update_current_navpose_callback)
  ##############################
  print("Starting RBX driver publisher and subscriber topics")
  # Start RBX Capabilities and Status Publishers
  rospy.Timer(rospy.Duration(rbx_capabilities_pub_interval), rbx_capabilities_pub_callback)
  rospy.Timer(rospy.Duration(rbx_status_pub_interval), rbx_status_pub_callback)
  ### Start RBX NavPose Publishers
  rospy.Subscriber(MAVLINK_SOURCE_GPS_TOPIC, NavSatFix, rbx_gps_topic_callback)
  rospy.Subscriber(MAVLINK_SOURCE_ODOM_TOPIC, Odometry, rbx_odom_topic_callback)
  rospy.Subscriber(MAVLINK_SOURCE_HEADING_TOPIC, Float64, rbx_heading_topic_callback)
  ### Start RBX Settings Subscribe Topics
  rospy.Subscriber(NEPI_RBX_SET_STATE_TOPIC, Int8, rbx_set_state_callback)
  rospy.Subscriber(NEPI_RBX_SET_MODE_TOPIC, Int8, rbx_set_mode_callback)
  rospy.Subscriber(NEPI_RBX_SET_HOME_CURRENT_TOPIC, Empty, rbx_set_home_current_callback)
  rospy.Subscriber(NEPI_RBX_SET_GOTO_SPEEDS_TOPIC, Float64MultiArray, rbx_set_goto_speeds_callback)
  rospy.Subscriber(NEPI_RBX_SET_GOTO_GOALS_TOPIC, Float64MultiArray, rbx_set_goto_goals_callback)
  ### Start RBX Control Subscribe Topics

  rospy.Subscriber(NEPI_RBX_GOTO_POSE_TOPIC, Float64MultiArray, rbx_goto_pose_callback)
  rospy.Subscriber(NEPI_RBX_GOTO_POSITION_TOPIC, Float64MultiArray, rbx_goto_position_callback)
  rospy.Subscriber(NEPI_RBX_GOTO_LOCATION_TOPIC, Float64MultiArray, rbx_goto_location_callback)
  rospy.Subscriber(NEPI_RBX_GO_HOME_TOPIC, Empty, rbx_go_home_callback)
  rospy.Subscriber(NEPI_RBX_HOLD_STATION_TOPIC, Empty, rbx_hold_station_callback)
  print("Initialization Complete")


##############################
# RBX Capabilities Topic Publishers
### Callback to publish RBX capabilite option lists
def rbx_capabilities_pub_callback(timer):
  global rbx_capabilities_states_pub
  global rbx_capabilities_mode_pub
  if not rospy.is_shutdown():
    rbx_capabilities_states_pub.publish(str(RBX_STATES))
    rbx_capabilities_mode_pub.publish(str(RBX_MODES))
  

##############################
# RBX Status Topic Publishers
### Callback for rbx status publisher
def rbx_status_pub_callback(timer):
  global current_rbx_state
  global current_rbx_mode
  global current_rbx_battery
  global current_rbx_goto_status
  global current_rbx_goto_speeds
  global current_rbx_goto_goals
  global current_rbx_goto_errors
  global rbx_status_state_pub
  global rbx_status_mode_pub
  global rbx_status_battery_pub
  global rbx_status_goto_status_pub
  global rbx_status_goto_speeds_pub
  global rbx_status_goto_goals_pub
  global rbx_status_goto_errors_pub
  if not rospy.is_shutdown():
    rbx_status_state_pub.publish(current_rbx_state)
    rbx_status_mode_pub.publish(current_rbx_mode)
    rbx_status_battery_pub.publish(current_rbx_battery)
    rbx_status_goto_status_pub.publish(current_rbx_goto_status)
    rbx_status_goto_speeds_pub.publish(data=current_rbx_goto_speeds)
    rbx_status_goto_goals_pub.publish(data=current_rbx_goto_goals)
    rbx_status_goto_errors_pub.publish(data=current_rbx_goto_errors)


##############################
# RBX NavPose Topic Publishers
### Callback to publish RBX navpose gps topic
def rbx_gps_topic_callback(navsatfix_msg):
  global rbx_navpose_gps_pub
  if not rospy.is_shutdown():
    rbx_navpose_gps_pub.publish(navsatfix_msg)
  
### Callback to publish RBX odom topic
def rbx_odom_topic_callback(odom_msg):
  global rbx_navpose_odom_pub
  if not rospy.is_shutdown():
    rbx_navpose_odom_pub.publish(odom_msg)

### Callback to publish RBX heading topic
def rbx_heading_topic_callback(heading_msg):
  global rbx_heading_odom_pub
  if not rospy.is_shutdown():
    rbx_navpose_heading_pub.publish(heading_msg)

##############################
# RBX Settigs Topic Callbacks
# ToDo: Create a custom RBX status message
### Callback to set state
def rbx_set_state_callback(state_msg):
  global current_rbx_state
  state_ind = state_msg
  if state_ind < 0 or state_ind > (len(rbx_state_functions)-1):
    print("No matching rbx state found")
  else:
    set_state_function = locals()[rbx_state_functions[state_ind]]
    set_state_function()
    current_rbx_state = state_ind

### Callback to set mode
def rbx_set_mode_callback(mode_msg):
  global current_rbx_mode
  mode_ind = mode_msg
  if mode_ind < 0 or mode_ind > (len(rbx_mode_functions)-1):
    print("No matching rbx mode found")
  else:
    set_mode_function = locals()[rbx_mode_functions[mode_ind]]
    set_mode_function() 
    current_rbx_mode = mode_ind
    
### Callback to set mode
def rbx_set_home_current_callback(set_home_msg):
  sethome_current()

  
### Callback to start rbx set goto speeds process
def rbx_set_goto_speeds_callback(goto_speeds_msg):
  global current_rbx_goto_speeds
  goto_speeds_list=list(goto_speeds_msg.data)
  if len(goto_speeds_list) != 2:
    print("Ignoring this Request")
    print("Messge is wrong length. Should be float list of size 2")
  else:    
    current_rbx_goto_speeds = goto_speeds_list

### Callback to start rbx set goto goals process
def rbx_set_goto_goals_callback(goto_goals_msg):
  global current_rbx_goto_goals
  goto_goals_list=list(goto_goals_msg.data)
  if len(goto_goals_list) != 3:
    print("Ignoring this Request")
    print("Messge is wrong length. Should be float list of size 3")
  else:
    current_rbx_goto_goals = goto_goals_list
  

##############################
# RBX Control Topic Callbacks
### Callback to start MAVLINK goto pose process
def rbx_goto_pose_callback(pose_cmd_msg):
  global current_rbx_goto_status
  print("*******************************")
  print("Recieved GoTo Pose Message")
  print("")
  print(pose_cmd_msg)
  setpoint_data=list(pose_cmd_msg.data)
  if len(setpoint_data) != 3:
    print("Ignoring this Request")
    print("Messge is wrong length. Should be float list of size 3")
    print("[Roll,Pitch,Yaw]")
  else:
    if current_rbx_goto_status is False:
      print("Another GoTo Command Process is Active")
      print("Ignoring this Request")
    else:
      setpoint_attitude_ned(setpoint_data)


### Callback to start rbx goto position process
def rbx_goto_position_callback(position_cmd_msg):
  global current_rbx_goto_status
  print("*******************************")
  print("Recieved GoTo Position Command Message")
  print("")
  print(position_cmd_msg)
  setpoint_data=list(position_cmd_msg.data)
  if len(setpoint_data) != 4:
    print("Ignoring this Request")
    print("Messge is wrong length. Should be float list of size 4")
    print("[X,Y,Z,Yaw]")
  else:    
    if current_rbx_goto_status is False:
      print("Another GoTo Command Process is Active")
      print("Ignoring this Request")
    else:
      setpoint_position_local_body(setpoint_data)


### Callback to start rbx goto location subscriber
def rbx_goto_location_callback(location_cmd_msg):
  global current_rbx_goto_status
  print("*******************************")
  print("Recieved GoTo Location Message")
  print("")
  print(location_cmd_msg)
  setpoint_data=list(location_cmd_msg.data)
  if len(setpoint_data) != 4:
    print("Ignoring this Request")
    print("Messge is wrong length. Should be float list of size 4")
    print("[Lat,Long,Alt,Yaw]")
  else:
    if current_rbx_goto_status is False:
      print("Another GoTo Command Process is Active")
      print("Ignoring this Request")
    else:
      setpoint_location_global_wgs84(setpoint_data)

### Callback to start rbx go home
def rbx_go_home_callback(home_msg):
  rtl()

### Callback to start rbx hold station
def rbx_hold_station_callback(hold_msg):
  loiter() 

#######################
# Mavlink Interface Methods

### Callback to get current state message
def get_state_callback(state_msg):
  global current_rbx_state
  global current_rbx_mode
  # Update rbx state value
  arm_val = state_msg.armed
  if arm_val == True:
    current_rbx_state=1
  else:
    current_rbx_state=0
  # Update rbx mode value
  mode_val = state_msg.mode
  mode_ind=-1
  for ind, mode in enumerate(RBX_MODES):
    if mode == mode_val:
      mode_ind=ind
  current_rbx_mode=mode_ind   


### Callback to get current battery message
def get_battery_callback(battery_msg):
  global current_rbx_battery
  current_rbx_battery = battery_msg.percentage


### Function to set mode
def update_mode(mode_new):
  global current_state
  global mode_client
  new_mode = SetModeRequest()
  new_mode.custom_mode = mode_new
  print("Updating mode")
  print(mode_new)
  mode_client = rospy.ServiceProxy(MAVLINK_SET_MODE_SERVICE, SetMode)
  while current_state.mode != mode_new and not rospy.is_shutdown():
    time.sleep(.25)
    mode_client.call(new_mode)
    print("Waiting for mode to set")
    print("Set Value: " + mode_new)
    print("Cur Value: " + str(current_state.mode))


### Function to set armed state
def set_arm_state(arm_value):
  global current_state
  global arming_client
  arm_cmd = CommandBoolRequest()
  arm_cmd.value = arm_value
  print("Updating armed")
  print(arm_value)
  arming_client = rospy.ServiceProxy(MAVLINK_ARMING_SERVICE, CommandBool)
  while current_state.armed != arm_value and not rospy.is_shutdown():
    time.sleep(.25)
    arming_client.call(arm_cmd)
    print("Waiting for armed value to set")
    print("Set Value: " + str(arm_value))
    print("Cur Value: " + str(current_state.armed))



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
  global current_rbx_goto_status
  global current_rbx_goto_goals
  global current_rbx_goto_errors
  current_rbx_goto_status = False
  current_rbx_goto_errors = [0,0,0,0,0,0]
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
  setpoint_attitude_pub = rospy.Publisher(MAVLINK_SETPOINT_ATTITUDE_TOPIC, AttitudeTarget, queue_size=1)
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
      if time_sec > current_rbx_goto_goals[2]:
        print(time_sec)
        print(current_rbx_goto_goals[2])
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
        if max_attitude_errors < current_rbx_goto_goals[1]:
          print('')
          print("Attitude Setpoint Reached")
          setpoint_attitude_reached = True
      else:
        attitude_errors.append(max_attutude_error_deg) # append last
    # Reset print timer if past
    if time_sec > GOTO_STABILIZED_SEC:
      time_sec=0 # Reset print timer
    current_rbx_goto_errors = [0,0,0] + attitude_errors_degs
  print("************************")
  print("Setpoint Attitude Reached")
  current_rbx_goto_status = True
  


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
  global current_rbx_goto_status
  global current_rbx_goto_goals
  global current_rbx_goto_errors
  current_rbx_goto_status = False
  current_rbx_goto_errors = [0,0,0,0,0,0]
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
  ## Send Message and Check for Setpoint Success
  ##############################################
  setpoint_position_local_pub = rospy.Publisher(MAVLINK_SETPOINT_POSITION_LOCAL_TOPIC, PoseStamped, queue_size=1)
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
      if time_sec > GOTO_STABILIZED_SEC:
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
        if max_point_errors < current_rbx_goto_goals[0]:
          print('')
          print("Position Setpoint Reached")
          setpoint_position_local_point_reached = True
      else:
        point_errors.append(max_point_ned_error_m) # append last
    # Check for setpoint position yaw point goal
    if  setpoint_position_local_yaw_reached is False:
      if time_sec > current_rbx_goto_goals[2]:
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
        if max_yaw_errors < Gcurrent_rbx_goto_goals[1]:
          print('')
          print("Yaw Setpoint Reached")
          setpoint_position_local_yaw_reached = True
      else:
        yaw_errors.append(max_yaw_ned_error_deg) # append last
    # Reset print timer if past
    if time_sec > current_rbx_goto_goals[2]:
      time_sec=0 # Reset print timer
    current_rbx_goto_errors = geopoint_errors_m + [0,0] + max_yaw_ned_error_deg
  print("************************")
  print("Setpoint Reached")
  current_rbx_goto_status = True



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
  global current_rbx_goto_status
  global current_rbx_goto_goals
  global current_rbx_goto_errors
  current_rbx_goto_status = False
  current_rbx_goto_errors = [0,0,0,0,0,0]
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
  ## Send Message and Check for Setpoint Success
  ##############################################
  setpoint_location_global_pub = rospy.Publisher(MAVLINK_SETPOINT_LOCATION_GLOBAL_TOPIC, GeoPoseStamped, queue_size=1)
  time.sleep(1) # VERY IMPORTANT - Sleep a bit between declaring a publisher and using it
  print("Sending MAVLINK Setpoint Position Local Command at 50 Hz and")
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
      if time_sec > current_rbx_goto_goals[3]:
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
        if max_geopoint_errors < current_rbx_goto_goals[0]:
          print('')
          print("Location Setpoint Reached")
          setpoint_location_global_geopoint_reached = True
      else:
        geopoint_errors.append(max_geopoint_error_m) # append last
    # Check for setpoint position yaw goal
    if  setpoint_location_global_yaw_reached is False:
      if time_sec > current_rbx_goto_goals[2]:
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
        if max_yaw_errors < current_rbx_goto_goals[1]:
          print('')
          print("Yaw Setpoint Reached")
          setpoint_location_global_yaw_reached = True
      else:
        yaw_errors.append(max_yaw_ned_error_deg) # append last
    # Reset print timer if past
    if time_sec > 1:
      time_sec=0 # Reset print timer
    current_rbx_goto_errors = geopoint_errors_m + [0,0] + max_yaw_ned_error_deg
  print("************************")
  print("Setpoint Reached")
  current_rbx_goto_status = True


#######################
# Mavlink Ardupilot Interface Methods
  
## Function for sending takeoff command
def takeoff():
  print("Sending Takeoff Command to Altitude")
  takeoff_client = rospy.ServiceProxy(MAVLINK_TAKEOFF_SERVICE, CommandTOL)
  time.sleep(.1) # VERY IMPORTANT - Sleep a bit between declaring a publisher and using it
  takeoff_client(min_pitch=10,altitude=10)
  print("Waiting for takeoff process to complete")
  time.sleep(15) # Wait for takeoff
    
### Function for switching to LAND mode
def land():
  global current_state
  global mavros_fake_gps_land_pub
  update_mode('LAND')
  if MAVLINK_FAKE_GPS_SIM_SUPPORT:
    print("Sending fake gps goto takeoff command")
    land_geopoint=[-999,-999,0]
    fake_gps_goto_geopoint(land_geopoint)
  print("Waiting for land process to complete")
  while current_state.armed == True:
    time.sleep(1)

### Function for sending go home command
def rtl():
  global current_home
  update_mode('RTL')
  # Set Fake GPS Start Location and Reset
  if current_home is not None:
    if MAVLINK_FAKE_GPS_SIM_SUPPORT:
      print("Updatihg Fake GPS to New Home Location")
      fake_gps_goto_geopoint(current_home)

### Function for switching to LOITER mode
def loiter():
  update_mode('LOITER')

### Function for switching back to current mission
def continue_mission():
  global original_state
  mode_org = original_state.mode.upper()
  update_mode(mode_org)

### Function for sending set home current
def sethome_current():
  global current_home
  print('Sending mavlink set home current command')
  set_home_client = rospy.ServiceProxy(MAVLINK_SET_HOME_SERVICE, CommandHome)
  time.sleep(.1) # VERY IMPORTANT - Sleep a bit between declaring a publisher and using it
  set_home_client(current_gps=True)



#######################
# NEPI NavPose Interface Methods

### Setup a regular background navpose get and update navpose data
def update_current_navpose_callback(timer):
  global current_heading_deg
  global current_orientation_enu_degs
  global current_orientation_ned_degs
  global current_position_enu_m
  global current_position_ned_m
  global current_location_amsl_geo
  global current_location_wgs84_geo
  global current_geoid_height_m
  # Get current NEPI NavPose data from NEPI ROS nav_pose_query service call
  try:
    get_navpose_service = rospy.ServiceProxy(NEPI_NAVPOSE_SERVICE_NAME, NavPoseQuery)
    nav_pose_response = get_navpose_service(NavPoseQueryRequest())
    #print(nav_pose_response)
    # Set current navpose
    current_navpose = nav_pose_response.nav_pose
    # Set current heading in degrees
    current_heading_deg = nav_pose_response.nav_pose.heading.heading
    # Set current orientation vector (roll, pitch, yaw) in degrees enu frame
    pose_enu_o = nav_pose_response.nav_pose.odom.pose.pose.orientation
    xyzw_enu_o = list([pose_enu_o.x,pose_enu_o.y,pose_enu_o.z,pose_enu_o.w])
    rpy_enu_d = convert_quat2rpy(xyzw_enu_o)
    current_orientation_enu_degs = Float64MultiArray()
    current_orientation_enu_degs.data = [rpy_enu_d[0],rpy_enu_d[1],rpy_enu_d[2]]
    # Set current orientation vector (roll, pitch, yaw) in degrees ned frame +-180
    pose_enu_o = nav_pose_response.nav_pose.odom.pose.pose.orientation
    xyzw_enu_o = list([pose_enu_o.x,pose_enu_o.y,pose_enu_o.z,pose_enu_o.w])
    rpy_enu_d = convert_quat2rpy(xyzw_enu_o)
    yaw_ned_d = convert_yaw_enu2ned(rpy_enu_d[2])
    rpy_ned_d = [rpy_enu_d[0],rpy_enu_d[1],yaw_ned_d]
    current_orientation_ned_degs = Float64MultiArray()
    current_orientation_ned_degs.data = [rpy_ned_d[0],rpy_ned_d[1],rpy_ned_d[2]]
    # Set current position vector (x, y, z) in meters enu frame
    pose_enu_p = nav_pose_response.nav_pose.odom.pose.pose.position
    current_position_enu_m = Float64MultiArray()
    current_position_enu_m.data = [pose_enu_p.x, pose_enu_p.y, pose_enu_p.z]
    # Set current position vector (x, y, z) in meters ned frame
    pose_enu_p = nav_pose_response.nav_pose.odom.pose.pose.position
    current_position_ned_m = Float64MultiArray()
    current_position_ned_m.data = [pose_enu_p.y, pose_enu_p.x, -pose_enu_p.z]
    # Set current location vector (lat, long, alt) in geopoint data with AMSL height
    fix_amsl = nav_pose_response.nav_pose.fix
    current_location_amsl_geo = Float64MultiArray()
    current_location_amsl_geo.data =  [fix_amsl.latitude,fix_amsl.longitude,fix_amsl.altitude]
    # Set current geoid height
    single_position=LatLon(fix_amsl.latitude,fix_amsl.longitude)
    geoid_height = ginterpolator(single_position)
    current_geoid_height =  geoid_height
    # Set current location vector (lat, long, alt) in geopoint data with WGS84 height
    fix_amsl = nav_pose_response.nav_pose.fix
    current_location_wgs84_geo = Float64MultiArray()
    current_location_wgs84_geo.data =  [fix_amsl.latitude,fix_amsl.longitude,(fix_amsl.altitude + geoid_height)]
  except Exception as e:
    print("navpose service call failed: " + str(e))

#######################
# Data Converstion Methods

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



#######################
# Node Process Methods

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
  rospy.loginfo("Starting Ardupilot RBX Driver Script")
  rospy.init_node("ardupilot_rbx_driver_script")
  #initialize system including pan scan process
  initialize_actions()
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

