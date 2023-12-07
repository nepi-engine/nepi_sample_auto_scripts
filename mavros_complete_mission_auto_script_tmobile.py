#!/usr/bin/env python

__author__ = "Jason Seawall"
__copyright__ = "Copyright 2023, Numurus LLC"
__email__ = "nepi@numurus.com"
__credits__ = ["Jason Seawall", "Josh Maximoff"]

__license__ = "GPL" 
__version__ = "2.0.4.0"


# Sample NEPI Automation Script.
# Uses onboard ROS python and mavros libraries to
# 1) Subscribes to NEPI nav_pose_current heading, orientation, position, location topics
# 2) Runs pre-mission processes
# 3) Runs mission setpoint command processes
# 4) Runs mission setpoint action processes
# 5) Runs post-mission processes

# Requires the following additional scripts are running
# a) navpose_get_and_publish_auto_script.py
# b) mavros_setpoint_controls_auto_script.py
# c) mavros_navpose_config_auto_script.py
# d) (Optional) MAVROS_fake_gps_sim_auto_script.py if a real GPS fix is not available
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
MAVROS_NAMESPACE = NEPI_BASE_NAMESPACE + "pixhawk_mavlink/"
MAVROS_CONTROLS_NAMESPACE = MAVROS_NAMESPACE + "controls/"

# Setpoint Action Topics
SNAPSHOT_TOPIC = NEPI_BASE_NAMESPACE + "snapshot_event"

# MAVROS Subscriber Topics
MAVROS_STATE_TOPIC = MAVROS_NAMESPACE + "state"

# MAVROS Required Services
MAVROS_SET_HOME_SERVICE = MAVROS_NAMESPACE + "cmd/set_home"
MAVROS_SET_MODE_SERVICE = MAVROS_NAMESPACE + "set_mode"
MAVROS_ARMING_SERVICE = MAVROS_NAMESPACE + "cmd/arming"
MAVROS_TAKEOFF_SERVICE = MAVROS_NAMESPACE + "cmd/takeoff"

# NEPI MAVROS Control Publisher Topics
MAVROS_CONTROL_ATTITUDE_TOPIC = MAVROS_CONTROLS_NAMESPACE + "setpoint_command_attitude"
MAVROS_CONTROL_POSITION_TOPIC = MAVROS_CONTROLS_NAMESPACE + "setpoint_command_position"
MAVROS_CONTROL_LOCATION_TOPIC = MAVROS_CONTROLS_NAMESPACE + "setpoint_command_location"
# NEPI MAVROS Control Subscriber Topics
MAVROS_CONTROL_PROCESS_COMPLETE_TOPIC = MAVROS_CONTROLS_NAMESPACE + "setpoint_complete_status"

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
  # Reset Fake GPS Start Location if Enabled
  if MAVROS_FAKE_GPS_SIM_SUPPORT:
    print("Calling Fake GPS reset to new geopoint")
    print(FAKE_GPS_HOME_GEOPOINT_WGS84)
    fake_gps_reset_geopoint(FAKE_GPS_HOME_GEOPOINT_WGS84)
    current_home = FAKE_GPS_HOME_GEOPOINT_WGS84
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
  

### Function to set mode
def update_mode(mode_new):
  global current_state
  global mode_client
  new_mode = SetModeRequest()
  new_mode.custom_mode = mode_new
  print("Updating mode")
  print(mode_new)
  mode_client = rospy.ServiceProxy(MAVROS_SET_MODE_SERVICE, SetMode)
  while current_state.mode != mode_new and not rospy.is_shutdown():
    time.sleep(.25)
    mode_client.call(new_mode)
    print("Waiting for mode to set")
    print("Set Value: " + mode_new)
    print("Cur Value: " + str(current_state.mode))


### Function to set armed state
def update_armed(armed_new):
  global current_state
  global arming_client
  arm_cmd = CommandBoolRequest()
  arm_cmd.value = armed_new
  print("Updating armed")
  print(armed_new)
  arming_client = rospy.ServiceProxy(MAVROS_ARMING_SERVICE, CommandBool)
  while current_state.armed != armed_new and not rospy.is_shutdown():
    time.sleep(.25)
    arming_client.call(arm_cmd)
    print("Waiting for armed value to set")
    print("Set Value: " + str(armed_new))
    print("Cur Value: " + str(current_state.armed))

### Function for sending set home current
def sethome_current():
  global current_home
  print('Sending mavlink set home current command')
  set_home_client = rospy.ServiceProxy(MAVROS_SET_HOME_SERVICE, CommandHome)
  time.sleep(.1) # VERY IMPORTANT - Sleep a bit between declaring a publisher and using it
  set_home_client(current_gps=True)

## Function for sending takeoff command
def takeoff(takeoff_height_m):
  print("Sending Takeoff Command to Altitude")
  takeoff_client = rospy.ServiceProxy(MAVROS_TAKEOFF_SERVICE, CommandTOL)
  time.sleep(.1) # VERY IMPORTANT - Sleep a bit between declaring a publisher and using it
  takeoff_client(min_pitch=10,altitude=takeoff_height_m)
  if MAVROS_FAKE_GPS_SIM_SUPPORT:
    print("Sending fake gps goto takeoff command")
    takeoff_geopoint=[-999,-999,FAKE_GPS_HOME_GEOPOINT_WGS84[2]+takeoff_height_m]
    fake_gps_goto_geopoint(takeoff_geopoint)
  print("Waiting for takeoff process to complete")
  time.sleep(10) # Wait for takeoff
    
### Function for switching to LAND mode
def land():
  global current_state
  global mavros_fake_gps_land_pub
  update_mode('LAND')
  if MAVROS_FAKE_GPS_SIM_SUPPORT:
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
    if MAVROS_FAKE_GPS_SIM_SUPPORT:
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

### Callback to get current state
def get_state_callback(state_msg):
  global current_state
  current_state = state_msg

### Function for simulating Fake GPS movement to new geopoint
def fake_gps_goto_geopoint(goto_geopoint_wgs84):
  global current_location_wgs84_geo
  # Send mavlink set home command and message
  fake_gps_goto_geopoint_pub = rospy.Publisher(MAVROS_FAKE_GPS_GOTO_GEOPOINT_TOPIC, GeoPoint, queue_size=1)
  time.sleep(.1)
  print("Sending Fake GPS move command:")
  print(goto_geopoint_wgs84)
  geopoint_msg=GeoPoint()
  geopoint_msg.latitude = goto_geopoint_wgs84[0]
  geopoint_msg.longitude = goto_geopoint_wgs84[1]
  geopoint_msg.altitude = goto_geopoint_wgs84[2]
  fake_gps_goto_geopoint_pub.publish(geopoint_msg)
  time.sleep(1)

### Function for reseting Fake GPS and global x,y NED home position at new geopoint
def fake_gps_reset_geopoint(reset_geopoint_wgs84):
  global current_location_wgs84_geo
  # Send mavlink set home command and message
  fake_gps_reset_geopoint_pub = rospy.Publisher(MAVROS_FAKE_GPS_RESET_GEOPOINT_TOPIC, GeoPoint, queue_size=1)
  time.sleep(.1)
  print("Sending Fake GPS reset to geopoint command")
  print(reset_geopoint_wgs84)
  geopoint_msg=GeoPoint()
  geopoint_msg.latitude = reset_geopoint_wgs84[0]
  geopoint_msg.longitude = reset_geopoint_wgs84[1]
  geopoint_msg.altitude = reset_geopoint_wgs84[2]
  fake_gps_reset_geopoint_pub.publish(geopoint_msg)
  print("Waiting for fake gps update to reset")
  time.sleep(12)
  

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
  time.sleep(.1)

  
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
  while not rospy.is_shutdown():

    #########################################
    # Start Mission
    #########################################
    # Send Setpoint Location Command
    print("Starting Setpoint Location Global Process")
    setpoint_location_global(SETPOINT_LOCATION_GLOBAL)
    ##########################################
##    # Send Setpoint Position Command
##    print("Starting Setpoint Position Local Process")
##    setpoint_position_body(SETPOINT_POSITION_BODY)
##    #########################################
##    # Send Setpoint Attitude Command
##    print("Sending Setpoint Attitude Control Message")
##    setpoint_attitude_ned(SETPOINT_ATTITUDE_NED)
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

