#!/usr/bin/env python

__author__ = "Jason Seawall"
__copyright__ = "Copyright 2023, Numurus LLC"
__email__ = "nepi@numurus.com"
__credits__ = ["Jason Seawall", "Josh Maximoff"]

__license__ = "GPL"
__version__ = "2.0.4.0"


# Sample NEPI Automation Script.
# Uses onboard ROS python and mavros libraries to
### Expects Classifier to be running ###
# 1) Connects to Target Range and Bearing data Topic
# 2) Monitors AI detector output for specfic target class 
# 3) Changes system to Guided mode
# 4) Sends Setpoint position command based on target range and bearing
# 5) Waits to achieve setpoint
# 6) Sets system back to original mode
# 6) Delays, then waits for next detection

# Requires the following additional scripts are running
# a) navpose_get_and_publish_auto_script.py
# b) mavros_navpose_config_auto_script.py
# b) mavros_setpoint_controls_auto_script.py
# c) ai_detector_setup_start_auto_script.py
# d) zed_3d_targeting_auto_script.py
# e) (Optional) MAVROS_fake_gps_sim_auto_script.py if a real GPS fix is not available
# These scripts are available for download at:
# [link text](https://github.com/numurus-nepi/nepi_sample_auto_scripts)

import rospy
import time
import numpy as np

from std_msgs.msg import Empty, Bool, String, Float64, Float64MultiArray
from nepi_ros_interfaces.msg import TargetLocalization
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped
from geographic_msgs.msg import GeoPoint, GeoPose, GeoPoseStamped
from mavros_msgs.msg import State, AttitudeTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL, CommandHome
from darknet_ros_msgs.msg import BoundingBoxes


#####################################################################################
# SETUP - Edit as Necessary ##################################
##########################################

OBJ_LABEL_OF_INTEREST = 'person'
TARGET_OFFSET_GOAL_M = 1 # How close to set setpoint to target
RESET_DELAY_S = 5 # Min delay between triggering a new move


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
# MAVROS Subscriber Topics
MAVROS_STATE_TOPIC = MAVROS_NAMESPACE + "state"
# MAVROS Required Services
MAVROS_ARMING_SERVICE = MAVROS_NAMESPACE + "cmd/arming"
MAVROS_SET_MODE_SERVICE = MAVROS_NAMESPACE + "set_mode"
# NEPI MAVROS Control Publisher Topics
MAVROS_CONTROL_ATTITUDE_TOPIC = MAVROS_CONTROLS_NAMESPACE + "setpoint_command_attitude"
MAVROS_CONTROL_POSITION_TOPIC = MAVROS_CONTROLS_NAMESPACE + "setpoint_command_position"
MAVROS_CONTROL_LOCATION_TOPIC = MAVROS_CONTROLS_NAMESPACE + "setpoint_command_location"
# NEPI MAVROS Control Subscriber Topics
MAVROS_CONTROL_PROCESS_COMPLETE_TOPIC = MAVROS_CONTROLS_NAMESPACE + "setpoint_complete_status"
# AI Classifier Subscription Topic
AI_BOUNDING_BOXES_TOPIC = NEPI_BASE_NAMESPACE + "classifier/bounding_boxes"
# AI 3D Targeting Subscriber Topic
TARGET_DATA_INPUT_TOPIC = NEPI_BASE_NAMESPACE + "targeting/targeting_data"
# MAVROS Fake GPS Publish Topics
MAVROS_FAKE_GPS_GOTO_GEOPOINT_TOPIC = MAVROS_NAMESPACE + "fake_gps/goto_geopoint_wgs84"
MAVROS_FAKE_GPS_RESET_GEOPOINT_TOPIC = MAVROS_NAMESPACE + "fake_gps/reset_geopoint_wgs84"


#####################################################################################
# Globals
#####################################################################################
current_state = None
original_state = None

#####################################################################################
# Methods
#####################################################################################

### System Initialization processes
def initialize_actions():
  global current_state
  global original_state
  print("Starting get_state_callback")
  # Wait for topic
  print("Waiting for topic: " + MAVROS_STATE_TOPIC)
  wait_for_topic(MAVROS_STATE_TOPIC, 'mavros_msgs/State')  
  rospy.Subscriber(MAVROS_STATE_TOPIC, State, callback = get_state_callback)
  print("Waiting for first state update")
  while current_state is not None and not rospy.is_shutdown():
    time.sleep(0.1)
  original_state = current_state
  print("Getting Original State")
  print(original_state)
  print("Starting get_target_data_callback")
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
  # Wait for Target Data topic
  print("Waiting for topic: " + TARGET_DATA_INPUT_TOPIC)
  wait_for_topic(TARGET_DATA_INPUT_TOPIC, 'nepi_ros_interfaces/TargetLocalization')
  print("Completed Initialization")

### Callback to get current state
def get_target_date_callback(target_data_msg):
  global current_target_data
  current_target_data = target_data_msg

# Action upon detection and targeting for object of interest 
def object_goto_callback(target_data_msg):
  global current_state
  global original_state
    print("Recieved target data message")
    print(target_data_msg)
    # Check for the object of interest and take appropriate actions
    if target_data_msg.name == OBJ_LABEL_OF_INTEREST:
      original_state = current_state
      print("Detected a " + OBJ_LABEL_OF_INTEREST)
      # Get target data for detected object of interest
      target_range_m = target_data_msg.range_m
      target_yaw_deg = target_data_msg.azimuth_deg
      target_pitch_deg = target_data_msg.elevation_deg
      # Calculate setpoint to target using offset goal
      setpoint_range_m = target_range_m - TARGET_OFFSET_GOAL_M
      setpoint_x_body_m = setpoint_range_m * math.cos(math.radians(target_yaw_d))
      setpoint_y_body_m = setpoint_range_m * math.sin(math.radians(target_yaw_d))
      setpoint_z_body_m = setpoint_range_m * math.sin(math.radians(target_pitch_d))
      setpoint_position_body_m = [setpoint_x_body_m,setpoint_z_body_m,setpoint_y_body_m]
      ##########################################
      # Switch to Guided Mode and Send Setpoint Command
      print("Switching to Guided mode")
      guided() # Change mode to Guided
      # Send setpoint command and wait for completion
      print("Sending setpoint position body command")
      print(setpoint_position_body_m)
      setpoint_position_body(setpoint_position_body_m)
      ##########################################
      print("Switching back to original mode")
      continue_mission()
      print("Delaying next trigger for " + str(RESET_DELAY_S) + " secs")
      time.sleep(RESET_DELAY_S)
      print("Waiting for next " + OBJ_LABEL_OF_INTEREST + " detection")
    else:
      print("No " + OBJ_LABEL_OF_INTEREST + " type for target data)
      time.sleep(1)

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

### Function for switching to LOITER mode
def guided():
  update_mode('GUIDED')

### Function for switching back to current mission
def continue_mission():
  global original_state
  mode_org = original_state.mode.upper()
  update_mode(mode_org)

### Callback to get current state
def get_state_callback(state_msg):
  global current_state
  current_state = state_msg



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

### Cleanup processes on node shutdown
def cleanup_actions():
  global org_mode
  time.sleep(.1)
  
### Script Entrypoint
def startNode():
  rospy.loginfo("Starting MAVROS Set Attitude Target automation script")
  rospy.init_node("mavros_set_attitude_target_auto_script")
  #initialize system including pan scan process
  initialize_actions()
  # Set up object detector subscriber
  print("Starting Object Goto callback")
  rospy.Subscriber(TARGET_DATA_INPUT_TOPIC, TargetLocalization, object_goto_callback, queue_size = 1)
  # run cleanup actions on shutdown
  rospy.on_shutdown(cleanup_actions)
  # Spin forever
  rospy.spin()


#####################################################################################
# Main
#####################################################################################

if __name__ == '__main__':
  startNode()

