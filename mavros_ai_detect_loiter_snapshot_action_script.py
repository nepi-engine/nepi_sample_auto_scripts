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
# 1) Monitors detection for specfic target class 
# 2) Sets system to Loiter mode
# 3) Calls a Snapshot Event Auto Script
# 4) Delays some set time for snapshot action
# 5) Sets system back to original mode
# 6) Delays, then starts monitoring again

# Requires the following additional scripts are running
# (Optional) Some Snapshot Action Automation Script like the following
# a)snapshot_event_save_data_auto_script.py
# b)snapshot_event_send_to_cloud_auto_script.py
# These scripts are available for download at:
# [link text](https://github.com/numurus-nepi/nepi_sample_auto_scripts)

import rospy
import time
import numpy as np

from std_msgs.msg import Empty
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from darknet_ros_msgs.msg import BoundingBoxes


#####################################################################################
# SETUP - Edit as Necessary ##################################
##########################################

OBJ_LABEL_OF_INTEREST = 'person'
ACTION_DELAY_S = 10 # Time to Loiter at this spot
RESET_DELAY_S = 5 # Min delay between triggers


# ROS namespace setup
NEPI_BASE_NAMESPACE = "/nepi/s2x/"
MAVROS_NAMESPACE = NEPI_BASE_NAMESPACE + "pixhawk_mavlink/"
# MAVROS Subscriber Topics
MAVROS_STATE_TOPIC = MAVROS_NAMESPACE + "state"
# MAVROS Required Services
MAVROS_ARMING_SERVICE = MAVROS_NAMESPACE + "cmd/arming"
MAVROS_SET_MODE_SERVICE = MAVROS_NAMESPACE + "set_mode"
# AI Classifier Subscription Topic
AI_BOUNDING_BOXES_TOPIC = NEPI_BASE_NAMESPACE + "classifier/bounding_boxes"
# Snapshot Event Publish Topic
SNAPSHOT_TOPIC = NEPI_BASE_NAMESPACE + "snapshot_event"


#####################################################################################
# Globals
#####################################################################################
snapshot_trigger_pub = rospy.Publisher(SNAPSHOT_TOPIC, Empty, queue_size = 1)
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
  time.sleep(1)
  original_state = current_state
  print("Getting Original State")
  print(original_state)
  print("Completed Initialization")


# Action upon detection of object of interest
def object_detected_callback(bounding_box_msg):
  global snapshot_trigger_pub
  # Iterate over all of the objects reported by the detector
  for box in bounding_box_msg.bounding_boxes:
    # Check for the object of interest and take appropriate actions
    if box.Class == OBJ_LABEL_OF_INTEREST:
      print("Detected a " + OBJ_LABEL_OF_INTEREST)
      print("Switching to Loiter mode")
      loiter() # Change mode to Loiter
      print("Sending snapshot event trigger")
      snapshot_trigger_pub.publish(Empty())
      print("Staying in Loiter mode for " + str(ACTION_DELAY_S) + " secs")
      time.sleep(RESET_DELAY_S)
      print("Switching back to original mode")
      continue_mission()
      print("Delaying next trigger for " + str(RESET_DELAY_S) + " secs")
      time.sleep(RESET_DELAY_S)
      print("Waiting for next " + OBJ_LABEL_OF_INTEREST + " detection")

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
  time.sleep(15) # Wait for takeoff
    
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
  rospy.loginfo("Starting MAVROS Set Attitude Target automation script")
  rospy.init_node("mavros_set_attitude_target_auto_script")
  #initialize system including pan scan process
  initialize_actions()
  # Set up object detector subscriber
  print("Waiting for Classifier Topic to detect and publish")
  rospy.Subscriber(AI_BOUNDING_BOXES_TOPIC, BoundingBoxes, object_detected_callback, queue_size = 1)
  # run cleanup actions on shutdown
  rospy.on_shutdown(cleanup_actions)
  # Spin forever
  rospy.spin()


#####################################################################################
# Main
#####################################################################################

if __name__ == '__main__':
  startNode()

