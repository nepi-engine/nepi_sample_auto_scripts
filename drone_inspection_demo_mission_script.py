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
# 1) Subscribes to NEPI nav_pose_current heading, orientation, position, location topics
# 2) Runs pre-mission processes
# 3) Runs mission goto command processes
# 4) Runs mission action processes
# 5) Runs post-mission processes

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
from nepi_edge_sdk_base import nepi_ros 
from nepi_edge_sdk_base import nepi_rbx

from std_msgs.msg import Empty,Bool, String, UInt8, Int8, Float32, Float64
from geographic_msgs.msg import GeoPoint
from nepi_ros_interfaces.msg import RBXInfo, RBXStatus, AxisControls, RBXErrorBounds, RBXGotoErrors, RBXMotorControl, \
     RBXGotoPose, RBXGotoPosition, RBXGotoLocation, SettingUpdate
from nepi_ros_interfaces.srv import RBXCapabilitiesQuery, RBXCapabilitiesQueryResponse

#########################################
# USER SETTINGS - Edit as Necessary 
#########################################
#RBX Robot Name
RBX_ROBOT_NAME = "ardupilot"

# Set Home Poistion
ENABLE_FAKE_GPS = True
SET_HOME = True
HOME_LOCATION = [47.6540828,-122.3187578,0.0]

# Process Timeout Values
STATE_TIMEOUT_SEC = 5
MODE_TIMEOUT_SEC = 5
ACTION_TIMEOUT_SEC = 10
GOTO_TIMEOUT_SEC = 12

# GoTo Position Global Settings
###################
# goto_location is [LAT, LONG, ALT_WGS84, YAW_NED_DEGREES]
# Altitude is specified as meters above the WGS-84 and converted to AMSL before sending
# Yaw is specified in NED frame degrees 0-360 or +-180 
GOTO_LOCATION = [47.6541208,-122.3186620, 10, -999] # [Lat, Long, Alt WGS84, Yaw NED Frame], Enter -999 to use current value
GOTO_LOCATION_CORNERS =  [[47.65412620,-122.31881480, -999, -999],[47.65402050,-122.31875320, -999, -999],[47.65391570,-122.31883630, -999, -999]]

#########################################
# ROS NAMESPACE SETUP
#########################################

# ROS namespace setup
NEPI_BASE_NAMESPACE = nepi_ros.get_base_namespace()

#########################################
# Node Class
#########################################

class drone_inspection_demo_mission(object):
  rbx_settings = []
  rbx_info = RBXInfo()
  rbx_status = RBXStatus()
  #######################
  ### Node Initialization
  def __init__(self):
    rospy.loginfo("Starting Initialization Processes")
    rospy.loginfo("Waiting for namespace containing: " + RBX_ROBOT_NAME)
    robot_namespace = nepi_ros.wait_for_node(RBX_ROBOT_NAME)
    robot_namespace = robot_namespace + "/"
    rospy.loginfo("Found namespace: " + robot_namespace)
    rbx_namespace = (robot_namespace + "rbx/")
    rospy.loginfo("Using rbx namesapce " + rbx_namespace)
    nepi_rbx.rbx_initialize(self,rbx_namespace)
    time.sleep(1)

    # Setup Settings, Info, and Status Subscribers
    self.NEPI_RBX_SETTINGS_TOPIC = robot_namespace + "settings_status"
    rospy.loginfo("Waiting for topic: " + self.NEPI_RBX_SETTINGS_TOPIC)
    nepi_ros.wait_for_topic(self.NEPI_RBX_SETTINGS_TOPIC)
    rbx_settings_pub = rospy.Publisher(robot_namespace + 'publish_settings', Empty, queue_size=1)
    rospy.loginfo("Starting rbx settings scubscriber callback")
    rospy.Subscriber(self.NEPI_RBX_SETTINGS_TOPIC, String, self.rbx_settings_callback, queue_size=None)
    while self.rbx_settings is None and not rospy.is_shutdown():
      rospy.loginfo("Waiting for current rbx settings to publish")
      time.sleep(1)
      rbx_settings_pub.publish(Empty())
    rospy.loginfo(self.rbx_info)

    self.NEPI_RBX_INFO_TOPIC = rbx_namespace + "info" # RBX Info Message
    rospy.loginfo("Waiting for topic: " + self.NEPI_RBX_INFO_TOPIC)
    nepi_ros.wait_for_topic(self.NEPI_RBX_INFO_TOPIC)
    rbx_info_pub = rospy.Publisher(rbx_namespace + 'publish_info', Empty, queue_size=1)
    rospy.loginfo("Starting rbx info scubscriber callback")
    rospy.Subscriber(self.NEPI_RBX_INFO_TOPIC,RBXInfo, self.rbx_info_callback, queue_size=None)
    while self.rbx_info is None and not rospy.is_shutdown():
      rospy.loginfo("Waiting for current rbx info to publish")
      time.sleep(1)
      rbx_info_pub.publish(Empty())
    rospy.loginfo(self.rbx_info)

    self.NEPI_RBX_STATUS_TOPIC = rbx_namespace + "status" # RBX Status Message
    rospy.loginfo("Waiting for topic: " + self.NEPI_RBX_STATUS_TOPIC)
    nepi_ros.wait_for_topic(self.NEPI_RBX_STATUS_TOPIC)
    rbx_status_pub = rospy.Publisher(rbx_namespace + 'publish_status', Empty, queue_size=1)
    rospy.loginfo("Starting rbx status scubscriber callback")
    rospy.Subscriber(self.NEPI_RBX_STATUS_TOPIC, RBXStatus, self.rbx_status_callback, queue_size=None)
    while self.rbx_status is None and not rospy.is_shutdown():
      rospy.loginfo("Waiting for current rbx status to publish")
      time.sleep(0.1)
      rbx_status_pub.publish(Empty())
    rospy.loginfo(self.rbx_status)

    # Create fake gps update process
    self.rbx_enable_fake_gps_pub.publish(ENABLE_FAKE_GPS)

    if SET_HOME:
      rospy.loginfo("Upating RBX Home Location")
      new_home_geo = GeoPoint()
      new_home_geo.latitude = HOME_LOCATION[0]
      new_home_geo.longitude = HOME_LOCATION[1]
      new_home_geo.altitude = HOME_LOCATION[2]
      self.rbx_set_home_pub.publish(new_home_geo)

      self.rbx_reset_fake_gps_pub.publish(Empty())
      nepi_ros.sleep(15,100) # Give system time to stabilize on new gps location
    SNAPSHOT_TRIGGER_TOPIC = NEPI_BASE_NAMESPACE + "snapshot_trigger"
    self.snapshot_trigger_pub = rospy.Publisher(SNAPSHOT_TRIGGER_TOPIC, Empty, queue_size = 1)
    ## Initiation Complete
    rospy.loginfo("Initialization Complete")

  #######################
  ### Node Methods

  ## Function for custom pre-mission actions
  def pre_mission_actions(self):
    ###########################
    # Start Your Custom Actions
    ###########################
    success = True
    # Set Mode to Guided
    success = nepi_rbx.set_rbx_mode(self,"GUIDED",timeout_s = MODE_TIMEOUT_SEC)
    # Arm System
    success = nepi_rbx.set_rbx_state(self,"ARM",timeout_s = STATE_TIMEOUT_SEC)
    # Send Takeoff Command
    success=nepi_rbx.go_rbx_action(self,"TAKEOFF",timeout_s = ACTION_TIMEOUT_SEC)
    nepi_ros.sleep(2,10)
    ###########################
    # Stop Your Custom Actions
    ###########################
    print("Pre-Mission Actions Complete")
    return success

  ## Function for custom mission
  def mission(self):
    ###########################
    # Start Your Custom Process
    ###########################
    success = True
    ##########################################
    # Send goto Position Command
    print("Starting goto Location Process")
    success = nepi_rbx.goto_rbx_location(self,GOTO_LOCATION,timeout_s = GOTO_TIMEOUT_SEC)
    #########################################
    # Run Mission Actions
    print("Starting Mission Actions")
    success = self.mission_actions()
   #########################################
    # Send goto Location Loop Command
    '''
    for ind in range(3):
      # Send goto Location Command
      print("Starting goto Location Corners Process")
      success = nepi_rbx.goto_rbx_location(self,GOTO_LOCATION_CORNERS[ind],timeout_s = GOTO_TIMEOUT_SEC)
      # Run Mission Actions
      print("Starting Mission Actions")
      success = self.mission_actions()
    '''
    ###########################
    # Stop Your Custom Process
    ###########################
    print("Mission Processes Complete")
    return success

  ## Function for custom mission actions
  def mission_actions(self):
    ###########################
    # Start Your Custom Actions
    ###########################
    ## Send Snapshot Trigger
    success = True
    success = nepi_rbx.set_rbx_process_name(self,"SNAPSHOT EVENT")
    rospy.loginfo("Sending snapshot event trigger")
    self.snapshot()
    nepi_ros.sleep(2,10)
    ###########################
    # Stop Your Custom Actions
    ###########################
    print("Mission Actions Complete")
    return success

  ## Function for custom post-mission actions
  def post_mission_actions(self):
    ###########################
    # Start Your Custom Actions
    ###########################
    success = True
    #success = nepi_rbx.set_rbx_mode(self,"LAND", timeout_s = MODE_TIMEOUT_SEC) # Uncomment to change to Land mode
    #success = nepi_rbx.set_rbx_mode(self,"LOITER", timeout_s = MODE_TIMEOUT_SEC) # Uncomment to change to Loiter mode
    success = nepi_rbx.set_rbx_mode(self,"RTL", timeout_s = MODE_TIMEOUT_SEC) # Uncomment to change to home mode
    #success = nepi_rbx.set_rbx_mode(self,"RESUME", timeout_s = MODE_TIMEOUT_SEC) # Uncomment to return to last mode
    nepi_ros.sleep(1,10)
    ###########################
    # Stop Your Custom Actions
    ###########################
    print("Post-Mission Actions Complete")
    return success

  #######################
  # RBX Status Callbacks
  ### Callback to update rbx current state value
  def rbx_info_callback(self,info_msg):
    self.rbx_info = info_msg

  ### Callback to update rbx current state value
  def rbx_status_callback(self,status_msg):
    self.rbx_status = status_msg

  #######################
  # Mission Action Functions

  ### Function to send snapshot event trigger and wait for completion
  def snapshot(self):
    self.snapshot_trigger_pub.publish(Empty())
    print("Snapshot trigger sent")

  #######################
  ### RBX Status Callbacks
  def rbx_settings_callback(self, msg):
    self.rbx_settings = msg.data

  def rbx_info_callback(self, msg):
    self.rbx_info = msg


  def rbx_status_callback(self, msg):
    self.rbx_status = msg

  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    rospy.loginfo("Shutting down: Executing script cleanup actions")

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
  rospy.loginfo("Launching node named: " + node_name)
  node_class = eval(node_name)
  node = node_class()
  #########################################
  # Run Pre-Mission Custom Actions
  print("Starting Mission Actions")
  success = node.pre_mission_actions()
  if success:
    #########################################
    # Start Mission
    #########################################
    # Send goto Location Command
    print("Starting Mission Processes")
    success = node.mission()
    #########################################
  # End Mission
  #########################################
  # Run Post-Mission Actions
  print("Starting Post-Goto Actions")
  success = node.post_mission_actions()
  nepi_ros.sleep(10,100)
  #########################################
  #Mission Complete, Shutting Down
  rospy.signal_shutdown("Mission Complete, Shutting Down")

  #Set up node shutdown
  rospy.on_shutdown(node.cleanup_actions)
  # Spin forever (until object is detected)
  rospy.spin()


  



