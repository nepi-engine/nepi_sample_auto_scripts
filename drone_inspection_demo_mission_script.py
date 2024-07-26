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

# Robot Settings Overides
###################
TAKEOFF_HEIGHT_M = 10.0

# GoTo Position Global Settings
###################
# goto_location is [LAT, LONG, ALT_WGS84, YAW_NED_DEGREES]
# Altitude is specified as meters above the WGS-84 and converted to AMSL before sending
# Yaw is specified in NED frame degrees 0-360 or +-180 
GOTO_LOCATION = [47.6541208,-122.3186620, 10, -999] # [Lat, Long, Alt WGS84, Yaw NED Frame], Enter -999 to use current value
GOTO_LOCATION_CORNERS =  [[47.65412620,-122.31881480, -999, -999],[47.65402050,-122.31875320, -999, -999],[47.65391570,-122.31883630, -999, -999]]

# Set Home Poistion
ENABLE_FAKE_GPS = True
SET_HOME = True
HOME_LOCATION = [47.6540828,-122.3187578,0.0]

# Goto Error Settings
GOTO_MAX_ERROR_M = 2.0 # Goal reached when all translation move errors are less than this value
GOTO_MAX_ERROR_DEG = 2.0 # Goal reached when all rotation move errors are less than this value
GOTO_STABILIZED_SEC = 1.0 # Window of time that setpoint error values must be good before proceeding

# CMD Timeout Values
CMD_STATE_TIMEOUT_SEC = 5
CMD_MODE_TIMEOUT_SEC = 5
CMD_ACTION_TIMEOUT_SEC = 15
CMD_GOTO_TIMEOUT_SEC = 15


#########################################
# Node Class
#########################################

class drone_inspection_demo_mission(object):

  NEPI_BASE_NAMESPACE = nepi_ros.get_base_namespace()

  rbx_settings = []
  rbx_info = RBXInfo()
  rbx_status = RBXStatus()
  #######################
  ### Node Initialization
  def __init__(self):
    rospy.loginfo("DRONE_INSPECT: Starting Initialization Processes")
    rospy.loginfo("DRONE_INSPECT: Waiting for namespace containing: " + RBX_ROBOT_NAME)
    robot_namespace = nepi_ros.wait_for_node(RBX_ROBOT_NAME)
    robot_namespace = robot_namespace + "/"
    rospy.loginfo("DRONE_INSPECT: Found namespace: " + robot_namespace)
    rbx_namespace = (robot_namespace + "rbx/")
    rospy.loginfo("DRONE_INSPECT: Using rbx namesapce " + rbx_namespace)
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
    rospy.loginfo("DRONE_INSPECT: Udated settings:" + settings_str)

    # Create fake gps update process
    self.rbx_enable_fake_gps_pub.publish(ENABLE_FAKE_GPS)
    if SET_HOME:
      rospy.loginfo("DRONE_INSPECT: Upating RBX Home Location")
      new_home_geo = GeoPoint()
      new_home_geo.latitude = HOME_LOCATION[0]
      new_home_geo.longitude = HOME_LOCATION[1]
      new_home_geo.altitude = HOME_LOCATION[2]
      self.rbx_set_home_pub.publish(new_home_geo)
      self.rbx_reset_fake_gps_pub.publish(Empty())
      nepi_ros.sleep(15,100) # Give system time to stabilize on new gps location

    # Setup mission action processes
    SNAPSHOT_TRIGGER_TOPIC = self.NEPI_BASE_NAMESPACE + "snapshot_trigger"
    self.snapshot_trigger_pub = rospy.Publisher(SNAPSHOT_TRIGGER_TOPIC, Empty, queue_size = 1)
    ## Initiation Complete
    rospy.loginfo("DRONE_INSPECT: Initialization Complete")


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
    success = nepi_rbx.set_rbx_mode(self,"GUIDED",timeout_sec =CMD_MODE_TIMEOUT_SEC)
    # Arm System
    success = nepi_rbx.set_rbx_state(self,"ARM",timeout_sec = CMD_STATE_TIMEOUT_SEC)
    # Send Takeoff Command
    success=nepi_rbx.go_rbx_action(self,"TAKEOFF",timeout_sec =CMD_ACTION_TIMEOUT_SEC)
    time.sleep(2)
    error_str = str(self.rbx_status.errors_prev)
    if success:
      rospy.loginfo("DRONE_INSPECT: Takeoff completed with errors: " + error_str )
    else:
      rospy.loginfo("DRONE_INSPECT: Takeoff failed with errors: " + error_str )
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
    success = nepi_rbx.goto_rbx_location(self,GOTO_LOCATION,timeout_sec =CMD_GOTO_TIMEOUT_SEC)

    error_str = str(self.rbx_status.errors_prev)
    if success:
      rospy.loginfo("DRONE_INSPECT: Goto Location completed with errors: " + error_str )
    else:
      rospy.loginfo("DRONE_INSPECT: Goto Location failed with errors: " + error_str )
    nepi_ros.sleep(2,10)
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
      success = nepi_rbx.goto_rbx_location(self,GOTO_LOCATION_CORNERS[ind],timeout_sec =CMD_GOTO_TIMEOUT_SEC)
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
    rospy.loginfo("DRONE_INSPECT: Sending snapshot event trigger")
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
    #success = nepi_rbx.set_rbx_mode(self,"LAND", timeout_sec =CMD_MODE_TIMEOUT_SEC) # Uncomment to change to Land mode
    #success = nepi_rbx.set_rbx_mode(self,"LOITER", timeout_sec =CMD_MODE_TIMEOUT_SEC) # Uncomment to change to Loiter mode
    success = nepi_rbx.set_rbx_mode(self,"RTL", timeout_sec =CMD_MODE_TIMEOUT_SEC) # Uncomment to change to home mode
    #success = nepi_rbx.set_rbx_mode(self,"RESUME", timeout_sec =CMD_MODE_TIMEOUT_SEC) # Uncomment to return to last mode
    nepi_ros.sleep(1,10)
    ###########################
    # Stop Your Custom Actions
    ###########################
    print("Post-Mission Actions Complete")
    return success


  #######################
  # Mission Action Functions

  ### Function to send snapshot event trigger and wait for completion
  def snapshot(self):
    self.snapshot_trigger_pub.publish(Empty())
    print("Snapshot trigger sent")


  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    rospy.loginfo("DRONE_INSPECT: Shutting down: Executing script cleanup actions")

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
  rospy.loginfo("DRONE_INSPECT: Launching node named: " + node_name)
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


  



