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
# Uses onboard ROS and MAVROS python libraries to
# 1. Connect NEPI NavPose topics to appropriate mavros topics
# 2. Set NEPI GPS source time synchronization

import rospy
import time

from std_msgs.msg import String, Bool, Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped
from nepi_ros_interfaces.srv import NavPoseQuery, NavPoseQueryRequest


#####################################################################################
# SETUP - Edit as Necessary ##################################
##########################################
SYNC_NEPI2GPS_CLOCK = True # Set to false to disable GPS clock sync

# ROS namespace setup
NEPI_BASE_NAMESPACE = "/nepi/s2x/"
MAVROS_NAMESPACE = NEPI_BASE_NAMESPACE + "pixhawk_mavlink/"

# MAVROS NavPose Source Topics
MAVROS_SOURCE_NAV_TOPIC = MAVROS_NAMESPACE + "global_position/global"
MAVROS_SOURCE_HEADING_TOPIC = MAVROS_NAMESPACE + "global_position/compass_hdg"
MAVROS_SOURCE_ORIENTATION_TOPIC = MAVROS_NAMESPACE + "global_position/local"

### Setup NEPI NavPose Settings Topic Namespaces
NEPI_SET_NAVPOSE_GPS_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose_mgr/set_gps_fix_topic"
NEPI_SET_NAVPOSE_HEADING_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose_mgr/set_heading_topic"
NEPI_SET_NAVPOSE_ORIENTATION_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose_mgr/set_orientation_topic"
NEPI_ENABLE_NAVPOSE_GPS_CLOCK_SYNC_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose_mgr/enable_gps_clock_sync"


#####################################################################################
# Globals
#####################################################################################
mavros_global_msg=None
mavros_heading_msg=None
mavros_orientation_msg=None

#####################################################################################
# Methods
#####################################################################################

### System Initialization processes
def initialize_actions():
  global mavros_global_msg
  global mavros_heading_msg
  global mavros_orientation_msg
  print("")
  print("Starting Initialization")
  print("Initialization Complete")


### Callback to set NEPI navpose topics
def set_nepi_navpose_topics_callback(timer):
  ##############################
  # Update Global Location source
  print("Waiting for topic: " + MAVROS_SOURCE_NAV_TOPIC)
  wait_for_topic(MAVROS_SOURCE_NAV_TOPIC, 'sensor_msgs/NavSatFix')
  set_gps_pub = rospy.Publisher(NEPI_SET_NAVPOSE_GPS_TOPIC, String, queue_size=1)
  time.sleep(.1) # Wait between creating and using publisher
  set_gps_pub.publish(MAVROS_SOURCE_NAV_TOPIC)
  print("GPS Topic Set to: " + MAVROS_SOURCE_NAV_TOPIC)
  ##############################
  # Update Orientation source
  print("Waiting for topic: " + MAVROS_SOURCE_ORIENTATION_TOPIC)
  wait_for_topic(MAVROS_SOURCE_ORIENTATION_TOPIC, 'nav_msgs/Odometry')
  set_orientation_pub = rospy.Publisher(NEPI_SET_NAVPOSE_ORIENTATION_TOPIC, String, queue_size=1)
  time.sleep(.1) # Wait between creating and using publisher
  set_orientation_pub.publish(MAVROS_SOURCE_ORIENTATION_TOPIC)
  print("Orientation Topic Set to: " + MAVROS_SOURCE_ORIENTATION_TOPIC)
  ##############################
  # Update Heading source
  print("Waiting for topic: " + MAVROS_SOURCE_HEADING_TOPIC)
  wait_for_topic(MAVROS_SOURCE_HEADING_TOPIC, 'std_msgs/Float64')
  set_heading_pub = rospy.Publisher(NEPI_SET_NAVPOSE_HEADING_TOPIC, String, queue_size=1)
  time.sleep(.1) # Wait between creating and using publisher
  set_heading_pub.publish(MAVROS_SOURCE_HEADING_TOPIC)
  print("Heading Topic Set to: " + MAVROS_SOURCE_HEADING_TOPIC)
  ##############################
  # Sync NEPI clock to GPS timestamp
  set_gps_timesync_pub = rospy.Publisher(NEPI_ENABLE_NAVPOSE_GPS_CLOCK_SYNC_TOPIC, Bool, queue_size=1)
  time.sleep(.1) # Wait between creating and using publisher
  set_gps_timesync_pub.publish(data=SYNC_NEPI2GPS_CLOCK)
  print("GPS Clock Sync Topic Set to: " + str(SYNC_NEPI2GPS_CLOCK))
  print("Setup complete")

### Callback to check mavros global data
def check_global_callback(global_msg):
  global mavros_global_msg
  mavros_global_msg = global_msg

### Callback to check mavros heading data
def check_heading_callback(heading_msg):
  global mavros_heading_msg
  mavros_heading_msg = heading_msg

### Callback to check mavros orientation data
def check_orientation_callback(orientation_msg):
  global mavros_orientation_msg
  mavros_orientation_msg = orientation_msg

### Function to wait for topic to exist
def check_for_topic(topic_name,message_name):
  topic_list=rospy.get_published_topics(namespace='/')
  topic_to_connect=[topic_name, message_name]
  topic_exists = False
  if topic_to_connect in topic_list:
    topic_exists = True
  print(topic_exists)
  return topic_exists

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
  rospy.init_node("mavros_navpose_config_autos_script")
  rospy.loginfo("Starting Mavros NavPose config automation script")
  # Run initialization processes
  initialize_actions()
  # Start timer callback that sends regular set navepose updates
  print("Starting set navpose topics timer callback")
  rospy.Timer(rospy.Duration(5.0), set_nepi_navpose_topics_callback)
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

