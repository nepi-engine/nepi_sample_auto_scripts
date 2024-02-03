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

# Sample NEPI Config Script.
# 1. Connect NEPI NavPose topics to appropriate mavros topics
# 2. Sets NEPI GPS source and time synchronization

import rospy
import time

from std_msgs.msg import String, Bool, Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped
from nepi_ros_interfaces.srv import NavPoseQuery, NavPoseQueryRequest


#########################################
# USER SETTINGS - Edit as Necessary 
#########################################

SYNC_NEPI2GPS_CLOCK = True # Set to false to disable GPS clock sync


#########################################
# ROS NAMESPACE SETUP
#########################################

NEPI_BASE_NAMESPACE = "/nepi/s2x/"
NEPI_RBX_NAMESPACE = NEPI_BASE_NAMESPACE + "ardupilot/rbx/"

# NEPI MAVLINK RBX Driver NavPose Publish Topics
NEPI_RBX_NAVPOSE_GPS_TOPIC = NEPI_RBX_NAMESPACE + "gps_fix"
NEPI_RBX_NAVPOSE_ODOM_TOPIC = NEPI_RBX_NAMESPACE + "odom"
NEPI_RBX_NAVPOSE_HEADING_TOPIC = NEPI_RBX_NAMESPACE + "heading"

### Setup NEPI NavPose Settings Topic Namespaces
NEPI_SET_NAVPOSE_GPS_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose_mgr/set_gps_fix_topic"
NEPI_SET_NAVPOSE_HEADING_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose_mgr/set_heading_topic"
NEPI_SET_NAVPOSE_ORIENTATION_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose_mgr/set_orientation_topic"
NEPI_ENABLE_NAVPOSE_GPS_CLOCK_SYNC_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose_mgr/enable_gps_clock_sync"


#########################################
# Globals
#########################################

mavros_global_msg=None
mavros_heading_msg=None
mavros_orientation_msg=None

#########################################
# Methods
#########################################

### System Initialization processes
def initialize_actions():
  print("")
  print("Starting Initialization")
  # Start timer callback that sends regular set navepose updates
  print("Starting set navpose topics timer callback")
  rospy.Timer(rospy.Duration(5.0), set_nepi_navpose_topics_callback)
  print("Initialization Complete")



### Callback to set NEPI navpose topics
def set_nepi_navpose_topics_callback(timer):
  ##############################
  # Update Global Location source
  print("Waiting for topic: " +NEPI_RBX_NAVPOSE_GPS_TOPIC)
  wait_for_topic(NEPI_RBX_NAVPOSE_GPS_TOPIC)
  set_gps_pub = rospy.Publisher(NEPI_SET_NAVPOSE_GPS_TOPIC, String, queue_size=1)
  time.sleep(1) # Wait between creating and using publisher
  set_gps_pub.publish(NEPI_RBX_NAVPOSE_GPS_TOPIC)
  print("GPS Topic Set to: " + NEPI_RBX_NAVPOSE_GPS_TOPIC)
  ##############################
  # Update Orientation source
  print("Waiting for topic: " + NEPI_RBX_NAVPOSE_ODOM_TOPIC)
  wait_for_topic(NEPI_RBX_NAVPOSE_ODOM_TOPIC)
  set_orientation_pub = rospy.Publisher(NEPI_SET_NAVPOSE_ORIENTATION_TOPIC, String, queue_size=1)
  time.sleep(1) # Wait between creating and using publisher
  set_orientation_pub.publish(NEPI_RBX_NAVPOSE_ODOM_TOPIC)
  print("Orientation Topic Set to: " + NEPI_RBX_NAVPOSE_ODOM_TOPIC)
  ##############################
  # Update Heading source
  print("Waiting for topic: " + NEPI_RBX_NAVPOSE_HEADING_TOPIC)
  wait_for_topic(NEPI_RBX_NAVPOSE_HEADING_TOPIC)
  set_heading_pub = rospy.Publisher(NEPI_SET_NAVPOSE_HEADING_TOPIC, String, queue_size=1)
  time.sleep(1) # Wait between creating and using publisher
  set_heading_pub.publish(NEPI_RBX_NAVPOSE_HEADING_TOPIC)
  print("Heading Topic Set to: " + NEPI_RBX_NAVPOSE_HEADING_TOPIC)
  ##############################
  # Sync NEPI clock to GPS timestamp
  set_gps_timesync_pub = rospy.Publisher(NEPI_ENABLE_NAVPOSE_GPS_CLOCK_SYNC_TOPIC, Bool, queue_size=1)
  time.sleep(1) # Wait between creating and using publisher
  set_gps_timesync_pub.publish(data=SYNC_NEPI2GPS_CLOCK)
  print("GPS Clock Sync Topic Set to: " + str(SYNC_NEPI2GPS_CLOCK))
  print("Setup complete")

#######################
# Initialization Functions

### Function to find a topic
def find_topic(topic_name):
  topic = ""
  topic_list=rospy.get_published_topics(namespace='/')
  for topic_entry in topic_list:
    if topic_entry[0].find(topic_name) != -1:
      topic = topic_entry[0]
  return topic

### Function to check for a topic 
def check_for_topic(topic_name):
  topic_exists = True
  topic=find_topic(topic_name)
  if topic == "":
    topic_exists = False
  return topic_exists

### Function to wait for a topic
def wait_for_topic(topic_name):
  topic = ""
  while topic == "" and not rospy.is_shutdown():
    topic=find_topic(topic_name)
    time.sleep(.1)
  return topic

#######################
# StartNode and Cleanup Functions

### Cleanup processes on node shutdown
def cleanup_actions():
  print("Shutting down: Executing script cleanup actions")

### Script Entrypoint
def startNode():
  rospy.loginfo("Starting Ardupilot NavPose Config Script")
  rospy.init_node("ardupilot_navpose_config_script")
  # Run initialization processes
  initialize_actions()
  # Run cleanup actions on rospy shutdown
  rospy.on_shutdown(cleanup_actions)
  # Spin forever
  rospy.spin()  
  


#########################################
# Main
#########################################

if __name__ == '__main__':
  startNode()

