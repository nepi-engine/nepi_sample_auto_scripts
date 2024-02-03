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
# 1. Connects NEPI NavPose topics to appropriate navpose topics


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


#########################################
# ROS NAMESPACE SETUP
#########################################

NEPI_BASE_NAMESPACE = "/nepi/s2x/"
PTX_NAMESPACE = NEPI_BASE_NAMESPACE + "iqr_pan_tilt/ptx/"

# NavPose Source Topics
NAVPOSE_SOURCE_ORIENTATION_TOPIC =PTX_NAMESPACE + "odom"

### Setup NEPI NavPose Settings Topic Namespaces
NEPI_SET_NAVPOSE_ORIENTATION_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose_mgr/set_orientation_topic"

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
  # Update Orientation source
  print("Waiting for topic: " + NAVPOSE_SOURCE_ORIENTATION_TOPIC)
  wait_for_topic(NAVPOSE_SOURCE_ORIENTATION_TOPIC)
  set_orientation_pub = rospy.Publisher(NEPI_SET_NAVPOSE_ORIENTATION_TOPIC, String, queue_size=1)
  time.sleep(1) # Wait between creating and using publisher
  set_orientation_pub.publish(NAVPOSE_SOURCE_ORIENTATION_TOPIC)
  print("Orientation Topic Set to: " + NAVPOSE_SOURCE_ORIENTATION_TOPIC)

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
  rospy.loginfo("Starting PanTilt PTX NavPose Config Script")
  rospy.init_node("pantilt_ptx_navpose_config_script")
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

