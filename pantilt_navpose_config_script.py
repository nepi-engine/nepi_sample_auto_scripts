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
# Uses onboard ROS libraries to
# 1. Connect NEPI NavPose topics to appropriate navpose topics


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
# ROS namespace setup
NEPI_BASE_NAMESPACE = "/nepi/s2x/"
PTX_NAMESPACE = NEPI_BASE_NAMESPACE + "iqr_pan_tilt/ptx/"

# NavPose Source Topics
NAVPOSE_SOURCE_ORIENTATION_TOPIC =PTX_NAMESPACE + "odom"

### Setup NEPI NavPose Settings Topic Namespaces
NEPI_SET_NAVPOSE_ORIENTATION_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose_mgr/set_orientation_topic"

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
  print("")
  print("Starting Initialization")
  print("Initialization Complete")


### Callback to set NEPI navpose topics
def set_nepi_navpose_topics_callback(timer):
  # Update Orientation source
  print("Waiting for topic: " + NAVPOSE_SOURCE_ORIENTATION_TOPIC)
  wait_for_topic(NAVPOSE_SOURCE_ORIENTATION_TOPIC, 'nav_msgs/Odometry')
  set_orientation_pub = rospy.Publisher(NEPI_SET_NAVPOSE_ORIENTATION_TOPIC, String, queue_size=1)
  time.sleep(.1) # Wait between creating and using publisher
  set_orientation_pub.publish(NAVPOSE_SOURCE_ORIENTATION_TOPIC)
  print("Orientation Topic Set to: " + NAVPOSE_SOURCE_ORIENTATION_TOPIC)

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

