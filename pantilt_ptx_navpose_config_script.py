#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

# Sample NEPI Config Script.
# 1. Connects NEPI NavPose topics to appropriate navpose topics


import rospy
import time
import sys
from nepi_edge_sdk_base import nepi_ros 
from nepi_edge_sdk_base import nepi_msg

from std_msgs.msg import String, Bool
from nepi_ros_interfaces.srv import NavPoseQuery, NavPoseQueryRequest


#########################################
# USER SETTINGS - Edit as Necessary 
#########################################

CONNECT_PTX_HEADING = True # Set to True to configure Heading to PTX Heading output
SYNC_NEPI_CLOCK = False
#########################################
# ROS NAMESPACE SETUP
#########################################


NEPI_PTX_NAMESPACE = self.base_namespace + "iqr_pan_tilt/ptx/"

# NavPose Source Topics
NEPI_NAVPOSE_SOURCE_GPS_TOPIC = ""  # Enter "" to Ignore
NEPI_NAVPOSE_SOURCE_ODOM_TOPIC = NEPI_PTX_NAMESPACE + "odom"
if CONNECT_PTX_HEADING:
  NEPI_NAVPOSE_SOURCE_HEADING_TOPIC = NEPI_PTX_NAMESPACE + "heading"
else:
  NEPI_NAVPOSE_SOURCE_HEADING_TOPIC = ""  # Enter "" to Ignore

#########################################
# Node Class
#########################################

class pantilt_navpose_config(object):

  #######################
  ### Node Initialization
  DEFAULT_NODE_NAME = "pantilt_navpose_config" # Can be overwitten by luanch command
  def __init__(self):
    #### APP NODE INIT SETUP ####
    nepi_ros.init_node(name= self.DEFAULT_NODE_NAME)
    self.node_name = nepi_ros.get_node_name()
    self.base_namespace = nepi_ros.get_base_namespace()
    nepi_msg.createMsgPublishers(self)
    nepi_msg.publishMsgInfo(self,"Starting Initialization Processes")
    ##############################
    ## Initialize Class Variables
    ## Define Class Namespaces
    NEPI_SET_NAVPOSE_GPS_TOPIC = self.base_namespace + "nav_pose_mgr/set_gps_fix_topic"
    NEPI_SET_NAVPOSE_HEADING_TOPIC = self.base_namespace + "nav_pose_mgr/set_heading_topic"
    NEPI_SET_NAVPOSE_ORIENTATION_TOPIC = self.base_namespace + "nav_pose_mgr/set_orientation_topic"
    NEPI_ENABLE_NAVPOSE_GPS_CLOCK_SYNC_TOPIC = self.base_namespace + "nav_pose_mgr/enable_gps_clock_sync"
    ## Define Class Services Calls
    # GPS Topic
    if NEPI_NAVPOSE_SOURCE_GPS_TOPIC != "":
      # Update Global Location source
      print("Waiting for topic: " + NEPI_NAVPOSE_SOURCE_GPS_TOPIC)
      nepi_ros.wait_for_topic(NEPI_NAVPOSE_SOURCE_GPS_TOPIC)
      self.set_gps_pub = rospy.Publisher(NEPI_SET_NAVPOSE_GPS_TOPIC, String, queue_size=1)
    # Set Orientation Topic
    if NEPI_NAVPOSE_SOURCE_ODOM_TOPIC != "":
      # Update Orientation source
      print("Waiting for topic: " + NEPI_NAVPOSE_SOURCE_ODOM_TOPIC)
      nepi_ros.wait_for_topic(NEPI_NAVPOSE_SOURCE_ODOM_TOPIC)
      self.set_orientation_pub = rospy.Publisher(NEPI_SET_NAVPOSE_ORIENTATION_TOPIC, String, queue_size=1)
    # Heading Topic
    if NEPI_NAVPOSE_SOURCE_HEADING_TOPIC != "":
      # Update Heading source
      print("Waiting for topic: " + NEPI_NAVPOSE_SOURCE_HEADING_TOPIC)
      nepi_ros.wait_for_topic(NEPI_NAVPOSE_SOURCE_HEADING_TOPIC)
      self.set_heading_pub = rospy.Publisher(NEPI_SET_NAVPOSE_HEADING_TOPIC, String, queue_size=1)
    ##############################
    # Sync NEPI clock to GPS timestamp
    set_gps_timesync_pub = rospy.Publisher(NEPI_ENABLE_NAVPOSE_GPS_CLOCK_SYNC_TOPIC, Bool, queue_size=1)
    nepi_ros.sleep(1,10) # Wait between creating and using publisher
    set_gps_timesync_pub.publish(data=SYNC_NEPI_CLOCK)
    print("GPS Clock Sync Topic Set to: " + str(SYNC_NEPI_CLOCK))
    print("Setup complete")
    ## Create Class Sevices
    ## Create Class Publishers
    ## Start Class Subscribers
    ## Start Node Processes
    print("Starting set navpose topics timer callback")
    rospy.Timer(rospy.Duration(5.0), self.set_nepi_navpose_topics_callback)

    ##############################
    ## Initiation Complete
    nepi_msg.publishMsgInfo(self," Initialization Complete")
    # Spin forever (until object is detected)
    rospy.spin()
    ##############################

  #######################
  ### Node Methods


  ### Callback to set NEPI navpose topics
  def set_nepi_navpose_topics_callback(self,timer):
    if NEPI_NAVPOSE_SOURCE_GPS_TOPIC != "":
      # Set GPS Topic
      self.set_gps_pub.publish(NEPI_NAVPOSE_SOURCE_GPS_TOPIC)
      print("GPS Topic Set to: " + NEPI_NAVPOSE_SOURCE_GPS_TOPIC)
    if NEPI_NAVPOSE_SOURCE_ODOM_TOPIC != "":
      # Set Orientation Topic
      self.set_orientation_pub.publish(NEPI_NAVPOSE_SOURCE_ODOM_TOPIC)
      print("Orientation Topic Set to: " + NEPI_NAVPOSE_SOURCE_ODOM_TOPIC)
    if NEPI_NAVPOSE_SOURCE_ODOM_TOPIC != "":
      # Set Heading Topic
      self.set_heading_pub.publish(NEPI_NAVPOSE_SOURCE_HEADING_TOPIC)
      print("Heading Topic Set to: " + NEPI_NAVPOSE_SOURCE_HEADING_TOPIC)

    
    #######################
    # Node Cleanup Function
  
  def cleanup_actions(self):
    nepi_msg.publishMsgInfo(self,"Shutting down: Executing script cleanup actions")


#########################################
# Main
#########################################
if __name__ == '__main__':
  pantilt_navpose_config()