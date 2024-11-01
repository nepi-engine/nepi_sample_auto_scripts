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
# 1. Connect NEPI NavPose topics to appropriate RBX driver topics
# 2. Option to synchronization NEPI's Clock to NavPose source time

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

# Set NEPI NavPose Source Topics Names, or Enter "" to Ignore
NEPI_NAVPOSE_SOURCE_GPS_TOPIC = "rbx/gps_fix"  # Enter "" to Ignore
NEPI_NAVPOSE_SOURCE_ODOM_TOPIC = "rbx/odom" # Enter "" to Ignore
NEPI_NAVPOSE_SOURCE_HEADING_TOPIC = "rbx/heading" # Enter "" to Ignore

SYNC_NEPI_CLOCK = True # Set to false to disable GPS clock sync


#########################################
# Node Class
#########################################

class navpose_config(object):

  #######################
  ### Node Initialization
  DEFAULT_NODE_NAME = "navpose_config" # Can be overwitten by luanch command
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
      nepi_msg.publishMsgInfo(self,"Waiting for topic: " + NEPI_NAVPOSE_SOURCE_GPS_TOPIC)
      nepi_ros.wait_for_topic(NEPI_NAVPOSE_SOURCE_GPS_TOPIC)
      self.set_gps_pub = rospy.Publisher(NEPI_SET_NAVPOSE_GPS_TOPIC, String, queue_size=1)
      nepi_msg.publishMsgInfo(self,"Waiting for topic: " + NEPI_NAVPOSE_SOURCE_GPS_TOPIC)
      self.gps_topic = nepi_ros.wait_for_topic(NEPI_NAVPOSE_SOURCE_GPS_TOPIC)
    # Set Orientation Topic
    if NEPI_NAVPOSE_SOURCE_ODOM_TOPIC != "":
      # Update Orientation source
      nepi_msg.publishMsgInfo(self,"Waiting for topic: " + NEPI_NAVPOSE_SOURCE_ODOM_TOPIC)
      nepi_ros.wait_for_topic(NEPI_NAVPOSE_SOURCE_ODOM_TOPIC)
      self.set_orientation_pub = rospy.Publisher(NEPI_SET_NAVPOSE_ORIENTATION_TOPIC, String, queue_size=1)
      nepi_msg.publishMsgInfo(self,"Waiting for topic: " + NEPI_NAVPOSE_SOURCE_ODOM_TOPIC)
      self.odom_topic = nepi_ros.wait_for_topic(NEPI_NAVPOSE_SOURCE_ODOM_TOPIC)
    # Heading Topic
    if NEPI_NAVPOSE_SOURCE_HEADING_TOPIC != "":
      # Update Heading source
      nepi_msg.publishMsgInfo(self,"Waiting for topic: " + NEPI_NAVPOSE_SOURCE_HEADING_TOPIC)
      nepi_ros.wait_for_topic(NEPI_NAVPOSE_SOURCE_HEADING_TOPIC)
      self.set_heading_pub = rospy.Publisher(NEPI_SET_NAVPOSE_HEADING_TOPIC, String, queue_size=1)
      nepi_msg.publishMsgInfo(self,"Waiting for topic: " + NEPI_NAVPOSE_SOURCE_HEADING_TOPIC)
      self.heading_topic = nepi_ros.wait_for_topic(NEPI_NAVPOSE_SOURCE_HEADING_TOPIC)
    ##############################
    # Sync NEPI clock to GPS timestamp
    set_gps_timesync_pub = rospy.Publisher(NEPI_ENABLE_NAVPOSE_GPS_CLOCK_SYNC_TOPIC, Bool, queue_size=1)
    nepi_ros.sleep(1,10) # Wait between creating and using publisher
    set_gps_timesync_pub.publish(data=SYNC_NEPI_CLOCK)
    nepi_msg.publishMsgInfo(self,"GPS Clock Sync Topic Set to: " + str(SYNC_NEPI_CLOCK))
    nepi_msg.publishMsgInfo(self,"Setup complete")
    ## Create Class Sevices
    ## Create Class Publishers
    ## Start Class Subscribers
    ## Start Node Processes
    nepi_msg.publishMsgInfo(self,"Starting set navpose topics timer callback")
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
      self.set_gps_pub.publish(self.gps_topic)
      nepi_msg.publishMsgInfo(self,"GPS Topic Set to: " + self.gps_topic)
    if NEPI_NAVPOSE_SOURCE_ODOM_TOPIC != "":
      # Set Orientation Topic
      self.set_orientation_pub.publish(self.odom_topic)
      nepi_msg.publishMsgInfo(self,"Orientation Topic Set to: " + self.odom_topic)
    if NEPI_NAVPOSE_SOURCE_ODOM_TOPIC != "":
      # Set Heading Topic
      self.set_heading_pub.publish(self.heading_topic)
      nepi_msg.publishMsgInfo(self,"Heading Topic Set to: " + self.heading_topic)

    
    #######################
    # Node Cleanup Function
  
  def cleanup_actions(self):
    nepi_msg.publishMsgInfo(self,"Shutting down: Executing script cleanup actions")


#########################################
# Main
#########################################
if __name__ == '__main__':
  navpose_config()

