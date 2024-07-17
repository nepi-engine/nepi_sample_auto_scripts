#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#


# Sample NEPI Process Script.
# 1. Test sensor save functions

import os
import time
import sys
import rospy
import numpy as np
from nepi_edge_sdk_base import nepi_ros
from nepi_edge_sdk_base import nepi_nex
from std_msgs.msg import Empty, String, Bool
from nepi_ros_interfaces.msg import IDXStatus, SaveData, SaveDataRate, SaveDataStatus


#########################################
# USER SETTINGS - Edit as Necessary 
#########################################

# IDX Sensor Name or partial topic to Find and Use
SENSOR_NAME = "color_2d_image"

SAVE_TIME_SEC = 5

SAVE_RATE_UPDATE_DICT = dict(color_2d_image = 2,
bw_2d_image=2,
depth_map=2,
depth_image=2,
pointcloud_image=2,
pointcloud=2)

SAVE_PREFIX_UPDATE = "Test_Folder/Test_Data"

SAVE_NAV_ENABLE = True
SAVE_NAV_RATE_HZ = 2


#########################################
# ROS NAMESPACE SETUP
#########################################

# ROS namespace setup
NEPI_BASE_NAMESPACE = nepi_ros.get_base_namespace()

#########################################
# Node Class
#########################################

class imaging_sensor_update_save_config(object):
  status_msg = None
  status_msg_last = None
  save_update_complete = False
  #######################
  ### Node Initialization
  def __init__(self):
    rospy.loginfo("Starting Initialization Processes")
    ## Initialize Class Variables
    ## Define Class Namespaces
    # Search for sensor topic and get base namespace
    #print(nepi_ros.get_topic_list())
    rospy.loginfo("Searching for topic name: " + SENSOR_NAME)
    sensor_topic=nepi_ros.wait_for_topic(SENSOR_NAME)
    sensor_basename = sensor_topic.split('idx')[0]  
    rospy.loginfo("Found basename: " + sensor_basename)
    ## Wait for Services and Topics
    save_status_topic = nepi_ros.wait_for_topic(sensor_basename + "save_data_status")
    save_rate_topic = sensor_basename + "save_data_rate" 
    save_enable_topic = sensor_basename + "save_data"
    save_prefix_topic = sensor_basename + "save_data_prefix"
    save_reset_topic = sensor_basename + "save_data_reset" 
    save_reset_factory_topic = sensor_basename + "save_data_reset_factory"

    ## Define Class Services Calls
    ## Create Class Sevices    
    # Get Capabilities Settings List
    # Create Class Subscribers
    print("Starting idx status subscriber")
    rospy.Subscriber(save_status_topic, SaveDataStatus, self.update_status_callback, queue_size = 1)
    # Create Class Publishers
    

    print("Creating save update publishers")
    self.save_rate_pub = rospy.Publisher(save_rate_topic, SaveDataRate, queue_size=10)
    self.save_prefix_pub = rospy.Publisher(save_prefix_topic, String, queue_size=10)
    self.save_pub = rospy.Publisher(save_enable_topic, SaveData, queue_size=10)
    self.save_reset_pub = rospy.Publisher(save_reset_topic, Empty, queue_size=10)

    self.save_nav_rate_pub = rospy.Publisher(NEPI_BASE_NAMESPACE + 'nav_pose_mgr/save_data_rate', SaveDataRate, queue_size=1)
    self.save_nav_prefix_pub = rospy.Publisher(NEPI_BASE_NAMESPACE + 'nav_pose_mgr/save_data_prefix', String, queue_size=1)
    self.save_nav_pub = rospy.Publisher(NEPI_BASE_NAMESPACE + 'nav_pose_mgr/save_data', SaveData, queue_size=1)

    ## Start Node Processes
    # Start a timed settings update publish callback
    rospy.Timer(rospy.Duration(1), self.save_config_update_callback)
    ## Initiation Complete
    rospy.loginfo("Initialization Complete")

  #######################
  ### Node Methods

  def update_status_callback(self,status_msg):
    if self.status_msg_last == None or self.status_msg_last != status_msg:
      print("")
      print("******************")
      print("Received staus update msg")
      print(status_msg)
      print("******************")
      print("")
      self.status_msg_last = status_msg


  ### Publish Messages
  def save_config_update_callback(self,timer):
    if self.status_msg_last == None:
      print("Waiting for valid settings_list from status before sending settings update")
    else:
      if not rospy.is_shutdown() and self.save_update_complete is False:
        print("Sending save rate update messages")
        saveRateMsg = SaveDataRate()
        for data_product in SAVE_RATE_UPDATE_DICT.keys():
          data_rate = SAVE_RATE_UPDATE_DICT[data_product]
          saveRateMsg.data_product = data_product
          saveRateMsg.save_rate_hz = SAVE_RATE_UPDATE_DICT[data_product]
          self.save_rate_pub.publish(saveRateMsg)
        if SAVE_NAV_ENABLE:
          saveRateMsg.data_product = "nav_pose"
          saveRateMsg.save_rate_hz = SAVE_NAV_RATE_HZ
          self.save_nav_rate_pub.publish(saveRateMsg)

        print("Sending save prefix update messages")
        savePrefixMsg = SAVE_PREFIX_UPDATE
        self.save_prefix_pub.publish(savePrefixMsg)
        if SAVE_NAV_ENABLE:
          self.save_nav_prefix_pub.publish(savePrefixMsg)
        nepi_ros.sleep(2,20)

        print("Start Saving Data")
        save_msg = SaveData()
        save_msg.save_continuous = True
        save_msg.save_raw = False

        self.save_pub.publish(save_msg)
        if SAVE_NAV_ENABLE:
          self.save_nav_pub.publish(save_msg)
        nepi_ros.sleep(SAVE_TIME_SEC,100)

        print("Stop Saving Data")
        save_msg.save_continuous = False
        self.save_pub.publish(save_msg)
        if SAVE_NAV_ENABLE:
          self.save_nav_pub.publish(save_msg)
        nepi_ros.sleep(2,20)

        print("Reset Save Settings")
        self.save_reset_pub.publish(Empty())
        nepi_ros.sleep(2,20)

        print("Save Update Processes Complete")
        self.save_update_complete = True





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
  #Set up node shutdown
  rospy.on_shutdown(node.cleanup_actions)
  # Spin forever (until object is detected)
  rospy.spin()


