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
# 1. Test new sensor settings functions

import os
import time
import sys
import rospy
import numpy as np
from nepi_edge_sdk_base import nepi_ros
from std_msgs.msg import Empty, String, Bool, Int32, Float32
from nepi_ros_interfaces.msg import SettingUpdate
from nepi_ros_interfaces.srv import SettingsCapabilitiesQuery, SettingsCapabilitiesQueryResponse

#########################################
# USER SETTINGS - Edit as Necessary 
#########################################

# IDX Sensor Name or Partial Topic String to Find and Use
SENSOR_NAME = "color_2d_image"

Update_Rate_Hz = 0.5 # Send update every 2 second

# Replace with actual setting values for your imaging device
SETTINGS_UPDATE_LIST = [["Discrete","TestDiscrete","Option_3"],
  			["String","TestString","NewString"],
  			["Bool","TestBool","False"],
  			["Int","TestInt","500"],
  			["Float","TestFloat","9.81"]] 


#########################################
# ROS NAMESPACE SETUP
#########################################

# ROS namespace setup
NEPI_BASE_NAMESPACE = nepi_ros.get_base_namespace()

#########################################
# Node Class
#########################################

class imaging_sensor_update_settings_config(object):
  settings_status_msg = None
  settings_status_msg = None
  last_settings_status_msg = None
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
    rospy.loginfo("Found basename: " + sensor_topic)
    ## Wait for Services and Topics
    cap_service = nepi_ros.wait_for_service(sensor_basename + "settings_capabilities_query")
    status_topic = nepi_ros.wait_for_topic(sensor_basename + "settings_status")
    update_settings_topic = nepi_ros.wait_for_topic(sensor_basename + "update_setting") 
    
    ## Define Class Services Calls
    ## Create Class Sevices    
    # Get Capabilities Settings List
    get_cap_service = rospy.ServiceProxy(cap_service, SettingsCapabilitiesQuery)
    cap_response = get_cap_service()
    #print(cap_response)

       
    #print("**************")
    print("")
    print("Received Cap Message Data")
    self.cap_settings = cap_response.settings_options
    print(self.cap_settings)
    cap_lists = nepi_ros.parse_settings_msg_data(self.cap_settings)
    print("")
    print("Parsed cap List")
    for setting in cap_lists:
      print(setting)
    print("")

    print("Getting Alphatized cap List")
    sorted_cap_lists = nepi_ros.sort_settings_alphabetically(cap_lists)
    for setting in sorted_cap_lists:
      print(setting)    

    print("")
    print("Getting Caps Lists by Type")
    types = nepi_ros.SETTING_TYPES
    for type in types:
      type_cap_lists = nepi_ros.get_settings_by_type(sorted_cap_lists,type)
      print("")
      print("Cap Lists by type " + type)
      for setting in type_cap_lists:
        print(setting)


    print("")
    print("**************")
    
    # Create Class Subscribers
    print("Starting idx status subscriber")
    rospy.Subscriber(status_topic, String, self.update_status_callback, queue_size = 1)
    # Create Class Publishers
    
    print("Publishing updates to topic: " + update_settings_topic)
    self.setting_update_pub = rospy.Publisher(update_settings_topic, SettingUpdate, queue_size=10)


    ## Start Node Processes
    # Start a timed settings update publish callback
    pub_interval_sec = float(1)/Update_Rate_Hz
    print("PublishingD at " + str(Update_Rate_Hz) + " Hz")
    rospy.Timer(rospy.Duration(pub_interval_sec), self.settings_update_callback)
    ## Initiation Complete
    rospy.loginfo("Initialization Complete")

  #######################
  ### Node Methods

  ### Publish Messages
  def settings_update_callback(self,timer):
    if self.settings_status_msg == None:
      print("Waiting for valid settings_list from status before sending settings update")
    else:
      if not rospy.is_shutdown():
        print("")
        print("**************")
        print("Update Settings List:")
        settings_update_list = SETTINGS_UPDATE_LIST
        changed_settings_list = []
        print("Checking for settings changes")
        for setting in settings_update_list:
          [name_match,type_match,value_match] = nepi_ros.compare_setting_in_settings(setting,self.settings_status_msg)
          if not value_match: # name_match would be true for value_match to be true
            rospy.loginfo("Sending setting update " + str(setting) )
            setting_update = SettingUpdate()
            setting_update.type_str = setting[0]
            setting_update.name_str = setting[1]
            setting_update.value_str = setting[2]
            self.setting_update_pub.publish(setting_update) 
          else:
            rospy.loginfo("Skipping setting update " + str(setting) + " because it matches current setting")
         



  def update_status_callback(self,status_msg):
    print("")
    print("******************")
    print("Received staus update msg")
    print(status_msg)
    self.settings_status_msg = status_msg.data
    if self.last_settings_status_msg != self.settings_status_msg:
      print(self.settings_status_msg)
      self.settings_status_msg = nepi_ros.parse_settings_msg_data(self.settings_status_msg)

      if self.settings_status_msg == None:
        print("No settings in received status")
      else:
        print("")
        print("Received Settings From Satus Msg")
        print(self.settings_status_msg)

        print("")
        print("Getting alphatized state List")
        sorted_settings= nepi_ros.sort_settings_alphabetically(self.settings_status_msg)
        print(sorted_settings)      

        print("")
        print("Getting State Lists by Type")
        types = nepi_ros.SETTING_TYPES
        for type in types:
          type_settings_list = nepi_ros.get_settings_by_type(sorted_settings,type)
          print("")
          print("Settings filtered by type " + type)
          print(type_settings_list)

        print("")
        print("Get Settings Values")
        for setting in sorted_settings:
          ret = nepi_ros.get_data_from_setting(setting)
          print(ret)
      print("******************")
      print("")
      self.last_settings_status_msg = self.settings_status_msg

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


