#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#


# Sample NEPI Action Script. 
# Uses onboard ROS python library to
# 1. Confirms IDX driver supported camera topic is publishing
# 2. Waits for event_event topic message
# 3. Captures and sends latest data to cloud
# 4. Delay a next trigger for some set delay time

import time
import sys
import rospy
from nepi_edge_sdk_base import nepi_ros 

from std_msgs.msg import UInt8, Empty, String, Bool
from nepi_ros_interfaces.msg import IDXStatus, SaveData, SaveDataRate, StringArray


#########################################
# USER SETTINGS - Edit as Necessary 
#########################################

#Configure Your Connect Topics to Send in NEPI RUI

###!!!!!!!! Set Automation action parameters !!!!!!!!
TIGGER_RESET_DELAY_S = 30.0 # Seconds. Delay before starting over

#########################################
# ROS NAMESPACE SETUP
#########################################

# ROS namespace setup
NEPI_BASE_NAMESPACE = nepi_ros.get_base_namespace()

#########################################
# Node Class
#########################################

class snapshot_triggered_send_to_cloud_action(object):

  #######################
  ### Node Initialization
  def __init__(self):
    rospy.loginfo("Starting Initialization Processes")
    ## Initialize Class Variables
    NEPI_LINK_NAMESPACE = NEPI_BASE_NAMESPACE + "nepi_link_ros_bridge/"
    NEPI_LINK_ENABLE_TOPIC = NEPI_LINK_NAMESPACE + "enable"
    NEPI_LINK_SET_DATA_SOURCES_TOPIC = NEPI_LINK_NAMESPACE + "lb/select_data_sources"
    NEPI_LINK_COLLECT_DATA_TOPIC = NEPI_LINK_NAMESPACE + "lb/create_data_set_now"
    NEPI_LINK_CONNECT_TOPIC = NEPI_LINK_NAMESPACE + "connect_now"
    ## Define Class Namespaces
    EVENT_TOPIC = NEPI_BASE_NAMESPACE + "snapshot_trigger"
    ## Define Class Services Calls
    ## Create Class Sevices    
    ## Create Class Publishers
    self.nepi_link_enable_pub = rospy.Publisher(NEPI_LINK_ENABLE_TOPIC, Bool, queue_size=10)
    self.nepi_link_set_data_sources = rospy.Publisher(NEPI_LINK_SET_DATA_SOURCES_TOPIC, StringArray, queue_size=10)
    self.nepi_link_collect_data_pub = rospy.Publisher(NEPI_LINK_COLLECT_DATA_TOPIC, Empty, queue_size=10)
    self.nepi_link_connect_now_pub = rospy.Publisher(NEPI_LINK_CONNECT_TOPIC, Empty, queue_size=10)
    ## Start Class Subscribers
    # Set up event event callback
    rospy.Subscriber(EVENT_TOPIC, Empty, self.event_event_callback, queue_size = 1)
    rospy.loginfo("Subscribed to : " + EVENT_TOPIC)
    ## Start Node Processes
    ## Initiation Complete
    rospy.loginfo("Initialization Complete")

  #######################
  ### Node Methods
  
  # Action upon detection of event event trigger
  def event_event_callback(self,event):
    rospy.loginfo("Starting data collection for NEPI CONNECT")
    self.nepi_link_collect_data_pub.publish()
    nepi_ros.sleep(1,10)
    rospy.loginfo("Kicking off NEPI CONNECT cloud connection")
    self.nepi_link_connect_now_pub.publish()
    nepi_ros.sleep(1,10)
    rospy.loginfo("Waiting for next event event trigger")
    rospy.loginfo("Delaying next trigger for " + str(TIGGER_RESET_DELAY_S) + " secs")
    nepi_ros.sleep(TIGGER_RESET_DELAY_S,100)
    rospy.loginfo("Waiting for next event event trigger")
  
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







