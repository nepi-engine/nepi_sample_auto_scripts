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

# Sample NEPI Action Script. 
# Uses onboard ROS python library to
# 1. Waits for snapshot event trigger
# 2. Runs your custom snapshot event actions
# 3. Delays next trigger event action for some set delay time

import time
import sys
import rospy
import os
import numpy as np
from resources import nepi

from std_msgs.msg import UInt8, Empty, String, Bool


#########################################
# USER SETTINGS - Edit as Necessary 
#########################################

###!!!!!!!! Set Automation action parameters !!!!!!!!
TIGGER_RESET_DELAY_S = 5.0 # Seconds. Delay before starting over search/save process


#########################################
# ROS NAMESPACE SETUP
#########################################

# ROS namespace setup
NEPI_BASE_NAMESPACE = "/nepi/s2x/"

#########################################
# Node Class
#########################################

class snapshot_event_your_custom_action(object):

  #######################
  ### Node Initialization
  def __init__(self):
    rospy.loginfo("Starting Initialization Processes")
    ## Initialize Class Variables
    ## Define Class Namespaces
    SNAPSHOT_TOPIC = NEPI_BASE_NAMESPACE + "snapshot_event"
    ## Define Class Services Calls
    ## Create Class Sevices    
    ## Create Class Publishers
    ## Start Class Subscribers
    # Set up snapshot event callback
    rospy.Subscriber(SNAPSHOT_TOPIC, Empty, self.snapshot_event_callback, queue_size = 1)
    rospy.loginfo("Subscribed to : " + SNAPSHOT_TOPIC)
    ## Start Node Processes
    ## Initiation Complete
    rospy.loginfo("Initialization Complete")

  #######################
  ### Node Methods
  
  # Action upon detection of snapshot event trigger
  def snapshot_event_callback(self,event):
    ###########################################################
    ### ADD YOUR CODE HERE
    ###########################################################

    ###########################################################
    ### END OF YOUR CODE
    ###########################################################
    rospy.loginfo("Delaying next trigger for " + str(TIGGER_RESET_DELAY_S) + " secs")
    time.sleep(TIGGER_RESET_DELAY_S)
    rospy.loginfo("Waiting for next snapshot event trigger")
  
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



