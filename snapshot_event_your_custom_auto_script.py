#!/usr/bin/env python


__author__ = "Jason Seawall"
__copyright__ = "Copyright 2023, Numurus LLC"
__email__ = "nepi@numurus.com"
__credits__ = ["Jason Seawall", "Josh Maximoff"]

__license__ = "GPL"
__version__ = "2.0.4.0"

# Sample NEPI Automation Script. 
# Uses onboard ROS python library to
# 1. Waits for snapshot event trigger
# 2. Runs your custom snapshot event actions
# 3. Delays next trigger event action for some set delay time

import time
import sys
import rospy
import os
import numpy as np

from std_msgs.msg import UInt8, Empty, String, Bool


###########################################################################
# SETUP - Edit as Necessary 
###########################################################################

###!!!!!!!! Set Automation action parameters !!!!!!!!
TIGGER_RESET_DELAY_S = 5.0 # Seconds. Delay before starting over search/save process

# NEPI ROS namespace setup
NEPI_BASE_NAMESPACE = "/nepi/s2x/"
### Snapshot Topic Name
SNAPSHOT_TOPIC = NEPI_BASE_NAMESPACE + "snapshot_event"

#####################################################################################
# Globals
#####################################################################################


#####################################################################################
# Methods
#####################################################################################

### System Initialization processes
def initialize_actions():
  print("")
  print("Starting Initialization")  
  time.sleep(.1)
  print("Initialization Complete")
  print("Waiting for snapshot event trigger topic to publish on:")
  print(SNAPSHOT_TOPIC)
  

# Action upon detection of snapshot event trigger
def snapshot_event_callback(event):
  ###########################################################
  ### ADD YOUR CODE HERE
  ###########################################################

  ###########################################################
  ### END OF YOUR CODE
  ###########################################################
  print("Delaying next trigger for " + str(TIGGER_RESET_DELAY_S) + " secs")
  time.sleep(TIGGER_RESET_DELAY_S)
  print("Waiting for next snapshot event trigger")

### Cleanup processes on node shutdown
def cleanup_actions():
  time.sleep(.1)

### Script Entrypoint
def startNode():
  rospy.loginfo("Starting Snapshot Event automation script", disable_signals=True) # Disable signals so we can force a shutdown
  rospy.init_node(name="snapshot_event_auto_script")
  # Run Initialization processes
  initialize_actions()
  # Set up snapshot event callback
  rospy.Subscriber(SNAPSHOT_TOPIC, Empty, snapshot_event_callback, queue_size = 1)
  #Set up cleanup on node shutdown
  rospy.on_shutdown(cleanup_actions)
  # Spin forever (until object is detected)
  rospy.spin()


#####################################################################################
# Main
#####################################################################################

if __name__ == '__main__':
  startNode()

