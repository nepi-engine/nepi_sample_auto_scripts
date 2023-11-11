#!/usr/bin/env python

# Sample NEPI Automation Script. 
# Uses onboard ROS python library to
# 1. Waits for snapshot_event topic message
# 2. Start saving data to onboard storage
# 3. Delay a specified amount of time, then stop saving
# 4. Delay a next trigger for some set delay time

import time
import sys
import rospy   

from std_msgs.msg import UInt8, Empty, String, Bool
from nepi_ros_interfaces.msg import IDXStatus, SaveData, SaveDataRate, StringArray


###########################################################################
# SETUP - Edit as Necessary 
###########################################################################

###!!!!!!!! Set Automation action parameters !!!!!!!!
SAVE_DURATION_S = 5.0 # Seconds. Length of time to save data
SAVE_DATA_RATE_HZ = 1.0
SAVE_RESET_DELAY_S = 5.0 # Seconds. Delay before starting over search/save process

BASE_NAMESPACE = "/nepi/s2x/"
SAVE_DATA_TOPIC = BASE_NAMESPACE + "save_data"
SNAPSHOT_TOPIC = BASE_NAMESPACE + "snapshot_event"

#####################################################################################
# Globals
#####################################################################################
save_data_pub = rospy.Publisher(SAVE_DATA_TOPIC, SaveData, queue_size=10)
#####################################################################################
# Methods
#####################################################################################

### System Initialization processes
def initialize_actions():
  print("Initialization Complete")
  print("Waiting for snapshot event trigger")

# Action upon detection of snapshot event trigger
def snapshot_event_callback(event):
  global save_data_pub
  save_data_pub.publish(save_continuous=True, save_raw=False)
  print("Saving for " + str(SAVE_DURATION_S) + " secs")
  time.sleep(SAVE_DURATION_S)
  print("Disabling data saving")
  save_data_pub.publish(save_continuous=False, save_raw=False)
  # Delay next trigger
  print("Delaying next trigger for " + str(SAVE_RESET_DELAY_S) + " secs")
  time.sleep(SAVE_RESET_DELAY_S)
  print("Waiting for next " + str(SNAPSHOT_TOPIC) + ' event trigger')

### Cleanup processes on node shutdown
def cleanup_actions():
  global save_data_pub
  print("Shutting down: Executing script cleanup actions")
  # Disabling data saving 
  save_data_pub.publish(save_continuous=False, save_raw=False)
  # Restoring data save setup defaults
  time.sleep(1)

### Script Entrypoint
def startNode():
  rospy.loginfo("Starting Snapshot Event Detect and Save automation script", disable_signals=True) # Disable signals so we can force a shutdown
  rospy.init_node(name="snapshot_event_detect_save_auto_script")
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

