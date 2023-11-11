#!/usr/bin/env python

# Sample NEPI Automation Script. 
# Uses onboard ROS python library to
# 1. Confirms IDX driver supported camera topic is publishing
# 2. Waits for snapshot_event topic message
# 3. Captures and sends latest data to cloud
# 4. Delay a next trigger for some set delay time

import time
import sys
import rospy   

from std_msgs.msg import UInt8, Empty, String, Bool
from nepi_ros_interfaces.msg import IDXStatus, SaveData, SaveDataRate, StringArray


#############################################################
# SETUP - Edit as Necessary 
#############################################################

###!!!!!!!! Set Automation action parameters !!!!!!!!
SEND_RESET_DELAY_S = 5.0 # Seconds. Delay before starting over

BASE_NAMESPACE = "/nepi/s2x/"
CAMERA_NAME = "nexigo_n60_fhd_webcam_audio/"
IMAGE_TO_SEND_TOPIC = BASE_NAMESPACE + CAMERA_NAME + "idx/color_2d_image"

# NEPI Link data topics and parameters
NEPI_LINK_NAMESPACE = BASE_NAMESPACE + "nepi_link_ros_bridge/"
NEPI_LINK_ENABLE_TOPIC = NEPI_LINK_NAMESPACE + "enable"
NEPI_LINK_SET_DATA_SOURCES_TOPIC = NEPI_LINK_NAMESPACE + "lb/select_data_sources"
NEPI_LINK_COLLECT_DATA_TOPIC = NEPI_LINK_NAMESPACE + "lb/create_data_set_now"
NEPI_LINK_CONNECT_TOPIC = NEPI_LINK_NAMESPACE + "connect_now"

### Snapshot Topic Name
SNAPSHOT_TOPIC = BASE_NAMESPACE + "snapshot_event"

#############################################################
# Globals
#############################################################
nepi_link_enable_pub = rospy.Publisher(NEPI_LINK_ENABLE_TOPIC, Bool, queue_size=10)
nepi_link_set_data_sources = rospy.Publisher(NEPI_LINK_SET_DATA_SOURCES_TOPIC, StringArray, queue_size=10)
nepi_link_collect_data_pub = rospy.Publisher(NEPI_LINK_COLLECT_DATA_TOPIC, Empty, queue_size=10)
nepi_link_connect_now_pub = rospy.Publisher(NEPI_LINK_CONNECT_TOPIC, Empty, queue_size=10)


#############################################################
# Methods
#############################################################

### System Initialization processes
def initialize_actions():
  print("Setting NEPI CONNECT data sources")
  nepi_link_set_data_sources.publish([IMAGE_TO_SEND_TOPIC])
  print("Waiting for a snapshot event trigger")


# Action upon detection of snapshot event trigger
def snapshot_event_callback(event):
  print("Starting data collection for NEPI CONNECT")
  nepi_link_collect_data_pub.publish()
  time.sleep(1)
  print("Kicking off NEPI CONNECT cloud connection")
  nepi_link_connect_now_pub.publish()
  time.sleep(1)
  print("Delaying " + str(SEND_RESET_DELAY_S) + " secs")
  time.sleep(SEND_RESET_DELAY_S)
  print("Waiting for next snapshot event trigger")

### Cleanup processes on node shutdown
def cleanup_actions():
  print("Cleanup Complete")

### Script Entrypoint
def startNode():
  rospy.loginfo("Starting Snapshot Event Send to Cloud automation script", disable_signals=True)
  rospy.init_node(name="snapshot_event_send_to_cloud_auto_script")
  # Run Initialization processes
  initialize_actions()
  # Set up snapshot event callback
  rospy.Subscriber(SNAPSHOT_TOPIC, Empty, snapshot_event_callback, queue_size = 1)
  #Set up cleanup on node shutdown
  rospy.on_shutdown(cleanup_actions)
  # Spin forever (until object is detected)
  rospy.spin()


#############################################################
# Main
#############################################################

if __name__ == '__main__':
  startNode()

