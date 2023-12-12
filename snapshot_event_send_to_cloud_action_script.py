#!/usr/bin/env python

__author__ = "Jason Seawall"
__copyright__ = "Copyright 2023, Numurus LLC"
__email__ = "nepi@numurus.com"
__credits__ = ["Jason Seawall", "Josh Maximoff"]

__license__ = "GPL"
__version__ = "2.0.4.0"


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

###!!!!!!!! Set Image ROS Topic Name to Send  !!!!!!!!
IMAGE_INPUT_TOPIC = "/nepi/s2x/nexigo_n60_fhd_webcam_audio/idx/color_2d_image"
#IMAGE_INPUT_TOPIC = "/nepi/s2x/see3cam_cu81/idx/color_2d_image"
#IMAGE_INPUT_TOPIC = "/nepi/s2x/sidus_ss400/idx/color_2d_image"
#IMAGE_INPUT_TOPIC = "/nepi/s2x/onwote_hd_poe/idx/color_2d_image"
#IMAGE_INPUT_TOPIC = "/nepi/s2x/zed2/zed_node/left/image_rect_color"

###!!!!!!!! Set Automation action parameters !!!!!!!!
SEND_RESET_DELAY_S = 5.0 # Seconds. Delay before starting over

NEPI_BASE_NAMESPACE = "/nepi/s2x/"

# NEPI Link data topics and parameters
NEPI_LINK_NAMESPACE = NEPI_BASE_NAMESPACE + "nepi_link_ros_bridge/"
NEPI_LINK_ENABLE_TOPIC = NEPI_LINK_NAMESPACE + "enable"
NEPI_LINK_SET_DATA_SOURCES_TOPIC = NEPI_LINK_NAMESPACE + "lb/select_data_sources"
NEPI_LINK_COLLECT_DATA_TOPIC = NEPI_LINK_NAMESPACE + "lb/create_data_set_now"
NEPI_LINK_CONNECT_TOPIC = NEPI_LINK_NAMESPACE + "connect_now"

### Snapshot Topic Name
SNAPSHOT_TOPIC = NEPI_BASE_NAMESPACE + "snapshot_event"

#############################################################
# Globals
#############################################################
nepi_link_enable_pub = rospy.Publisher(NEPI_LINK_ENABLE_TOPIC, Bool, queue_size=10)
nepi_link_set_data_sources = rospy.Publisher(NEPI_LINK_SET_DATA_SOURCES_TOPIC, StringArray, queue_size=10)
nepi_link_collect_data_pub = rospy.Publisher(NEPI_LINK_COLLECT_DATA_TOPIC, Empty, queue_size=10)
nepi_link_connect_now_pub = rospy.Publisher(NEPI_LINK_CONNECT_TOPIC, Empty, queue_size=10)
image_source_name = IMAGE_INPUT_TOPIC.replace(NEPI_BASE_NAMESPACE,'')

#############################################################
# Methods
#############################################################

### System Initialization processes
def initialize_actions():
  global image_source_name
  print("")
  print("Starting Initialization")  
  # Wait for image topic to exist
  print("Waiting for image topic")  
  wait_for_topic(IMAGE_INPUT_TOPIC, 'sensor_msgs/Image')
  print("Image topic found")
  print("Setting NEPI CONNECT data sources")
  nepi_link_set_data_sources.publish([image_source_name])
  print("Initialization Complete")
  print("Waiting for snapshot event trigger topic to publish on:")
  print(SNAPSHOT_TOPIC)


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

