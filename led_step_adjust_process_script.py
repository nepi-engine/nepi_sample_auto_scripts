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

# Sample NEPI Process Script. 
# 1. Waits for LED system
# 2. Steps through LED intensities

# Requires the following additional scripts are running
# a)ai_detector_config_script.py
# These scripts are available for download at:
# [link text](https://github.com/numurus-nepi/nepi_sample_auto_scripts)

import time
import sys
import rospy



from std_msgs.msg import Empty, Float32

#########################################
# USER SETTINGS - Edit as Necessary 
#########################################

LED_LEVEL_MAX = 0.3 # 0-1 ratio
LED_LEVEL_STEP = 0.05 # 0-1 ratio
LED_STEP_SEC = 1.0

#Set LED Control ROS Topic Name (or partial name)
LED_CONTROL_TOPIC_NAME = "lsx/set_intensity"

#########################################
# ROS NAMESPACE SETUP
#########################################

# ROS namespace setup
NEPI_BASE_NAMESPACE = "/nepi/s2x/"

#########################################
# Globals
#########################################

led_intensity_pub = None
led_last_level = 0

#########################################
# Methods
#########################################

### System Initialization processes
def initialize_actions():
  global led_intensity_pub
  print("")
  print("Starting Initialization Processes")
  # Wait for topic by name
  print("Waiting for topic name: " + LED_CONTROL_TOPIC_NAME)
  led_control_topic=find_topic(LED_CONTROL_TOPIC_NAME)
  print("Found topic: " + led_control_topic)
  led_intensity_pub = rospy.Publisher(led_control_topic, Float32, queue_size = 1)
  #Start level step loop
  print("Starting LED level step loop")
  rospy.Timer(rospy.Duration(LED_STEP_SEC), led_step_callback)
  print("Initialization Complete")
 
### Setup a regular led adjust process
def led_step_callback(timer):
  global led_intensity_pub
  global led_last_level
  led_level = led_last_level + LED_LEVEL_STEP
  if led_level > LED_LEVEL_MAX:
    led_level = 0.0
  print("Setting LED level to: " + '%.2f' % led_level)
  if not rospy.is_shutdown():
    led_intensity_pub.publish(data = led_level)
    led_last_level = led_level

#######################
# Initialization Functions

### Function to find a topic
def find_topic(topic_name):
  topic = ""
  topic_list=rospy.get_published_topics(namespace='/')
  #print(topic_list)
  for topic_entry in topic_list:
    #print(topic_entry[0])
    if topic_entry[0].find(topic_name) != -1:
      topic = topic_entry[0]
  return topic

### Function to check for a topic 
def wait_for_topic(topic_name):
  topic_exists = True
  topic=find_topic(topic_name)
  if topic == "":
    topic_exists = False
  return topic_exists

### Function to wait for a topic
def wait_for_topic(topic_name):
  topic = ""
  while topic == "" and not rospy.is_shutdown():
    topic=find_topic(topic_name)
    time.sleep(.1)
  return topic

#######################
# StartNode and Cleanup Functions

def cleanup_actions():
  global led_intensity_pub
  print("Shutting down: Executing script cleanup actions")
  led_intensity_pub.publish(data = 0)


### Script Entrypoint
def startNode():
  rospy.loginfo("Starting AI Detect and Snapshot Process Script", disable_signals=True) # Disable signals so we can force a shutdown
  rospy.init_node
  rospy.init_node(name="ai_detect_and_snapshot_process_script")
  # Run Initialization processes
  initialize_actions()
  #Set up Anode shutdown
  rospy.on_shutdown(cleanup_actions)
  # Spin forever (until object is detected)
  rospy.spin()


#########################################
# Main
#########################################

if __name__ == '__main__':
  startNode()

