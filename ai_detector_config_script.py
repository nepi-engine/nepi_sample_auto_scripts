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

# Sample NEPI Config Script.
# 1. Checks if AI input image topic exists
# 2. Loads selected AI model
# 3. Starts AI detection process using input image stream
# 4. Stops AI detection process on shutdown

import time
import sys
import rospy   

from sensor_msgs.msg import Image
from std_msgs.msg import UInt8, Empty, String, Bool
from nepi_ros_interfaces.msg import ClassifierSelection, StringArray

#########################################
# USER SETTINGS - Edit as Necessary 
#########################################


#Set AI Detector Image ROS Topic Name
IMAGE_INPUT_TOPIC_NAME = "color_2d_image"

#Set AI Detector Parameters
DETECTION_MODEL = "common_object_detection"
DETECTION_THRESHOLD = 0.5

#########################################
# ROS NAMESPACE SETUP
#########################################

NEPI_BASE_NAMESPACE = "/nepi/s2x/"

# AI Detector Publish Topics
AI_START_TOPIC = NEPI_BASE_NAMESPACE + "start_classifier"
AI_STOP_TOPIC = NEPI_BASE_NAMESPACE + "stop_classifier"

#########################################
# Globals
#########################################

stop_classifier_pub = rospy.Publisher(AI_STOP_TOPIC, Empty, queue_size=10)

#########################################
# Methods
#########################################

### System Initialization processes
def initialize_actions():
  print("")
  print("Starting Initialization")  
  # Wait for topic by name
  print("Waiting for topic name: " + IMAGE_INPUT_TOPIC_NAME)
  image_topic=wait_for_topic(IMAGE_INPUT_TOPIC_NAME)
  print("Found topic: " + image_topic)
  # Classifier initialization, and wait for it to publish
  start_classifier_pub = rospy.Publisher(AI_START_TOPIC, ClassifierSelection, queue_size=1)
  classifier_selection = ClassifierSelection(img_topic=image_topic, classifier=DETECTION_MODEL, detection_threshold=DETECTION_THRESHOLD)
  time.sleep(1) # Important to sleep between publisher constructor and publish()
  rospy.loginfo("Starting object detector: " + str(start_classifier_pub.name))
  start_classifier_pub.publish(classifier_selection)
  print("Initialization Complete")

#######################
# Initialization Functions

#######################
# Initialization Functions

### Function to find a topic
def find_topic(topic_name):
  topic = ""
  topic_list=rospy.get_published_topics(namespace='/')
  for topic_entry in topic_list:
    if topic_entry[0].find(topic_name) != -1:
      topic = topic_entry[0]
  return topic

### Function to check for a topic 
def check_for_topic(topic_name):
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


### Cleanup processes on node shutdown
def cleanup_actions():
  global stop_classifier_pub
  print("Shutting down: Executing script cleanup actions")
  stop_classifier_pub.publish(Empty())

### Script Entrypoint
def startNode():
  rospy.loginfo("Starting AI 2D Detector Config Script", disable_signals=True) # Disable signals so we can force a shutdown
  rospy.init_node(name="ai_2d_detector_config_auto_script")
  # Run initialization processes
  initialize_actions()
  # run cleanup actions on shutdown
  rospy.on_shutdown(cleanup_actions)
  # Spin forever
  rospy.spin()
  
  
#########################################
# Main
#########################################

if __name__ == '__main__':
  startNode()

