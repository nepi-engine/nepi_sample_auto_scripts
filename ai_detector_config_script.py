#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

# Sample NEPI Config Script.
# 1. Checks if AI input image topic exists
# 2. Loads selected AI model
# 3. Starts AI detection process using input image stream
# 4. Stops AI detection process on shutdown

import time
import sys
import rospy
from nepi_edge_sdk_base import nepi_ros
from nepi_edge_sdk_base import nepi_msg

from sensor_msgs.msg import Image
from std_msgs.msg import UInt8, Empty, String, Bool
from nepi_ros_interfaces.msg import ClassifierSelection, StringArray

#########################################
# USER SETTINGS - Edit as Necessary 
#########################################

#Set AI Detector Image ROS Topic Name or Partial Name
IMAGE_INPUT_TOPIC_NAME = "color_2d_image"

#Set AI Detector Parameters
DETECTION_MODEL = "darknet_common_object_detection_fast"
DETECTION_THRESHOLD = 0.5


#########################################
# Node Class
#########################################

class ai_detector_config(object):

  #######################
  ### Node Initialization
  DEFAULT_NODE_NAME = "ai_detector_config" # Can be overwitten by luanch command
  def __init__(self):
    #### APP NODE INIT SETUP ####
    nepi_ros.init_node(name= self.DEFAULT_NODE_NAME)
    self.node_name = nepi_ros.get_node_name()
    self.base_namespace = nepi_ros.get_base_namespace()
    nepi_msg.createMsgPublishers(self)
    nepi_msg.publishMsgInfo(self,"Starting Initialization Processes")
    ##############################
    ## Initialize Class Variables
    ## Define Class Namespaces
    AI_START_TOPIC = self.base_namespace + "ai_detector_mgr/start_classifier"
    AI_STOP_TOPIC = self.base_namespace + "ai_detector_mgr/stop_classifier"
    ## Create Class Publishers
    self.start_classifier_pub = rospy.Publisher(AI_START_TOPIC, ClassifierSelection, queue_size=1)
    self.stop_classifier_pub = rospy.Publisher(AI_STOP_TOPIC, Empty, queue_size=10)
    ## Start Class Subscribers
    # Wait for image topic to publish
    nepi_msg.publishMsgInfo(self,"Waiting for topic name: " + IMAGE_INPUT_TOPIC_NAME)
    image_topic=nepi_ros.wait_for_topic(IMAGE_INPUT_TOPIC_NAME)
    nepi_msg.publishMsgInfo(self,"Found topic: " + image_topic)
    self.classifier_selection = ClassifierSelection(img_topic=image_topic, classifier=DETECTION_MODEL, detection_threshold=DETECTION_THRESHOLD)
    ## Start Node Processes
    nepi_msg.publishMsgInfo(self,"Starting object detector: " + str(self.start_classifier_pub.name))
    self.start_classifier_pub.publish(self.classifier_selection)

    ##############################
    ## Initiation Complete
    nepi_msg.publishMsgInfo(self," Initialization Complete")
    # Spin forever (until object is detected)
    rospy.spin()
    ##############################

  #######################
  ### Node Methods
  

  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    nepi_msg.publishMsgInfo(self,"Shutting down: Executing script cleanup actions")
    self.stop_classifier_pub.publish(Empty())



#########################################
# Main
#########################################
if __name__ == '__main__':
  ai_detector_config()

