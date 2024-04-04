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
from resources import nepi

from sensor_msgs.msg import Image
from std_msgs.msg import UInt8, Empty, String, Bool
from nepi_ros_interfaces.msg import ClassifierSelection, StringArray

#########################################
# USER SETTINGS - Edit as Necessary 
#########################################

#Set AI Detector Image ROS Topic Name or Partial Name
IMAGE_INPUT_TOPIC_NAME = "color_2d_image"

#Set AI Detector Parameters
DETECTION_MODEL = "common_object_detection"
DETECTION_THRESHOLD = 0.5

#########################################
# ROS NAMESPACE SETUP
#########################################

# ROS namespace setup
NEPI_BASE_NAMESPACE = nepi.get_base_namespace()

#########################################
# Node Class
#########################################

class ai_detector_config(object):

  #######################
  ### Node Initialization
  def __init__(self):
    rospy.loginfo("Starting Initialization Processes")
    ## Initialize Class Variables
    ## Define Class Namespaces
    AI_START_TOPIC = NEPI_BASE_NAMESPACE + "start_classifier"
    AI_STOP_TOPIC = NEPI_BASE_NAMESPACE + "stop_classifier"
    ## Create Class Publishers
    self.start_classifier_pub = rospy.Publisher(AI_START_TOPIC, ClassifierSelection, queue_size=1)
    self.stop_classifier_pub = rospy.Publisher(AI_STOP_TOPIC, Empty, queue_size=10)
    ## Start Class Subscribers
    # Wait for image topic to publish
    rospy.loginfo("Waiting for topic name: " + IMAGE_INPUT_TOPIC_NAME)
    image_topic=nepi.wait_for_topic(IMAGE_INPUT_TOPIC_NAME)
    rospy.loginfo("Found topic: " + image_topic)
    self.classifier_selection = ClassifierSelection(img_topic=image_topic, classifier=DETECTION_MODEL, detection_threshold=DETECTION_THRESHOLD)
    ## Start Node Processes
    nepi.sleep(1,10)
    rospy.loginfo("Starting object detector: " + str(self.start_classifier_pub.name))
    self.start_classifier_pub.publish(self.classifier_selection)
    ## Initiation Complete
    rospy.loginfo("Initialization Complete")

  #######################
  ### Node Methods
  

  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    rospy.loginfo("Shutting down: Executing script cleanup actions")
    self.stop_classifier_pub.publish(Empty())



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

