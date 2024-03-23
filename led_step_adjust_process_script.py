#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

# Sample NEPI Process Script. 
# 1. Waits for LED system
# 2. Steps through LED intensities

# Requires the following additional scripts are running
# a)ai_detector_config_script.py
# These scripts are available for download at:
# [link text](https://github.com/numurus-nepi/nepi_sample_auto_scripts)

import rospy
import time
import sys
from resources import nepi

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
NEPI_BASE_NAMESPACE = nepi.get_base_namespace()


#########################################
# Node Class
#########################################

class led_step_adjust_process(object):

  #######################
  ### Node Initialization
  
  def __init__(self):
    rospy.loginfo("Starting Initialization Processes")
    ## Initialize Class Variables
    self.led_level_max = LED_LEVEL_MAX
    self.led_level_step = LED_LEVEL_STEP
    self.led_step_sec = LED_STEP_SEC
    self.led_last_level = 0
    ## Define Class Namespaces
    ## Create Class Publishers
    led_control_topic_name = LED_CONTROL_TOPIC_NAME
    rospy.loginfo("Waiting for topic name: " + led_control_topic_name)
    led_control_topic=nepi.wait_for_topic(led_control_topic_name)
    rospy.loginfo("Found topic: " + led_control_topic)
    self.led_intensity_pub = rospy.Publisher(led_control_topic, Float32, queue_size = 1)
    ## Start Class Subscribers
    ## Start Node Processes
    #Start level step loop
    rospy.loginfo("Starting LED level step loop")
    rospy.Timer(rospy.Duration(self.led_step_sec), self.led_step_callback)
    ## Initiation Complete
    rospy.loginfo("Initialization Complete")

  #######################
  ### Node Methods

  ### Setup a regular led adjust process
  def led_step_callback(self,timer):
    led_level = self.led_last_level + self.led_level_step
    if led_level > self.led_level_max:
      led_level = 0.0
    rospy.loginfo("Setting LED level to: " + '%.2f' % led_level)
    if not rospy.is_shutdown():
      self.led_intensity_pub.publish(data = led_level)
      self.led_last_level = led_level


  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    rospy.loginfo("Shutting down: Executing script cleanup actions")
    self.led_intensity_pub.publish(data = 0)


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

