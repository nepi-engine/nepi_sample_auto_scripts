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
# 2. Waits for NEPI AI Alerts application
# 3. Adjust LED level based on target location in image



import time
import sys
import rospy
import statistics
import numpy as np
from nepi_edge_sdk_base import nepi_ros 
from nepi_edge_sdk_base import nepi_msg

from std_msgs.msg import Bool, Empty, Float32
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBoxes, ObjectCount


#########################################
# USER SETTINGS - Edit as Necessary 
#########################################

OBJECT_LABEL_OF_INTEREST = "bottle"
LOST_COUNT_THRESHOLD = 5
LED_LEVEL_MAX = 0.3
LED_BLINK_RATE = 0.5
LED_BLINK_THRESHOLD = 0.5
WATCHDOG_TIME = 4
AVG_LENGTH = 2





#########################################
# Node Class
#########################################

class led_adjust_on_object_detect(object):

  has_intensity = False
  has_blink = False
  is_blinking = False
  set_intensity = 0
  lost_count = 0

  
  #######################
  ### Node Initialization
  DEFAULT_NODE_NAME = "led_adjust_on_object_detect" # Can be overwitten by luanch command
  def __init__(self):
    #### APP NODE INIT SETUP ####
    nepi_ros.init_node(name= self.DEFAULT_NODE_NAME)
    self.node_name = nepi_ros.get_node_name()
    self.base_namespace = nepi_ros.get_base_namespace()
    nepi_msg.createMsgPublishers(self)
    nepi_msg.publishMsgInfo(self,"Starting Initialization Processes")
    ##############################
    ## Initialize Class Variables
    self.led_intensity_pub = None
    self.object_label_of_interest = OBJECT_LABEL_OF_INTEREST
    self.led_level_max = LED_LEVEL_MAX
    self.wd_timeout_sec = WATCHDOG_TIME
    self.wd_check_interval_sec = 1
    self.wd_timer = 0
    self.intensity_history = np.zeros(AVG_LENGTH)


    led_control_topic_name = "lsx/turn_on_off"
    nepi_msg.publishMsgInfo(self,"Waiting for topic name: " + led_control_topic_name)
    led_control_topic=nepi_ros.wait_for_topic(led_control_topic_name)
    if led_control_topic != "":
      self.led_on_off_pub = rospy.Publisher(led_control_topic, Bool, queue_size = 1)

    led_control_topic_name = "lsx/set_intensity"
    nepi_msg.publishMsgInfo(self,"Looking for topic name: " + led_control_topic_name)
    led_control_topic=nepi_ros.find_topic(led_control_topic_name)
    if led_control_topic != "":
      self.has_intensity = True
      self.led_intensity_pub = rospy.Publisher(led_control_topic, Float32, queue_size = 1)

    led_control_topic_name = "lsx/blink_on_off"
    nepi_msg.publishMsgInfo(self,"Looking for topic name: " + led_control_topic_name)
    led_control_topic=nepi_ros.find_topic(led_control_topic_name)
    if led_control_topic != "":
      self.has_blink = True
      self.led_blink_on_off_pub = rospy.Publisher(led_control_topic, Bool, queue_size = 1)
      led_control_topic = led_control_topic.replace("blink_on_off","set_blink_interval")
      self.led_blink_interval_pub = rospy.Publisher(led_control_topic, Float32, queue_size = 1)


    if self.has_intensity or self.has_blink:
      time.sleep(1)
      if not rospy.is_shutdown():
        if self.has_intensity:
          self.led_intensity_pub.publish(data = 0)
        if self.has_blink:
          self.led_blink_on_off_pub.publish(False)
        self.led_on_off_pub.publish(True)
          


      self.img_width = 0 # Updated on receipt of first image
      self.img_height = 0 # Updated on receipt of first image
      self.object_detected = False
      ## Define Class Namespaces
      # AI Detector Subscriber Topics
      AI_BOUNDING_BOXES_TOPIC = self.base_namespace + "ai_detector_mgr/bounding_boxes"
      AI_DETECTION_IMAGE_TOPIC = self.base_namespace + "ai_detector_mgr/detection_image"
      AI_FOUND_OBJECT_TOPIC = self.base_namespace + "ai_detector_mgr/found_object"
      ## Class subscribers

      # Wait for AI detector image topic to publish
      nepi_msg.publishMsgInfo(self,"Connecting to NEPI Detector Image Topic")
      nepi_msg.publishMsgInfo(self,AI_DETECTION_IMAGE_TOPIC )
      nepi_msg.publishMsgInfo(self,"Waiting for topic: " + AI_DETECTION_IMAGE_TOPIC)
      nepi_ros.wait_for_topic(AI_DETECTION_IMAGE_TOPIC)
      nepi_msg.publishMsgInfo(self,"Found topic: " + led_control_topic)
      ## Start Class Subscribers
      # Set up object detector subscriber
      nepi_msg.publishMsgInfo(self,"Starting object detection subscriber: Object of interest = " + self.object_label_of_interest + "...")
      rospy.Subscriber(AI_BOUNDING_BOXES_TOPIC, BoundingBoxes, self.object_detected_callback, queue_size = 1)
      #Set up found object subscriber which monitors all AI outputs
      nepi_msg.publishMsgInfo(self,"Starting found object subscriber")
      rospy.Subscriber(AI_FOUND_OBJECT_TOPIC, ObjectCount, self.found_object_callback, queue_size = 1)
      ## Start Node Processes
      # Setup LED process
      nepi_msg.publishMsgInfo(self,"Setting up LED timer")
      rospy.Timer(rospy.Duration(self.wd_check_interval_sec), self.led_timer_callback)

      ##############################
      ## Initiation Complete
      nepi_msg.publishMsgInfo(self," Initialization Complete")
      # Spin forever (until object is detected)
      rospy.spin()
      ##############################



  #######################
  ### Node Methods

  # Action upon detection of object of interest
  def object_detected_callback(self,bounding_boxes_msg):
    self.img_height = bounding_boxes_msg.image_height
    self.img_width = bounding_boxes_msg.image_width
    object_detected = False
    # Iterate over all of the objects reported by the detector
    for box in bounding_boxes_msg.bounding_boxes:
      # Check for the object of interest and take appropriate actions
      if box.Class == self.object_label_of_interest:
        self.lost_count = 0
        box_of_interest=box
        #nepi_msg.publishMsgInfo(box_of_interest.Class)
        # Calculate the box center in image ratio terms
        object_loc_y_pix = box_of_interest.ymin + ((box_of_interest.ymax - box_of_interest.ymin)  / 2) 
        object_loc_x_pix = box_of_interest.xmin + ((box_of_interest.xmax - box_of_interest.xmin)  / 2)
        object_loc_y_ratio = float(object_loc_y_pix) / self.img_height
        object_loc_x_ratio = float(object_loc_x_pix) / self.img_width
        #nepi_msg.publishMsgInfo("Object Detected " + self.object_label_of_interest + " with box center (" + str(object_loc_x_ratio) + ", " + str(object_loc_y_ratio) + ")")
        # check if we are AIose enough to center in either dimension to stop motion: Hysteresis band
        box_abs_error_x_ratio = 2.0 * abs(object_loc_x_ratio - 0.5)
        box_abs_error_y_ratio = 2.0 * abs(object_loc_y_ratio - 0.5)
        #nepi_msg.publishMsgInfo("Object Detection Error Ratios Horz: " "%.2f" % (box_abs_error_x_ratio) + " Vert: " + "%.2f" % (box_abs_error_y_ratio))
        # Sending LED level update
        center_ratios = [1-box_abs_error_x_ratio] # ignore vertical
        mean_center_ratio = statistics.mean(center_ratios)
        #nepi_msg.publishMsgInfo(self,"Target center ratio: " + "%.2f" % (mean_center_ratio))
        intensity = self.led_level_max *  mean_center_ratio**4
        self.intensity_history = np.roll(self.intensity_history,1)
        self.intensity_history[0]=intensity
        self.set_intensity = np.mean(self.intensity_history)
        if mean_center_ratio > LED_BLINK_THRESHOLD:
          self.set_blink_interval = LED_BLINK_RATE
        else:
          self.set_blink_interval = 0
        object_detected = True
      else:
        self.lost_count += 1
    self.object_detected = object_detected


  ### Check the number of objects detected on last detection process
  def found_object_callback(self,found_obj_msg):
    self.wd_timer = 0
    if found_obj_msg.count == 0:
      self.lost_count += 1
    if self.lost_count > LOST_COUNT_THRESHOLD:
      #nepi_msg.publishMsgInfo(self,"No objects found")
      self.object_detected=False


  ### Setup a regular background scan process based on timer callback
  def led_timer_callback(self,timer):
    # Called periodically no matter what as a Timer object callback
    nepi_msg.publishMsgWarn(self,"LED timer: " + str(self.wd_timer))
    if self.wd_timer > self.wd_timeout_sec:
      nepi_msg.publishMsgInfo(self,"Past timeout time, turning lights off")
      if self.has_intensity:
        self.led_intensity_pub.publish(data = 0)
      if self.has_blink and self.is_blinking == True:
        self.led_blink_on_off_pub.publish(False)
        self.is_blinking = False
      self.led_on_off_pub.publish(False)
    else:
      self.wd_timer += self.wd_check_interval_sec
      #self.led_on_off_pub.publish(True)
      if self.object_detected:
        if not rospy.is_shutdown():
          #self.led_on_off_pub.publish(True)
          if self.has_intensity:
            nepi_msg.publishMsgInfo(self,"Setting intensity level to: " + "%.2f" % (self.set_intensity))
            self.led_intensity_pub.publish(data = self.set_intensity)
          nepi_msg.publishMsgInfo(self,"Have blink interval of: " + "%.2f" % (self.set_blink_interval))
          if self.has_blink and self.set_blink_interval > 0:
            if self.is_blinking == False:
              nepi_msg.publishMsgInfo(self,"Setting blink interval to: " + "%.2f" % (self.set_blink_interval))
              self.led_blink_on_off_pub.publish(True)
              self.led_blink_interval_pub.publish(data = self.set_blink_interval)
              self.is_blinking = True
          else:
            self.led_blink_on_off_pub.publish(False)
            self.is_blinking = False
      elif not rospy.is_shutdown():
          if self.has_intensity:
            self.led_intensity_pub.publish(data = 0)
          if self.has_blink and self.is_blinking == True:
            self.led_blink_on_off_pub.publish(False)
            self.is_blinking = False




  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    global led_intensity_pub
    nepi_msg.publishMsgInfo(self,"Shutting down: Executing script cleanup actions")
    self.led_intensity_pub.publish(data = 0)



#########################################
# Main
#########################################
if __name__ == '__main__':
  led_adjust_on_object_detect()


