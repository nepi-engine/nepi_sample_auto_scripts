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
# 3. Takes actions base on current alert state

import os
#### ROS namespace setup
#NEPI_BASE_NAMESPACE = '/nepi/s2x/'
#os.environ["ROS_NAMESPACE"] = NEPI_BASE_NAMESPACE[0:-1] # remove to run as automation script
import rospy
import time
import sys
from nepi_edge_sdk_base import nepi_ros
from nepi_edge_sdk_base import nepi_msg 

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64
from nepi_ros_interfaces.msg import LSXStatus
from nepi_ros_interfaces.srv import LSXCapabilitiesQuery, LSXCapabilitiesQueryResponse
from nepi_app_ai_alerts.msg import AiAlertsStatus, AiAlerts


#########################################
# USER SETTINGS - Edit as Necessary 
#########################################

LED_LEVEL_MAX = 0.3 # 0-1 ratio
LED_LEVEL_STEP = 0.05 # 0-1 ratio
LED_STEP_SEC = 1.0

#Set LED Control ROS Topic Name (or partial name)
LED_STATUS_TOPIC_NAME = "lsx/status"

# Start and Alert Actions List [on_off_state,intensity_val,blink_state,blink_time_sec,color_string] Use -999 to ignore (not set)
# States or actions for unsupported capabilities will be ignored
START_STATE = [True,0.2,False,0.0,"GREEN"]
ALERT_TRUE_ACTIONS = [True,0.2,True,0.5,"RED"]
ALERT_FALSE_ACTIONS = [True,0.4,False,-999,"GREEN"]

#########################################
# Node Class
#########################################

class led_alert_actions(object):

  led_on_off_pub = None
  led_intensity_pub = None
  led_color_pub = None
  led_blink_interval_pub = None
  led_on_off_pub = None

  led_state = None
  last_led_state = None

  alert_state = False
  last_alert_state = None

  #######################
  ### Node Initialization
  DEFAULT_NODE_NAME = "led_alert_acations" # Can be overwitten by luanch command
  def __init__(self):
    #### APP NODE INIT SETUP ####
    nepi_ros.init_node(name= self.DEFAULT_NODE_NAME)
    self.node_name = nepi_ros.get_node_name()
    self.base_namespace = nepi_ros.get_base_namespace()
    nepi_msg.createMsgPublishers(self)
    nepi_msg.publishMsgInfo(self,"Starting Initialization Processes")
    ##############################
    ## Initialize Class Variables
    ## Connect to LED
    led_status_topic_name = LED_STATUS_TOPIC_NAME
    nepi_msg.publishMsgInfo(self,"Waiting for status topic name: " + led_status_topic_name)
    led_status_topic=nepi_ros.wait_for_topic(led_status_topic_name)
    nepi_msg.publishMsgInfo(self,"Found status topic: " + led_status_topic)
    rospy.Subscriber(led_status_topic, LSXStatus, self.ledStatusCb, queue_size = 1)
    nepi_msg.publishMsgInfo(self,"Waiting for status msg to publish")
    while self.led_state == None:
      time.sleep(1)
    led_namespace = led_status_topic.split('/lsx')[0]
    # Get LED device capabilities
    led_capabilities_service_topic = os.path.join( led_namespace,"lsx/capabilities_query")
    try:
      led_caps_service = rospy.ServiceProxy(led_capabilities_service_topic, LSXCapabilitiesQuery)
      time.sleep(1)
      led_caps = led_caps_service()
      self.has_standby_mode = led_caps.has_standby_mode
      self.has_on_off_control = led_caps.has_on_off_control
      self.has_intensity_control = led_caps.has_intensity_control
      self.has_color_control = led_caps.has_color_control
      self.color_options_list = led_caps.color_options_list
      self.has_kelvin_control = led_caps.has_kelvin_control
      self.kelvin_min = led_caps.kelvin_min
      self.kelvin_max = led_caps.kelvin_max
      self.has_blink_control = led_caps.has_blink_control
    except Exception as e:
      nepi_msg.publishMsgWarn(self,"Failed to call led capabilities service: " + led_capabilities_service_topic + " " + str(e))
      self.has_standby_mode = False
      self.has_on_off_control = False
      self.has_intensity_control = False
      self.has_color_control = False
      self.has_blink_control = False
      self.color_options_list = ["None"]
      self.has_kelvin_control = False
      self.kelvin_min = 0
      self.kelvin_max = 1
    # Check for action controls support
    if  self.has_on_off_control == False and self.has_intensity_control == False \
        and self.has_color_control == False and self.has_blink_control == False:
        rospy.signal_shutdown("LED has no capabilities need for actions, Shutting Down")
    else:
      ## Create Class Publishers
      if self.has_on_off_control == True:
        self.led_on_off_pub = rospy.Publisher(os.path.join(led_namespace,"lsx/turn_on_off"), Bool, queue_size = 1)
      if self.has_intensity_control == True:
        self.led_intensity_pub = rospy.Publisher(os.path.join(led_namespace,"lsx/set_intensity_ratio"), Float32, queue_size = 1)
      if self.has_blink_control == True:
        self.led_blink_on_off_pub = rospy.Publisher(os.path.join(led_namespace,"lsx/blink_on_off"), Bool, queue_size = 1)
        self.led_blink_interval_pub = rospy.Publisher(os.path.join(led_namespace,"lsx/set_blink_interval"), Float32, queue_size = 1)
      if self.has_color_control == True:
        self.led_color_pub = rospy.Publisher(os.path.join(led_namespace,"lsx/set_color"), String, queue_size = 1)
      time.sleep(1)
      # Initialize LED
      nepi_msg.publishMsgInfo(self,"Initializaing LED state")
      self.updateLedState(START_STATE)

      ## Connect to Alert App
      alert_state_topic = os.path.join(self.base_namespace,"app_ai_alerts/alert_state")
      nepi_msg.publishMsgInfo(self,"Waiting for alert state topic name: " + alert_state_topic)
      alert_state_topic=nepi_ros.wait_for_topic(alert_state_topic)
      nepi_msg.publishMsgInfo(self,"Found alert state topic: " + alert_state_topic)
      rospy.Subscriber(alert_state_topic, Bool, self.alert_stateCb, queue_size = 1)
      nepi_msg.publishMsgInfo(self,"Waiting for alert state msg to publish")
      while self.alert_state == None:
        time.sleep(1)

    ##############################
    ## Initiation Complete
    nepi_msg.publishMsgInfo(self," Initialization Complete")
    # Spin forever (until object is detected)
    rospy.spin()
    ##############################


  def updateLedState(self,new_led_state):
      if new_led_state != self.last_led_state:
        if self.led_on_off_pub != None and new_led_state[0] != -999:
          self.led_on_off_pub.publish(new_led_state[0])
        if self.led_intensity_pub != None and new_led_state[1] != -999:
          self.led_intensity_pub.publish(new_led_state[1])
        if self.led_blink_on_off_pub and new_led_state[2] != -999:
          self.led_blink_on_off_pub.publish(new_led_state[2])
        if self.led_blink_interval_pub != None and new_led_state[3] != -999:
          self.led_blink_interval_pub.publish(new_led_state[3])
        if self.led_color_pub != None and new_led_state[4] != -999:
          self.led_color_pub.publish(new_led_state[4])
      self.last_led_state = new_led_state

  def ledStatusCb(self,msg):
    self.led_state = [msg.on_off_state,msg.intensity_ratio,msg.blink_state,msg.blink_interval,msg.color_setting]
    if self.last_led_state == None:
      self.last_led_state = [msg.on_off_state,msg.intensity_ratio,msg.blink_state,msg.blink_interval,msg.color_setting]

  def alert_stateCb(self,msg):
    self.alert_state = msg.data
    if self.alert_state != self.last_alert_state or self.last_alert_state == None:
      nepi_msg.publishMsgInfo(self,"Alert State updated to: " + str(self.alert_state))
      if self.alert_state == True:
        self.updateLedState(ALERT_TRUE_ACTIONS)
      else:
        self.updateLedState(ALERT_FALSE_ACTIONS)
    self.last_alert_state = msg.data


  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    nepi_msg.publishMsgInfo(self,"Shutting down: Executing script cleanup actions")
    self.updateLedState(START_STATE)


#########################################
# Main
#########################################
if __name__ == '__main__':
  led_alert_actions()

