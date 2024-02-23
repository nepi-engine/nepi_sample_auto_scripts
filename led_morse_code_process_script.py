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
# 2. Blinks LED with Morce Code message then repeats

import time
import sys
import rospy



from std_msgs.msg import Empty, Float32

#########################################
# USER SETTINGS - Edit as Necessary 
#########################################

LED_BLINK_LEVEL = 0.3 # 0-1 ratio
LED_BLINK_SEC = 1
MC_MESSAGE = "SOS"
REPEAT_DELAY_SEC = 2.0

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
mc_message = MC_MESSAGE

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
  led_intensity_pub = rospy.Publisher(led_control_topic, Float32, queue_size = 30)
  print("Initialization Complete")
 
### Setup a regular led adjust process
def led_morce_code_process():
  global led_intensity_pub
  global mc_message
  print("Blinking Morce Code message: " + mc_message)
  if not rospy.is_shutdown():
    for let_char in mc_message:
      let_mc = mcDict(let_char)
      for mc_char in let_mc:
        if mc_char == ' ' and not rospy.is_shutdown():
          led_intensity_pub.publish(data = 0.0)
          time.sleep(LED_BLINK_SEC)
        if mc_char == '.' and not rospy.is_shutdown():
          led_intensity_pub.publish(data = LED_BLINK_LEVEL)
          time.sleep(LED_BLINK_SEC)
        if mc_char == '-' and not rospy.is_shutdown():
          led_intensity_pub.publish(data = LED_BLINK_LEVEL)
          time.sleep(3*LED_BLINK_SEC)
        led_intensity_pub.publish(data = 0.0)
        time.sleep(LED_BLINK_SEC)
    led_intensity_pub.publish(data = 0.0)


def mcDict(letter):
    code = {'A': '.-',
            'B': '-...',
            'C': '-.-.',
            'D': '-..',
            'E': '.',
            'F': '..-.',
            'G': '--.',
            'H': '....',
            'I': '..',
            'J': '.---',
            'K': '-.-',
            'L': '.-..',
            'M': '--',
            'N': '-.',
            'O': '---',
            'P': '.--.',
            'Q': '--.-',
            'R': '.-.',
            'S': '...',
            'T': '-',
            'U': '..-',
            'V': '...-',
            'W': '.--',
            'X': '-..-',
            'Y': '-.--',
            'Z': '--..',
            '1': '.----',
            '2': '..---',
            '3': '...--',
            '4': '....-',
            '5': '.....',
            '6': '-....',
            '7': '--...',
            '8': '---..',
            '9': '----.',
            '0': '-----',
            ' ': '  '}
    return code[letter]



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
  #Start level step loop
  print("Starting LED Morse Code message loop")
  while not rospy.is_shutdown():
    led_morce_code_process()
    time.sleep(REPEAT_DELAY_SEC)
  #Set up node shutdown
  rospy.on_shutdown(cleanup_actions)
  # Spin forever (until object is detected)
  rospy.spin()


#########################################
# Main
#########################################

if __name__ == '__main__':
  startNode()

