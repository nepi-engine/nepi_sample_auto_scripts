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
# 1. Waits for ai detection topic
# 2. Waits for specific object to be detected and centered
# 3. Publishes a event event trigger out
# 4. Delays trigger event for some set delay time

# Requires the following additional scripts are running
# a)ai_detector_config_script.py
# This automation script only sends a event event trigger.
# You will also want one or more event event action scripts running
# The following automation scripts are event event action scripts you can test
# a)(Optional)event_event_save_to_disk_action_script.py
# b)(Optional) event_event_send_to_cloud_action_script.py for cloud portal support
# These scripts are available for download at:
# [link text](https://github.com/numurus-nepi/nepi_sample_auto_scripts)

import time
import sys
import rospy   

from std_msgs.msg import Empty
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBoxes

#########################################
# USER SETTINGS - Edit as Necessary 
#########################################

OBJ_LABEL_OF_INTEREST = "person"
OBJ_CENTERED_BUFFER_RATIO = 0.5 # acceptable band about center of image for saving purposes
RESET_DELAY_S = 5 # Min delay between triggers

#########################################
# ROS NAMESPACE SETUP
#########################################

# ROS namespace setup
NEPI_BASE_NAMESPACE = nepi_ros.get_base_namespace()

# AI Detector Subscriber Topics
AI_BOUNDING_BOXES_TOPIC = NEPI_BASE_NAMESPACE + "classifier/bounding_boxes"
AI_DETECTION_IMAGE_TOPIC = NEPI_BASE_NAMESPACE + "classifier/detection_image"

# Snapshot Publish Topic
EVENT_TRIGGER_TOPIC = NEPI_BASE_NAMESPACE + "event_trigger"

#########################################
# Globals
#########################################

event_trigger_pub = rospy.Publisher(EVENT_TRIGGER_TOPIC, Empty, queue_size = 1)
img_width = 0 # Updated on receipt of first image
img_height = 0 # Updated on receipt of first image

#########################################
# Methods
#########################################

### System Initialization processes
def initialize_actions():
  global img_height
  global img_width
  print("")
  print("Starting Initialization Processes")
  print("Connecting to NEPI Detector Image Topic")
  print(AI_DETECTION_IMAGE_TOPIC )
  # Wait for topic
  print("Waiting for topic: " + AI_DETECTION_IMAGE_TOPIC)
  wait_for_topic(AI_DETECTION_IMAGE_TOPIC)
  img_sub = rospy.Subscriber(AI_DETECTION_IMAGE_TOPIC, Image, image_callback)
  while img_width == 0 and img_height == 0:
    print("Waiting for Classifier Detection Image")
    time.sleep(1)
  img_sub.unregister() # Don't need it anymore
  # Set up object detector subscriber
  print("Starting object detection subscriber: Object of interest = " + OBJ_LABEL_OF_INTEREST + "...")
  rospy.Subscriber(AI_BOUNDING_BOXES_TOPIC, BoundingBoxes, object_detected_callback, queue_size = 1)
  print("Initialization Complete")
 
### Simple callback to get image height and width
def image_callback(img_msg):
  # This is just to get the image size for ratio purposes
  global img_height
  global img_width
  if (img_height == 0 and img_width == 0):
    print("Initial input image received. Size = " + str(img_msg.width) + "x" + str(img_msg.height))
    img_height = img_msg.height
    img_width = img_msg.width

# Action upon detection of object of interest
def object_detected_callback(bounding_box_msg):
  global img_height
  global img_width
  global event_trigger_pub
  # Iterate over all of the objects reported by the detector
  for box in bounding_box_msg.bounding_boxes:
    # Check for the object of interest and take appropriate actions
    if box.Class == OBJ_LABEL_OF_INTEREST:
      box_of_interest=box
      print(box_of_interest.Class)
      # Calculate the box center in image ratio terms
      object_loc_y_pix = box_of_interest.ymin + ((box_of_interest.ymax - box_of_interest.ymin)  / 2) 
      object_loc_x_pix = box_of_interest.xmin + ((box_of_interest.xmax - box_of_interest.xmin)  / 2)
      object_loc_y_ratio = float(object_loc_y_pix) / img_height
      object_loc_x_ratio = float(object_loc_x_pix) / img_width
      print("Object Detected " + OBJ_LABEL_OF_INTEREST + " with box center (" + str(object_loc_x_ratio) + ", " + str(object_loc_y_ratio) + ")")
      # check if we are AIose enough to center in either dimension to stop motion: Hysteresis band
      box_abs_error_x_ratio = 1- 2.0 * abs(object_loc_x_ratio - 0.5)
      box_abs_error_y_ratio = 1- 2.0 * abs(object_loc_y_ratio - 0.5)
      print("Object Detection Error Ratios Horz: " "%.2f" % (box_abs_error_x_ratio) + " Vert: " + "%.2f" % (box_abs_error_y_ratio))
      if (box_abs_error_y_ratio >= OBJ_CENTERED_BUFFER_RATIO ) and \
         (box_abs_error_x_ratio >= OBJ_CENTERED_BUFFER_RATIO ):
        print("Detected a " + OBJ_LABEL_OF_INTEREST + " Close to image center")
        print("Sending event event trigger")
        event_trigger_pub.publish(Empty())
        print("Delaying next trigger for " + str(RESET_DELAY_S) + " secs")
        time.sleep(RESET_DELAY_S)
    else:
      print("No " + OBJ_LABEL_OF_INTEREST + " type for target data")
      time.sleep(1)


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
  print("Shutting down: Executing script cleanup actions")


### Script Entrypoint
def startNode():
  rospy.loginfo("Starting Event Trigger on AI Dection Process Script", disable_signals=True) # Disable signals so we can force a shutdown
  rospy.init_node
  rospy.init_node(name="event_trigger_on_ai_detect_process_script")
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

