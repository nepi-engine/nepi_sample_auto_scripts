#!/usr/bin/env python

__author__ = "Jason Seawall"
__copyright__ = "Copyright 2023, Numurus LLC"
__email__ = "nepi@numurus.com"
__credits__ = ["Jason Seawall", "Josh Maximoff"]

__license__ = "GPL"
__version__ = "2.0.4.0"


# Sample NEPI Automation Script. 
# Uses onboard ROS python library to
# 1. Checks if AI input image topic exists
# 2. Try's to set reslution on camera 
# 3. Loads selected AI model
# 4. Starts AI detection process using input image stream
# 5. Stops AI detection process on shutdown

import time
import sys
import rospy   

from sensor_msgs.msg import Image
from std_msgs.msg import UInt8, Empty, String, Bool
from nepi_ros_interfaces.msg import ClassifierSelection, StringArray

#####################################################################################
# SETUP - Edit as Necessary ##################################
##########################################

# Resolution control for NEPI ROS IDX supported camera drivers

CAM_RES=0 # Number 0-3, 0 Low, 1 Med, 2 High, 3 Ultra, or None to skip step
#CAM_RES=None

###!!!!!!!! Set AI Detector Image ROS Topic Name !!!!!!!!
IMAGE_INPUT_TOPIC = "/nepi/s2x/nexigo_n60_fhd_webcam_audio/idx/color_2d_image"
#IMAGE_INPUT_TOPIC = "/nepi/s2x/see3cam_cu81/idx/color_2d_image"
#IMAGE_INPUT_TOPIC = "/nepi/s2x/sidus_ss400/idx/color_2d_image"
#IMAGE_INPUT_TOPIC = "/nepi/s2x/onwote_hd_poe/idx/color_2d_image"
#IMAGE_INPUT_TOPIC = "/nepi/s2x/zed2/zed_node/left/image_rect_color"

DETECTION_MODEL = "common_object_detection"
DETECTION_THRESHOLD = 0.5

# NEPI ROS namespace setup
NEPI_BASE_NAMESPACE = "/nepi/s2x/"

# AI Detector Publish Topics
AI_START_TOPIC = NEPI_BASE_NAMESPACE + "start_classifier"
AI_STOP_TOPIC = NEPI_BASE_NAMESPACE + "stop_classifier"

#####################################################################################
# Globals
#####################################################################################
stop_classifier_pub = rospy.Publisher(AI_STOP_TOPIC, Empty, queue_size=10)
#####################################################################################
# Methods
#####################################################################################

### System Initialization processes
def initialize_actions():
  print("")
  print("Starting Initialization")  
  # Wait for topic
  print("Waiting for topic: " + IMAGE_INPUT_TOPIC)
  wait_for_topic(IMAGE_INPUT_TOPIC, 'sensor_msgs/Image')
  # Try updating IDX sensor resolution
  if CAM_RES is not None:
    if IMAGE_INPUT_TOPIC.find('idx')>0: # Is IDX supported sensor stream
      print("Image topic has IDX sensor driver support")
      resolution_adj_topic = IMAGE_INPUT_TOPIC.split('idx')[0] + 'idx/set_resolution_mode'
      print("Updating resolution to value: " + str(CAM_RES) + " on topic")
      print(resolution_adj_topic)
      res_adj_pub = rospy.Publisher(resolution_adj_topic, UInt8, queue_size=10)
      time.sleep(1) # Important to sleep between publisher constructor and publish()
      res_adj_pub.publish(CAM_RES)
    else:
      print("Image has no IDX sensor driver")
      print("Skipping resolution update step")
  else:
    print("Resolution = None")
    print("Skipping resolution update step")
  # Classifier initialization, and wait for it to publish
  start_classifier_pub = rospy.Publisher(AI_START_TOPIC, ClassifierSelection, queue_size=1)
  classifier_selection = ClassifierSelection(img_topic=IMAGE_INPUT_TOPIC, classifier=DETECTION_MODEL, detection_threshold=DETECTION_THRESHOLD)
  time.sleep(1) # Important to sleep between publisher constructor and publish()
  rospy.loginfo("Starting object detector: " + str(start_classifier_pub.name))
  start_classifier_pub.publish(classifier_selection)
  print("Initialization Complete")

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
  global stop_classifier_pub
  print("Shutting down: Executing script cleanup actions")
  stop_classifier_pub.publish(Empty())

### Script Entrypoint
def startNode():
  rospy.loginfo("Starting AI Detection Start automation script", disable_signals=True) # Disable signals so we can force a shutdown
  rospy.init_node(name="ai_detection_setup_start_auto_script")
  # Run initialization processes
  initialize_actions()
  # run cleanup actions on shutdown
  rospy.on_shutdown(cleanup_actions)
  # Spin forever
  rospy.spin()
  
  


#####################################################################################
# Main
#####################################################################################

if __name__ == '__main__':
  startNode()

