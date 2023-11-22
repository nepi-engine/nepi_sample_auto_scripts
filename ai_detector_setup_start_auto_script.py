#!/usr/bin/env python

# Sample NEPI Automation Script. 
# Uses onboard ROS python library to
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

#####################################################################################
# SETUP - Edit as Necessary ##################################
##########################################

###!!!!!!!! Set AI Detector Image ROS Topic Name !!!!!!!!
IMAGE_INPUT_TOPIC = "/nepi/s2x/nexigo_n60_fhd_webcam_audio/idx/color_2d_image"
#IMAGE_INPUT_TOPIC = "/nepi/s2x/see3cam_cu81/idx/color_2d_image"
#IMAGE_INPUT_TOPIC = "/nepi/s2x/sidus_ss400/idx/color_2d_image"
#IMAGE_INPUT_TOPIC = "/nepi/s2x/onwote_hd_poe/idx/color_2d_image"
#IMAGE_INPUT_TOPIC = "/nepi/s2x/zed2/zed_node/left/image_rect_color"

DETECTION_MODEL = "common_object_detection"
DETECTION_THRESHOLD = 0.5
OBJ_CENTERED_BUFFER_RATIO = 0.5 # acceptable band about center of image for saving purposes

# NEPI ROS namespace setup
NEPI_BASE_NAMESPACE = "/nepi/s2x/"
# AI Detector Publish Topics
AI_START_TOPIC = NEPI_BASE_NAMESPACE + "start_classifier"
AI_STOP_TOPIC = NEPI_BASE_NAMESPACE + "stop_classifier"

#####################################################################################
# Globals
#####################################################################################

#####################################################################################
# Methods
#####################################################################################

### System Initialization processes
def initialize_actions():
  print("")
  print("Starting Initialization")  
  # Check if camera topic is publishing
  topic_list=rospy.get_published_topics(namespace='/')
  topic_to_connect=[IMAGE_INPUT_TOPIC, 'sensor_msgs/Image']
  while topic_to_connect not in topic_list:
    print("!!!!! Image topic not found, waiting 1 second")
    time.sleep(1)
  print("Image topic found")
  ### Classifier initialization, and wait for it to publish
  start_classifier_pub = rospy.Publisher(AI_START_TOPIC, ClassifierSelection, queue_size=1)
  classifier_selection = ClassifierSelection(img_topic=IMAGE_INPUT_TOPIC, classifier=DETECTION_MODEL, detection_threshold=DETECTION_THRESHOLD)
  time.sleep(2) # Important to sleep between publisher constructor and publish()
  rospy.loginfo("Starting object detector: " + str(start_classifier_pub.name))
  start_classifier_pub.publish(classifier_selection)
  print("Initialization Complete")


### Cleanup processes on node shutdown
def cleanup_actions():
  print("Shutting down: Executing script cleanup actions")
  stop_classifier_pub = rospy.Publisher(AI_STOP_TOPIC, Empty, queue_size=10)
  time.sleep(.1) # Important to sleep between publisher constructor and publish()
  stop_classifier_pub.publish(Empty()) 

### Script Entrypoint
def startNode():
  rospy.loginfo("Starting Object Detection Start automation script", disable_signals=True) # Disable signals so we can force a shutdown
  rospy.init_node(name="obj_detection_start_auto_script")
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

