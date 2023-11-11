#!/usr/bin/env python

# Sample NEPI Automation Script. 
# Uses onboard ROS python library to
# 1. Checks if camera image topic exists
# 2. Sets Camera Resolution
# 2. Starts classifier to that camera
# 3. Exits

import time
import sys
import rospy   

from sensor_msgs.msg import Image
from std_msgs.msg import UInt8, Empty, String, Bool
from nepi_ros_interfaces.msg import ClassifierSelection, StringArray
from darknet_ros_msgs.msg import BoundingBoxes

#####################################################################################
# SETUP - Edit as Necessary ##################################
##########################################


CAM_RES=0 # Number 0-3, 0 Low, 1 Med, 2 High, 3 Ultra

###!!!!!!!! Set Camera ROS Topic Name !!!!!!!!
CAMERA_NAME = "nexigo_n60_fhd_webcam_audio/"
#CAMERA_NAME = "see3cam_cu81/"
#CAMERA_NAME = "sidus_ss400/"
#CAMERA_NAME = "onwote_hd_poe/"

# ROS namespace setup
BASE_NAMESPACE = "/nepi/s2x/"
CAMERA_NAMESPACE = BASE_NAMESPACE + CAMERA_NAME
RESOLUTION_ADJ_TOPIC = CAMERA_NAMESPACE + "idx/set_resolution_mode"
IMAGE_INPUT_TOPIC = CAMERA_NAMESPACE + "idx/color_2d_image"

###!!!!!!!! Set Classifier topics and parameters !!!!!!!!
BOUNDING_BOXES_TOPIC = BASE_NAMESPACE + "classifier/bounding_boxes"
FOUND_OBJECT_TOPIC = BASE_NAMESPACE + "classifier/found_object"
START_CLASSIFIER_TOPIC = BASE_NAMESPACE + "start_classifier"
STOP_CLASSIFIER_TOPIC = BASE_NAMESPACE + "stop_classifier"
DETECTION_MODEL = "common_object_detection"
DETECTION_THRESHOLD = 0.5
OBJ_CENTERED_BUFFER_RATIO = 0.5 # acceptable band about center of image for saving purposes

#####################################################################################
# Globals
#####################################################################################
res_adj_pub = rospy.Publisher(RESOLUTION_ADJ_TOPIC, UInt8, queue_size=10)
stop_classifier_pub = rospy.Publisher(STOP_CLASSIFIER_TOPIC, Empty, queue_size=10)

#####################################################################################
# Methods
#####################################################################################


### Script Entrypoint
def startNode():
  rospy.loginfo("Starting Object Detection Start automation script", disable_signals=True) # Disable signals so we can force a shutdown
  rospy.init_node
  rospy.init_node(name="obj_detection_start_auto_script")
  global res_adj_pub
  global start_classifier_pub
  print("")
  rospy.loginfo("Initializing " + CAMERA_NAMESPACE )
  rospy.loginfo("Connecting to ROS Topic " + IMAGE_INPUT_TOPIC )
  ### Check if camera topic is publishing exists then initialize saving parameters
  topic_list=rospy.get_published_topics(namespace='/')
  topic_to_connect=[IMAGE_INPUT_TOPIC, 'sensor_msgs/Image']
  if topic_to_connect in topic_list: 
    print("Camera topic found, starting initializing process")
    res_adj_pub.publish(CAM_RES)
    time.sleep(1)
    ### Classifier initialization, and wait for it to publish
    start_classifier_pub = rospy.Publisher(START_CLASSIFIER_TOPIC, ClassifierSelection, queue_size=1)
    classifier_selection = ClassifierSelection(img_topic=IMAGE_INPUT_TOPIC, classifier=DETECTION_MODEL, detection_threshold=DETECTION_THRESHOLD)
    time.sleep(2) # Important to sleep between publisher constructor and publish()
    rospy.loginfo("Starting object detector: " + str(start_classifier_pub.name))
    start_classifier_pub.publish(classifier_selection)
    print("Initialization Complete")
  else: 
    print("!!!!! Camera topic not found, shutting down")
    time.sleep(1)
    rospy.signal_shutdown("Camera topic not found")


#####################################################################################
# Main
#####################################################################################

if __name__ == '__main__':
  startNode()

