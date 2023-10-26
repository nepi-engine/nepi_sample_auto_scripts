#!/usr/bin/env python

# Sample NEPI Automation Script. 
# Uses onboard ROS python library to
# 1. Converts ROS image to OpenCV image
# 2. Blank area for custom code
# 2. Converts OpenCV image back to ROS image


import time
import sys
import rospy
import numpy as np
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import UInt8, Empty, String, Bool

#####################################################################################
# SETUP - Edit as Necessary ##################################
##########################################

CAM_RES=1 # Number 0-3, 0 Low, 1 Med, 2 High, 3 Ultra

# ROS namespace setup
BASE_NAMESPACE = "/nepi/s2x/"

###!!!!!!!! Set Camera topics and parameters !!!!!!!!
CAMERA_NAME = "nexigo_n60_fhd_webcam_audio/"
#CAMERA_NAME = "see3cam_cu81/"
#CAMERA_NAME = "sidus_ss400/"
#CAMERA_NAME = "onwote_hd_poe/"

CAMERA_NAMESPACE = BASE_NAMESPACE + CAMERA_NAME

RESOLUTION_ADJ_TOPIC = CAMERA_NAMESPACE + "idx/set_resolution_mode"
IMAGE_INPUT_TOPIC = CAMERA_NAMESPACE + "idx/color_2d_image"
IMAGE_OUTPUT_TOPIC = CAMERA_NAMESPACE + "idx/custom_image"

#####################################################################################
# Globals
#####################################################################################
publsih_enable=True #  This is turned to False during cleanup
custom_image_pub = rospy.Publisher(IMAGE_OUTPUT_TOPIC, Image, queue_size=10)
res_adj_pub = rospy.Publisher(RESOLUTION_ADJ_TOPIC, UInt8, queue_size=10)

#####################################################################################
# Methods
#####################################################################################

### System Initialization processes
def initialize_actions():
  print("")
  rospy.loginfo("Initializing " + CAMERA_NAMESPACE )
  rospy.loginfo("Connecting to ROS Topic " + IMAGE_INPUT_TOPIC )
  # Check if camera topic is publishing
  topic_list=rospy.get_published_topics(namespace='/')
  topic_to_connect=[IMAGE_INPUT_TOPIC, 'sensor_msgs/Image']
  if topic_to_connect in topic_list: 
    print("Camera topic found, starting initializing process")
    res_adj_pub.publish(CAM_RES)
    time.sleep(1)
    print("Initialization Complete")
  else: 
    print("!!!!! Camera topic not found, shutting down")
    time.sleep(1)
    rospy.signal_shutdown("Camera topic not found")


### Add your CV2 image customization code here
def image_customization_callback(img_msg):
  global publsih_enable
  global contour_image_pub

  #Convert image from ros to cv2
  bridge = CvBridge()
  cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
  # Get contours
  ###########################################################
  ### ADD YOUR CODE HERE
  ###########################################################



  ###########################################################
  ### END OF YOUR CODE
  ###########################################################
  #Convert image from cv2 to ros
  img_out_msg = bridge.cv2_to_imgmsg(cv_image,"bgr8")#desired_encoding='passthrough')
  # Publish new image to ros
  if publsih_enable:
    custom_image_pub.publish(img_out_msg) # You can view the enhanced_2D_image topic at //192.168.179.103:9091/ in a connected web browser
  

### Cleanup processes on node shutdown
def cleanup_actions():
  global publish_enable
  global contour_image_pub
  print("Shutting down: Executing script cleanup actions")
  # Disable all publishers
  publish_enable=False
  time.sleep(2)
  # Unregister publishing topics
  custom_image_pub.unregister()


### Script Entrypoint
def startNode():
  rospy.loginfo("Starting Image Contours Overlay automation script", disable_signals=True) # Disable signals so we can force a shutdown
  rospy.init_node
  rospy.init_node(name="image_contours_overlay_auto_script")
  # Run Initialization processes
  initialize_actions()
  # Start image contours overlay process and pubslisher
  rospy.Subscriber(IMAGE_INPUT_TOPIC, Image, image_customization_callback, queue_size = 1)
  #Set up cleanup on node shutdown
  rospy.on_shutdown(cleanup_actions)
  # Spin forever (until object is detected)
  rospy.spin()


#####################################################################################
# Main
#####################################################################################

if __name__ == '__main__':
  startNode()

