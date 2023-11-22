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

###!!!!!!!! Set Image ROS Topic Name to Use  !!!!!!!!
IMAGE_INPUT_TOPIC = "/nepi/s2x/nexigo_n60_fhd_webcam_audio/idx/color_2d_image"
#IMAGE_INPUT_TOPIC = "/nepi/s2x/see3cam_cu81/idx/color_2d_image"
#IMAGE_INPUT_TOPIC = "/nepi/s2x/sidus_ss400/idx/color_2d_image"
#IMAGE_INPUT_TOPIC = "/nepi/s2x/onwote_hd_poe/idx/color_2d_image"
#IMAGE_INPUT_TOPIC = "/nepi/s2x/zed2/zed_node/left/image_rect_color"

# ROS namespace setup
NEPI_BASE_NAMESPACE = "/nepi/s2x/"

IMAGE_OUTPUT_TOPIC = NEPI_BASE_NAMESPACE + "image_custom"

#####################################################################################
# Globals
#####################################################################################
custom_image_pub = rospy.Publisher(IMAGE_OUTPUT_TOPIC, Image, queue_size=10)

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
  print("Initialization Complete")


### Add your CV2 image customization code here
def image_customization_callback(img_msg):
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
  if not rospy.is_shutdown():
    custom_image_pub.publish(img_out_msg)
    # You can view the enhanced_2D_image topic at 
    # //192.168.179.103:9091/ in a connected web browser
  

### Cleanup processes on node shutdown
def cleanup_actions():
  global contour_image_pub
  print("Shutting down: Executing script cleanup actions")
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

