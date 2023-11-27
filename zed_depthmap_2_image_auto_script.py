#!/usr/bin/env python


__author__ = "Jason Seawall"
__copyright__ = "Copyright 2023, Numurus LLC"
__email__ = "nepi@numurus.com"
__credits__ = ["Jason Seawall", "Josh Maximoff"]

__license__ = "GPL"
__version__ = "2.0.4.0"


# Sample NEPI Automation Script. 
# Uses onboard ROS python library to
# 1. convert/publish zed stereo camera depthmap image to human viewable color image
# 2. Run until Stopped

# Requires the following additional scripts are running
# a)ai_detector_setup_start_auto_script.py
# These scripts are available for download at:
# [link text](https://github.com/numurus-nepi/nepi_sample_auto_scripts)

import time
import sys
import rospy
import numpy as np
import cv2

from sensor_msgs.msg import Image
from rospy.numpy_msg import numpy_msg
from cv_bridge import CvBridge
from std_msgs.msg import UInt8, Empty, String, Bool

#####################################################################################
# SETUP - Edit as Necessary ##################################
##########################################


#Set Runtime User Settings
DEPTH_IMAGE_MIN_RANGE_METERS=0.25 #Set to meter value to adjust depth image active range
DEPTH_IMAGE_MAX_RANGE_METERS=2.0 #Set to meter value to adjust depth image active range


# ROS namespace setup
NEPI_BASE_NAMESPACE = "/nepi/s2x/"

###!!!!!!!! Set data stream topics and parameters !!!!!!!!
DEPTH_DATA_INPUT_TOPIC = NEPI_BASE_NAMESPACE + "zed2/zed_node/depth/depth_registered"
DEPTH_IMAGE_OUTPUT_TOPIC = NEPI_BASE_NAMESPACE + "zed2/zed_node/depth/depth_image"

#####################################################################################
# Globals
#####################################################################################
depth_image_pub = rospy.Publisher(DEPTH_IMAGE_OUTPUT_TOPIC, Image, queue_size=10)

#####################################################################################
# Methods
#####################################################################################


### System Initialization processes
def initialize_actions():
  print("")
  print("Starting Initialization")  
  # Wait for topic
  print("Waiting for topic: " + DEPTH_DATA_INPUT_TOPIC)
  wait_for_topic(DEPTH_DATA_INPUT_TOPIC, 'sensor_msgs/Image')
  print("Initialization Complete")

### callback to get depthmap, convert to global float array of meter depths corrisponding to image pixel location
def convert_depthmap_callback(ros_depth_data):
  global depth_image_pub
  # Zed depth data is floats in m, but passed as 4 bytes each that must be converted to floats
  # Use cv2_bridge() to convert the ROS image to OpenCV format
  #Convert the depth 4xbyte data to global float meter array
  cv2_bridge = CvBridge()
  cv2_depth_image = cv2_bridge.imgmsg_to_cv2(ros_depth_data, desired_encoding="passthrough")
  np_depth_array_m = (np.array(cv2_depth_image, dtype=np.float32)) # replace nan values
  np_depth_array_m[np.isnan(np_depth_array_m)] = 0
  ##################################################
  # Turn depth_array_m into colored image and publish
  # Create thresholded and 255 scaled version
  min_range_m=DEPTH_IMAGE_MIN_RANGE_METERS
  max_range_m=DEPTH_IMAGE_MAX_RANGE_METERS
  np_depth_array_scaled = np_depth_array_m
  np_depth_array_scaled[np_depth_array_scaled < min_range_m] = 0
  np_depth_array_scaled[np_depth_array_scaled > max_range_m] = 0
  np_depth_array_scaled=np_depth_array_scaled-min_range_m
  max_value=np.max(np_depth_array_scaled)
  np_depth_array_scaled=np.array(np.abs(np_depth_array_scaled-float(max_value)),np.uint8) # Reverse for colormaping
  depth_scaler=max_range_m-min_range_m
  np_depth_array_scaled = np.array(255*np_depth_array_m/depth_scaler,np.uint8)

  ## Debug Code ###
  ##cv2.imwrite('/mnt/nepi_storage/data/image_bw.jpg', np_depth_array_scaled)
  ##print(np_depth_array_scaled.shape)
  ##time.sleep(1)
  #################
  # Apply colormap
  cv2_depth_image_color = cv2.applyColorMap(np_depth_array_scaled, cv2.COLORMAP_JET)
  ## Debug Code ###
  ##cv2.imwrite('/mnt/nepi_storage/data/image_color.jpg', im_color)
  ##print(im_color.shape)
  ##time.sleep(1)
  #################
  # Convert to cv2 image to Ros Image message
  ros_depth_image = cv2_bridge.cv2_to_imgmsg(cv2_depth_image_color,"bgr8")
  # Publish new image to ros
  if not rospy.is_shutdown():
    depth_image_pub.publish(ros_depth_image)

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
  global depth_image_pub
  print("Shutting down: Executing script cleanup actions")
  # Unregister publishing topics
  depth_image_pub.unregister()

### Script Entrypoint
def startNode():
  rospy.loginfo("Starting Image Depthmap to Image script", disable_signals=True) # Disable signals so we can force a shutdown
  rospy.init_node
  rospy.init_node(name="depmap_2_image_auto_script")
  # Run initialization processes
  initialize_actions()
  print("Starting convert depthmap subscriber")
  rospy.Subscriber(DEPTH_DATA_INPUT_TOPIC, numpy_msg(Image), convert_depthmap_callback, queue_size = 1)
  # run cleanup actions on shutdown
  rospy.on_shutdown(cleanup_actions)
  # Spin forever
  rospy.spin()


#####################################################################################
# Main
#####################################################################################

if __name__ == '__main__':
  startNode()

