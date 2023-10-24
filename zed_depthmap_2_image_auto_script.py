#!/usr/bin/env python

# Sample NEPI Automation Script. 
# Uses onboard ROS python library to
# 1. convert zed stereo camera depthmap image to human viewable color image
# 2. Run until Stopped

import time
import sys
import rospy
import numpy as np
import cv2 as cv
import struct
from PIL import Image as im

from sensor_msgs.msg import Image
from rospy.numpy_msg import numpy_msg
from cv_bridge import CvBridge
from std_msgs.msg import UInt8, Empty, String, Bool

#####################################################################################
# SETUP - Edit as Necessary ##################################
##########################################

# ROS namespace setup
BASE_NAMESPACE = "/nepi/s2x/"

###!!!!!!!! Set data stream topics and parameters !!!!!!!!
DEPTHMAP_INPUT_TOPIC = BASE_NAMESPACE + "zed2/zed_node/depth/depth_registered/"
IMAGE_OUTPUT_TOPIC = BASE_NAMESPACE + "zed2/zed_node/depth/depth_image"

#####################################################################################
# Globals
#####################################################################################
depth_image_pub = rospy.Publisher(IMAGE_OUTPUT_TOPIC, Image, queue_size=10)
np_depth_array_m=None # will be replaced when depthmap is recieved and converted
img_height=None
img_width=None

#####################################################################################
# Methods
#####################################################################################

### callback to get depthmap, convert to global float array and rgb image, then publish depth_image
def convert_depthmap_callback(ros_depth_data,min_range_m=0.25,max_range_m=5):
  global np_depth_array_m
  global img_height
  global img_width
  # Zed depth data is floats in m, but passed as 4 bytes each that must be converted to floats
  # Use cv_bridge() to convert the ROS image to OpenCV format
  #Convert the depth 4xbyte data to global float meter array
  bridge = CvBridge()
  cv_depth_image = bridge.imgmsg_to_cv2(ros_depth_data, desired_encoding="passthrough")
  np_depth_array_m = (np.array(cv_depth_image, dtype=np.float32)) # replace nan values
  np_depth_array_m[np.isnan(np_depth_array_m)] = 0
  img_height, img_width = np_depth_array_m.shape[:2]
  # Create thresholded and 255 scaled version
  np_depth_array_scaled = np_depth_array_m
  np_depth_array_scaled[np_depth_array_scaled < min_range_m] = max_range_m # put to max
  np_depth_array_scaled[np_depth_array_scaled > max_range_m] = max_range_m # put to max
  np_depth_array_scaled=np_depth_array_scaled-min_range_m
  depth_scaler=np.amax(np_depth_array_m) # use max range as scalar
  np_depth_array_scaled = np.array(255*np_depth_array_m/depth_scaler,np.uint8)
  max_value=np.max(np_depth_array_scaled)
  np_depth_array_scaled=np.array(np.abs(np_depth_array_scaled-float(max_value)),np.uint8) # Reverse for colormaping
  ### Debug Code
  #cv.imwrite('/mnt/nepi_storage/data/image_bw.jpg', np_depth_array_scaled)
  #print(np_depth_array_scaled.shape)


  # Convert to CV color image using colormap
  cv_depth_image_color = cv.applyColorMap(np_depth_array_scaled, cv.COLORMAP_JET)
  ### Debug Code
  #cv.imwrite('/mnt/nepi_storage/data/image_color.jpg', im_color)
  #print(im_color.shape)
  ###
  ros_depth_image = bridge.cv2_to_imgmsg(cv_depth_image_color,"bgr8")
  depth_image_pub.publish(ros_depth_image)

  


### Cleanup processes on node shutdown
def cleanup_actions():
    print("Shutting down: Executing script cleanup actions")


### Script Entrypoint
def startNode():
  rospy.loginfo("Starting Image Depthmap to Image script", disable_signals=True) # Disable signals so we can force a shutdown
  rospy.init_node
  rospy.init_node(name="depmap_2_image_auto_script")
  # Start depthmap to image process
  rospy.Subscriber(DEPTHMAP_INPUT_TOPIC, numpy_msg(Image), convert_depthmap_callback)
  # run cleanup actions on shutdown
  rospy.on_shutdown(cleanup_actions)
  # Spin forever
  rospy.spin()


#####################################################################################
# Main
#####################################################################################

if __name__ == '__main__':
  startNode()

