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


# Concept NEPI IDX Driver Script for ZED2 Camera.

###################################################
# NEPI NavPose Axis Info
# x+ axis is forward
# y+ axis is right
# z+ axis is down
# roll: RHR about x axis
# pitch: RHR about y axis
# yaw: RHR about z axis
#####################################################

### Set the namespace before importing rospy
import os
os.environ["ROS_NAMESPACE"] = "/nepi/s2x"

import time
import sys
import rospy
import dynamic_reconfigure.client
import numpy as np
import cv2
import math
import tf

from std_msgs.msg import UInt8, Empty, String, Bool, Float32
from sensor_msgs.msg import Image, PointCloud2
from nepi_ros_interfaces.msg import IDXStatus, RangeWindow
from nepi_ros_interfaces.srv import IDXCapabilitiesQuery, IDXCapabilitiesQueryResponse
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped, QuaternionStamped
from dynamic_reconfigure.msg import Config
from rospy.numpy_msg import numpy_msg
from cv_bridge import CvBridge



#####################################################################################
# SETUP - Edit as Necessary ##################################
##########################################

#Define Sensor Native Parameters
SENSOR_RES_OPTION_LIST = [0,1,2,3]  # Maps to IDX Res Options 0-3
SENSOR_MAX_BRIGHTNESS = 8
SENSOR_MAX_CONTRAST = 8
SENSOR_MAX_FRAMERATE_FPS = 30
SENSOR_MIN_THRESHOLD = 1
SENSOR_MAX_THRESHOLD = 100
SENSOR_MAX_RANGE_M = 20

#Set Initialize IDX Parameters
IDX_RES_MODE = 1
IDX_FRAMERATE_MODE = 3
IDX_BRIGHTNESS_RATIO = 0.5
IDX_CONTRAST_RATIO = 0.5
IDX_THRESHOLD_RATIO = 0.5
IDX_MIN_RANGE_RATIO=0.02 
IDX_MAX_RANGE_RATIO=0.15 

# ROS namespace setup
NEPI_BASE_NAMESPACE = "/nepi/s2x/"
NEPI_IDX_NAME = "zed2_stereo_camera"
NEPI_IDX_NAMESPACE = NEPI_BASE_NAMESPACE + NEPI_IDX_NAME + "/idx/"

ZED_BASE_NAMESPACE = NEPI_BASE_NAMESPACE + "zed2/zed_node/"
# Zed control topics
ZED_PARAMETER_UPDATES_TOPIC = ZED_BASE_NAMESPACE + "parameter_updates"

# Zed data stream topics
ZED_COLOR_2D_IMAGE_TOPIC = ZED_BASE_NAMESPACE + "left/image_rect_color"
ZED_BW_2D_IMAGE_TOPIC = ZED_BASE_NAMESPACE + "left/image_rect_gray"
ZED_DEPTH_MAP_TOPIC = ZED_BASE_NAMESPACE + "depth/depth_registered"
ZED_POINTCLOUD_TOPIC = ZED_BASE_NAMESPACE + "point_cloud/cloud_registered"
ZED_ODOM_TOPIC = ZED_BASE_NAMESPACE + "odom"

### NEPI IDX NavPose Publish Topic
NEPI_IDX_NAVPOSE_ODOM_TOPIC = NEPI_IDX_NAMESPACE + "odom"

# NEPI IDX status and control topics
NEPI_IDX_STATUS_TOPIC = NEPI_IDX_NAMESPACE + "status"
NEPI_IDX_SET_BRIGHTNESS_TOPIC = NEPI_IDX_NAMESPACE + "set_brightness"
NEPI_IDX_SET_CONTRAST_TOPIC = NEPI_IDX_NAMESPACE + "set_contrast"
NEPI_IDX_SET_FRAMERATE_MODE_TOPIC = NEPI_IDX_NAMESPACE + "set_framerate_mode"
NEPI_IDX_SET_RESOLUTION_MODE_TOPIC = NEPI_IDX_NAMESPACE + "set_resolution_mode"
NEPI_IDX_SET_THRESHOLDING_TOPIC = NEPI_IDX_NAMESPACE + "set_thresholding"
NEPI_IDX_SET_RANGE_WINDOW_TOPIC = NEPI_IDX_NAMESPACE + "set_range_window"

# NEPI IDX data stream topics
NEPI_IDX_COLOR_2D_IMAGE_TOPIC = NEPI_IDX_NAMESPACE + "color_2d_image"
NEPI_IDX_BW_2D_IMAGE_TOPIC = NEPI_IDX_NAMESPACE + "bw_2d_image"
NEPI_IDX_DEPTH_MAP_TOPIC = NEPI_IDX_NAMESPACE + "depth_map"
NEPI_IDX_DEPTH_IMAGE_TOPIC = NEPI_IDX_NAMESPACE + "depth_image"
NEPI_IDX_POINTCLOUD_TOPIC = NEPI_IDX_NAMESPACE + "pointcloud"
NEPI_IDX_POINTCLOUD_IMAGE_TOPIC = NEPI_IDX_NAMESPACE + "pointcloud_image"

# NEPI IDX capabilities query service
NEPI_IDX_CAPABILITY_REPORT_SERVICE = NEPI_IDX_NAMESPACE + "capabilities_query"

#####################################################################################
# Globals
#####################################################################################

idx_status_pub = rospy.Publisher(NEPI_IDX_STATUS_TOPIC, IDXStatus, queue_size=1, latch=True)

idx_color_2d_image_pub = rospy.Publisher(NEPI_IDX_COLOR_2D_IMAGE_TOPIC, Image, queue_size=1)
idx_bw_2d_image_pub = rospy.Publisher(NEPI_IDX_BW_2D_IMAGE_TOPIC, Image, queue_size=1)
idx_depth_map_pub = rospy.Publisher(NEPI_IDX_DEPTH_MAP_TOPIC, Image, queue_size=1)
idx_depth_image_pub = rospy.Publisher(NEPI_IDX_DEPTH_IMAGE_TOPIC, Image, queue_size=1)
idx_pointcloud_pub = rospy.Publisher(NEPI_IDX_POINTCLOUD_TOPIC, PointCloud2, queue_size=1)
idx_pointcloud_image_pub = rospy.Publisher(NEPI_IDX_POINTCLOUD_IMAGE_TOPIC, Image, queue_size=1)

idx_navpose_odom_pub = rospy.Publisher(NEPI_IDX_NAVPOSE_ODOM_TOPIC, Odometry, queue_size=1)

#zed_parameter_update_pub = rospy.Publisher(ZED_PARAMETER_UPDATES_TOPIC, Config, queue_size=1)
zed_dynamic_reconfig_client = dynamic_reconfigure.client.Client(ZED_BASE_NAMESPACE, timeout=30)

idx_status_msg=IDXStatus()
idx_capabilities_report = IDXCapabilitiesQueryResponse()

#####################################################################################
# Methods
#####################################################################################

### System Initialization processes
def initialize_actions():
  global idx_status_msg
  print("")
  print("Starting Initialization")
  # Wait for zed odom topic
  ##############################
  print("Waiting for ZED odom message to publish on " + ZED_ODOM_TOPIC)
  # Update Orientation source to our new update orientation publisher
  wait_for_topic(ZED_ODOM_TOPIC)
  rospy.Subscriber(ZED_ODOM_TOPIC, Odometry, idx_odom_topic_callback) 
  # Wait for zed depth topic
  print("Waiting for topic: " + ZED_DEPTH_MAP_TOPIC)
  wait_for_topic(ZED_DEPTH_MAP_TOPIC)
  # Initialize IDX status msg and sensor
  idx_status_msg.resolution_mode = IDX_RES_MODE  # Not sure if this is adjustable
  idx_status_msg.framerate_mode = IDX_FRAMERATE_MODE # Not sure if this is adjustable
  idx_status_msg.brightness = IDX_BRIGHTNESS_RATIO
  update_sensor_brightness(idx_status_msg.brightness)
  idx_status_msg.contrast = IDX_CONTRAST_RATIO
  update_sensor_contrast(idx_status_msg.contrast)  
  idx_status_msg.thresholding = IDX_THRESHOLD_RATIO
  update_sensor_thresholding(IDX_THRESHOLD_RATIO)
  idx_status_msg.range_window.start_range = IDX_MIN_RANGE_RATIO 
  idx_status_msg.range_window.stop_range = IDX_MAX_RANGE_RATIO  
  idx_status_msg.frame_3d = "nepi_center_frame"
  idx_status_pub_callback()
  # Start IDX Subscribers
  rospy.Subscriber(NEPI_IDX_SET_BRIGHTNESS_TOPIC, Float32, idx_set_brightness_callback)
  rospy.Subscriber(NEPI_IDX_SET_CONTRAST_TOPIC, Float32, idx_set_contrast_callback)
  #rospy.Subscriber(NEPI_IDX_SET_FRAMERATE_MODE_TOPIC, UInt8, idx_set_framerate_mode_callback)
  #rospy.Subscriber(NEPI_IDX_SET_RESOLUTION_MODE_TOPIC, UInt8, idx_set_resolution_mode_callback)
  rospy.Subscriber(NEPI_IDX_SET_THRESHOLDING_TOPIC, Float32, idx_set_thresholding_callback)
  rospy.Subscriber(NEPI_IDX_SET_RANGE_WINDOW_TOPIC, RangeWindow, idx_set_range_window_callback)
  # Populate and advertise IDX Capability Report
  global idx_capabilities_report
  idx_capabilities_report.adjustable_resolution = False # Pending callback implementation
  idx_capabilities_report.adjustable_framerate = False # Pending callback implementation
  idx_capabilities_report.adjustable_contrast = True
  idx_capabilities_report.adjustable_brightness = True
  idx_capabilities_report.adjustable_thresholding = True
  idx_capabilities_report.adjustable_range = False # Pending callback implementation
  idx_capabilities_report.has_color_2d_image = True
  idx_capabilities_report.has_bw_2d_image = True
  idx_capabilities_report.has_depth_map = True
  idx_capabilities_report.has_depth_image = True 
  idx_capabilities_report.has_pointcloud_image = False # TODO: Create this data
  idx_capabilities_report.has_pointcloud = True
  rospy.Service(NEPI_IDX_CAPABILITY_REPORT_SERVICE, IDXCapabilitiesQuery, idx_capabilities_query_callback)
  print("Starting Zed IDX subscribers and publishers")
  rospy.Subscriber(ZED_COLOR_2D_IMAGE_TOPIC, Image, color_2d_image_callback, queue_size = 1)
  rospy.Subscriber(ZED_BW_2D_IMAGE_TOPIC, Image, bw_2d_image_callback, queue_size = 1)
  rospy.Subscriber(ZED_DEPTH_MAP_TOPIC, numpy_msg(Image), depth_map_callback, queue_size = 1)
  rospy.Subscriber(ZED_POINTCLOUD_TOPIC, PointCloud2, pointcloud_callback, queue_size = 1)
  print("Initialization Complete")


#######################
# Driver NavPose Publishers Functions

### Callback to publish idx odom topic
def idx_odom_topic_callback(odom_msg):
  global idx_navpose_odom_pub
  # TODO: Need to convert data from zed odom ref frame to nepi ref frame
  if not rospy.is_shutdown():
    idx_navpose_odom_pub.publish(odom_msg)

#######################
# Driver Status Publishers Functions

### function to publish IDX status message and updates
def idx_status_pub_callback():
  global idx_status_msg
  global idx_status_pub
  if not rospy.is_shutdown():
    idx_status_pub.publish(idx_status_msg)

#######################
# Driver Control Subscribers Functions

### callback to get and apply brightness control
def idx_set_brightness_callback(brightness_msg):
  global idx_status_msg
  print(brightness_msg)
  idx_brightness_ratio = brightness_msg.data
  # udpate sensor native values
  update_sensor_brightness(idx_brightness_ratio)
  # publish IDX status update
  idx_status_msg.brightness = idx_brightness_ratio
  idx_status_pub_callback()

def update_sensor_brightness(brightness_ratio):
  # Sensor Specific
  sensor_brightness_val = int(float(SENSOR_MAX_BRIGHTNESS)*brightness_ratio)
  zed_dynamic_reconfig_client.update_configuration({"brightness":sensor_brightness_val})
  
### callback to get and apply contrast control
def idx_set_contrast_callback(contrast_msg):
  global idx_status_msg
  print(contrast_msg)
  idx_contrast_ratio = contrast_msg.data
  # udpate sensor native values
  update_sensor_contrast(idx_contrast_ratio)
  # publish IDX status update
  idx_status_msg.contrast = idx_contrast_ratio
  idx_status_pub_callback()

def update_sensor_contrast(contrast_ratio):
  # Sensor Specific
  sensor_contrast_val = int(float(SENSOR_MAX_CONTRAST)*contrast_ratio)
  zed_dynamic_reconfig_client.update_configuration({"contrast":sensor_contrast_val})
  
def idx_set_thresholding_callback(thresholding_msg):
  global idx_status_msg
  idx_thresholding_ratio = thresholding_msg.data
  # udpate sensor native values
  update_sensor_thresholding(idx_thresholding_ratio)
  # publish IDX status update
  idx_status_msg.thresholding = idx_thresholding_ratio
  idx_status_pub_callback()
  
def update_sensor_thresholding(thresholding_ratio):
  # Sensor specific
  sensor_depth_confidence_val = int((float(SENSOR_MAX_THRESHOLD - SENSOR_MIN_THRESHOLD) * thresholding_ratio) + SENSOR_MIN_THRESHOLD)
  zed_dynamic_reconfig_client.update_configuration({"depth_confidence":sensor_depth_confidence_val})

### callback to get and apply range window controls
def idx_set_range_window_callback(range_window_msg):
  global idx_status_msg
  print(range_window_msg)
  idx_range_window = range_window_msg.range_window
  # TODO: What impact will this have -- depth image colormap range, or something else?
  # publish IDX status update
  idx_status_msg.range_window.start_range = idx_range_window.start_range
  idx_status_msg.range_window.stop_range = idx_range_window.stop_range  
  idx_status_pub_callback()


#######################
# Driver Data Publishers Functions

### callback to get and republish color 2d image
def color_2d_image_callback(image_msg):
  global idx_color_2d_image_pub
  # Publish to IDX namespace
  if not rospy.is_shutdown():
    idx_color_2d_image_pub.publish(image_msg)

### callback to get and republish bw 2d image
def bw_2d_image_callback(image_msg):
  global idx_bw_2d_image_pub
  # Publish to IDX namespace
  if not rospy.is_shutdown():
    idx_bw_2d_image_pub.publish(image_msg)


### callback to get depthmap, republish it, convert it to global float array of meter depths corrisponding to image pixel location
def depth_map_callback(depth_map_msg):
  global idx_status_msg
  global idx_depth_map_pub
  global idx_depth_image_pub
  # Zed depth data is floats in m, but passed as 4 bytes each that must be converted to floats
  # Use cv2_bridge() to convert the ROS image to OpenCV format
  #Convert the depth 4xbyte data to global float meter array
  cv2_bridge = CvBridge()
  cv2_depth_image = cv2_bridge.imgmsg_to_cv2(depth_map_msg, desired_encoding="passthrough")
  np_depth_array_m = (np.array(cv2_depth_image, dtype=np.float32)) # replace nan values
  np_depth_array_m[np.isnan(np_depth_array_m)] = 0
  ##################################################
  # Turn depth_array_m into colored image and publish
  # Create thresholded and 255 scaled version
  min_range_m=idx_status_msg.range_window.start_range * SENSOR_MAX_RANGE_M
  max_range_m=idx_status_msg.range_window.stop_range * SENSOR_MAX_RANGE_M
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
    idx_depth_map_pub.publish(depth_map_msg)
    idx_depth_image_pub.publish(ros_depth_image)

### callback to get and republish point_cloud and image
def pointcloud_callback(pointcloud2_msg):
  global idx_pointcloud_pub
  # Publish to IDX namespace
  if not rospy.is_shutdown():
      idx_pointcloud_pub.publish(pointcloud2_msg)

### callback to provide capabilities report ###
def idx_capabilities_query_callback(_):
  global idx_capabilities_report
  return idx_capabilities_report

#######################
# Process Functions

### Function to Convert Quaternion Attitude to Roll, Pitch, Yaw Degrees
def convert_quat2rpy(xyzw_attitude):
  rpy_attitude_rad = tf.transformations.euler_from_quaternion(xyzw_attitude)
  rpy_attitude_ned_deg = np.array(rpy_attitude_rad) * 180/math.pi
  roll_deg = rpy_attitude_ned_deg[0] 
  pitch_deg = rpy_attitude_ned_deg[1] 
  yaw_deg = rpy_attitude_ned_deg[2]
  return rpy_attitude_ned_deg

### Function to Convert Roll, Pitch, Yaw Degrees to Quaternion Attitude
def convert_rpy2quat(rpy_attitude_ned_deg):
  roll_deg = rpy_attitude_ned_deg[0] 
  pitch_deg = rpy_attitude_ned_deg[1] 
  yaw_deg = rpy_attitude_ned_deg[2]
  xyzw_attitude = tf.transformations.quaternion_from_euler(math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg))
  return xyzw_attitude

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
def check_for_topic(topic_name):
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

  
### Cleanup processes on node shutdown
def cleanup_actions():
  global idx_color_2d_image_pub
  global idx_bw_2d_image_pub
  global idx_depth_image_pub
  global idx_pointcloud_pub
  global idx_pointcloud_image_pub
  print("Shutting down: Executing script cleanup actions")
  # Unregister publishing topics
  idx_color_2d_image_pub.unregister()
  idx_bw_2d_image_pub.unregister()
  idx_depth_image_pub.unregister()
  idx_pointcloud_pub.unregister()
  idx_pointcloud_image_pub.unregister()

### Script Entrypoint
def startNode():
  rospy.loginfo("Starting ZED2 IDX driver script", disable_signals=True) # Disable signals so we can force a shutdown
  rospy.init_node
  rospy.init_node(name= NEPI_IDX_NAME)
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

