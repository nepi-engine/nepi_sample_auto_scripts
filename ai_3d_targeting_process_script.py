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
# 2. Runs a process that convert zed stereo camera depthmap depth_array and depth_image
# 3. Runs a process to calculate range and bearing of detected targets
# 4. Publishes new target range/bearing data topic and image overlay topic
# 5. Runs until Stopped


# Requires the following additional scripts are running
# a)zed2_idx_driver_script.py
# b)ai_detector_config_script.py
# These scripts are available for download at:
# [link text](https://github.com/numurus-nepi/nepi_sample_auto_scripts)

import time
import sys
import rospy
import numpy as np
import cv2
from resources import nepi

from sensor_msgs.msg import Image
from rospy.numpy_msg import numpy_msg
from cv_bridge import CvBridge
from std_msgs.msg import UInt8, Empty, String, Bool
from nepi_ros_interfaces.msg import ClassifierSelection, StringArray, TargetLocalization
from darknet_ros_msgs.msg import BoundingBoxes, ObjectCount

#########################################
# USER SETTINGS - Edit as Necessary 
#########################################

#Set AI Detector Image ROS Topic Name or Partial Name
IMAGE_INPUT_TOPIC_NAME = "color_2d_image"

#Set Runtime User Settings
TARGET_BOX_REDUCTION_PERCENT=50 # Sets the percent of target box around center to use for range calc
TARGET_DEPTH_METERS=0.3 # Sets the depth filter around mean depth to use for range calc
TARGET_MIN_VALUES=10 # Sets the minimum number of valid points to consider for a valid range

#Set 3D Imaging Sensor Info
DEPTH_MAP_INPUT_TOPIC_NAME = "depth_map"
FOV_VERT_DEG=70 # Camera Vertical Field of View (FOV)
FOV_HORZ_DEG=110 # Camera Horizontal Field of View (FOV)

#########################################
# ROS NAMESPACE SETUP
#########################################

# ROS namespace setup
NEPI_BASE_NAMESPACE = nepi.get_base_namespace()

#########################################
# Node Class
#########################################

class ai_3d_targeting_process(object):

  #######################
  ### Node Initialization
  def __init__(self):
    rospy.loginfo("Starting Initialization Processes")
    ## Initialize Class Variables
    self.np_depth_array_m=None # will be replaced when depthmap is recieved and converted
    self.detect_boxes=None
    self.img_height = 0
    self.img_width = 0
    ## Define Class Namespaces
    # Targeting process output topics
    TARGET_DATA_OUTPUT_TOPIC = NEPI_BASE_NAMESPACE + "targeting/targeting_data"
    TARGET_IMAGE_OUTPUT_TOPIC = NEPI_BASE_NAMESPACE + "targeting/targeting_image"
    # Classifier topics and parameters
    BOUNDING_BOXES_TOPIC = NEPI_BASE_NAMESPACE + "classifier/bounding_boxes"
    FOUND_OBJECT_TOPIC = NEPI_BASE_NAMESPACE + "classifier/found_object"
    AI_DETECTION_IMAGE_TOPIC = NEPI_BASE_NAMESPACE + "classifier/detection_image"
    ## Create Class Publishers
    self.target_data_pub = rospy.Publisher(TARGET_DATA_OUTPUT_TOPIC, TargetLocalization, queue_size=10)
    self.target_overlay_pub = rospy.Publisher(TARGET_IMAGE_OUTPUT_TOPIC, Image, queue_size=10)
    ## Start Class Subscribers
    print("Starting Initialization")
    # Wait for topic by name
    print("Waiting for topic name: " + IMAGE_INPUT_TOPIC_NAME)
    image_topic=nepi.wait_for_topic(IMAGE_INPUT_TOPIC_NAME)
    print("Found topic: " + image_topic)
    rospy.Subscriber(image_topic, Image, self.object_targeting_callback, queue_size = 1)
    print("Initialization Complete")
    # Wait for topic by name
    print("Waiting for topic name: " + DEPTH_MAP_INPUT_TOPIC_NAME)
    depth_map_topic=nepi.wait_for_topic(DEPTH_MAP_INPUT_TOPIC_NAME)
    print("Found topic: " + depth_map_topic)
    rospy.Subscriber(depth_map_topic, numpy_msg(Image), self.get_depth_data_callback, queue_size = 1)
    print("Starting targeteting subscriber")
    # Wait for topic
    print("Waiting for topic: " + AI_DETECTION_IMAGE_TOPIC)
    nepi.wait_for_topic(AI_DETECTION_IMAGE_TOPIC)
    # Start Subscriber Topics
    print("Starting found object subscriber")
    rospy.Subscriber(FOUND_OBJECT_TOPIC, ObjectCount, self.found_object_callback, queue_size = 1)
    print("Starting object detection subscriber")
    rospy.Subscriber(BOUNDING_BOXES_TOPIC, BoundingBoxes, self.object_detected_callback, queue_size = 1)
    ## Start Node Processes
    ## Initiation Complete
    rospy.loginfo("Initialization Complete")

  #######################
  ### Node Methods

  ### Monitor Output of AI model to clear detection status
  def found_object_callback(self,found_obj_msg):
    # Must reset target lists if no targets are detected
    if found_obj_msg.count == 0:
      #print("No objects detected")
      self.detect_boxes=None

  ### If object(s) detected, save bounding box info to global
  def object_detected_callback(self,bounding_box_msg):
    #print("Objects detected, passsing to targeting process")
    self.detect_boxes=bounding_box_msg
      

  ### callback to get depthmap, convert to global float array and rgb image, then publish depth_image
  def get_depth_data_callback(self,depth_data):
    # Zed depth data is floats in m, but passed as 4 bytes each that must be converted to floats
    # Use cv2_bridge() to convert the ROS image to OpenCV format
    #Convert the depth 4xbyte data to global float meter array
    cv2_bridge = CvBridge()
    cv2_depth_image = cv2_bridge.imgmsg_to_cv2(depth_data, desired_encoding="passthrough")
    self.np_depth_array_m = (np.array(cv2_depth_image, dtype=np.float32)) # replace nan values
    self.np_depth_array_m[np.isnan(self.np_depth_array_m)] = 0 # zero pixels with no value
    self.np_depth_array_m[np.isinf(self.np_depth_array_m)] = 0 # zero pixels with inf value


  ### calculate range and bearing of AI detected targets
  def object_targeting_callback(self,img_msg):
    # Convert ROS image to OpenCV for editing
    self.img_height = img_msg.height
    self.img_width = img_msg.width
    cv2_bridge = CvBridge()
    cv2_image = cv2_bridge.imgmsg_to_cv2(img_msg, "bgr8")
    # Iterate over all of the objects and calculate range and bearing data
    if self.detect_boxes is not None:
      for box in self.detect_boxes.bounding_boxes:
        # Get target label
        target_label=box.Class
        # reduce target box based on user settings
        box_reduction_y_pix=int(float((box.ymax - box.ymin))*float(TARGET_BOX_REDUCTION_PERCENT)/100/2)
        box_reduction_x_pix=int(float((box.xmax - box.xmin))*float(TARGET_BOX_REDUCTION_PERCENT)/100/2)
        ymin_adj=int(box.ymin + box_reduction_y_pix)
        ymax_adj=int(box.ymax - box_reduction_y_pix)
        xmin_adj=box.xmin + box_reduction_x_pix
        xmax_adj=box.xmax - box_reduction_x_pix
        # Calculate target range
        if self.np_depth_array_m is not None:
          # Get target range from cropped and filtered depth data
          depth_box_adj= self.np_depth_array_m[ymin_adj:ymax_adj,xmin_adj:xmax_adj]
          depth_mean_val=np.mean(depth_box_adj)
          depth_array=depth_box_adj.flatten()
          min_filter=depth_mean_val-TARGET_DEPTH_METERS/2
          max_filter=depth_mean_val+TARGET_DEPTH_METERS/2
          depth_array=depth_array[depth_array > min_filter]
          depth_array=depth_array[depth_array < max_filter]
          depth_len=len(depth_array)
          if depth_len > TARGET_MIN_VALUES:
            target_range_m=np.mean(depth_box_adj)
          else:
            target_range_m=float(-999) # NEPI standard unset value
        else:
          target_range_m=float(-999)  # NEPI standard unset value
        # Calculate target bearings
        object_loc_y_pix = float(box.ymin + ((box.ymax - box.ymin))  / 2) 
        object_loc_x_pix = float(box.xmin + ((box.xmax - box.xmin))  / 2)
        object_loc_y_ratio_from_center = float(object_loc_y_pix - self.img_height/2) / float(self.img_height/2)
        object_loc_x_ratio_from_center = float(object_loc_x_pix - self.img_width/2) / float(self.img_width/2)
        target_vert_angle_deg = -(object_loc_y_ratio_from_center * float(FOV_VERT_DEG/2))
        target_horz_angle_deg = (object_loc_x_ratio_from_center * float(FOV_HORZ_DEG/2))
        ### Print the range and bearings for each detected object
  ##      print(target_label)
  ##      print(str(depth_box_adj.shape) + " detection box size")
  ##      print(str(depth_len) + " valid depth readings")
  ##      print("%.2f" % target_range_m + "m : " + "%.2f" % target_horz_angle_deg + "d : " + "%.2f" % target_vert_angle_deg + "d : ")
  ##      print("")
        ###### Publish Targeting_Data ROS Message
        target_data_msg=TargetLocalization()
        target_data_msg.name=target_label
        target_data_msg.range_m=target_range_m
        target_data_msg.azimuth_deg=target_horz_angle_deg
        target_data_msg.elevation_deg=target_vert_angle_deg
        if not rospy.is_shutdown():
          self.target_data_pub.publish(target_data_msg)
        ###### Apply Image Overlays and Publish Targeting_Image ROS Message
        # Overlay adjusted detection boxes on image 
        start_point = (xmin_adj, ymin_adj)
        end_point = (xmax_adj, ymax_adj)
        cv2.rectangle(cv2_image, start_point, end_point, color=(255,0,0), thickness=2)
        # Overlay text data on OpenCV image
        font                   = cv2.FONT_HERSHEY_SIMPLEX
        fontScale              = 0.5
        fontColor              = (0, 255, 0)
        thickness              = 1
        lineType               = 1
       # Overlay Label
        text2overlay=box.Class
        bottomLeftCornerOfText = (int(object_loc_x_pix),int(object_loc_y_pix))
        cv2.putText(cv2_image,text2overlay, 
          bottomLeftCornerOfText, 
          font, 
          fontScale,
          fontColor,
          thickness,
          lineType)
        # Overlay Data
        if target_range_m == -999:
          t_range_m = np.nan
        else:
          t_range_m = target_range_m
        text2overlay="%.1f" % t_range_m + "m," + "%.f" % target_horz_angle_deg + "d," + "%.f" % target_vert_angle_deg + "d"
        bottomLeftCornerOfText = (int(object_loc_x_pix),int(object_loc_y_pix)+15)
        cv2.putText(cv2_image,text2overlay, 
          bottomLeftCornerOfText, 
          font, 
          fontScale,
          fontColor,
          thickness,
          lineType)
    #Convert OpenCV image to ROS image
    img_out_msg = cv2_bridge.cv2_to_imgmsg(cv2_image,"bgr8")#desired_encoding='passthrough')
    # Publish new image to ros
    if not rospy.is_shutdown():
      self.target_overlay_pub.publish(img_out_msg)

  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    rospy.loginfo("Shutting down: Executing script cleanup actions")



#########################################
# Main
#########################################
if __name__ == '__main__':
  current_filename = sys.argv[0].split('/')[-1]
  current_filename = current_filename.split('.')[0]
  rospy.loginfo(("Starting " + current_filename), disable_signals=True) # Disable signals so we can force a shutdown
  rospy.init_node(name=current_filename)
  #Launch the node
  node_name = current_filename.rpartition("_")[0]
  rospy.loginfo("Launching node named: " + node_name)
  node_class = eval(node_name)
  node = node_class()
  #Set up node shutdown
  rospy.on_shutdown(node.cleanup_actions)
  # Spin forever (until object is detected)
  rospy.spin()

