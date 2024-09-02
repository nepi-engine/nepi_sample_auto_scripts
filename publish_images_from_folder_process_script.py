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
# 1. Opens folder and publish image files at 1 Hz
# 2. Repeats from beginning

import os
import time
import sys
import rospy
import numpy as np
import cv2

from sensor_msgs.msg import Image
from nepi_edge_sdk_base import nepi_ros
from nepi_edge_sdk_base import nepi_img

#########################################
# USER SETTINGS - Edit as Necessary 
#########################################
Current_Path = (os.path.dirname(os.path.realpath(__file__)) )

### Open and publish images
Source_Folder= Current_Path + "/sample_data"
Source_Folder= "/mnt/nepi_storage/data_labeling/HardHats/Images"
File_Types = ['png','PNG','jpg','jpeg','JPG']

Publish_Image_Size = [800,600]
Publish_Image_Encoding = "bgr8"  # "bgr8", "rgb8", or "mono8"
Publish_Images_Topic_Name = "test_images"




#########################################
# ROS NAMESPACE SETUP
#########################################

# ROS namespace setup
NEPI_BASE_NAMESPACE = nepi_ros.get_base_namespace()


#########################################
# Node Class
#########################################

class publish_images_from_folder_process(object):
  current_file_ind = 0
  num_files = 0
  
  #######################
  ### Node Initialization
  def __init__(self):
    
    rospy.loginfo("Starting Initialization Processes")
    ## Initialize Class Variables
    ## Define Class Namespaces
    IMAGE_OUTPUT_TOPIC = NEPI_BASE_NAMESPACE + Publish_Images_Topic_Name
    ## Define Class Services Calls
    ## Create Class Sevices    
    ## Create Class Publishers
    self.image_pub = rospy.Publisher(IMAGE_OUTPUT_TOPIC, Image, queue_size=10)
    nepi_ros.sleep(1,10) # Wait for Publisher to setup
    ## Start Class Subscribers
    ## Start Node Processes
    # Get list of image files in the folder
    self.file_list = []
    self.num_files = 0
    if os.path.exists(Source_Folder):
      for f_type in File_Types:
        [file_list, num_files] = nepi_ros.get_file_list(Source_Folder,f_type)
        self.file_list.extend(file_list)
        self.num_files += num_files
      if self.num_files > 0:
        pub_interval_sec = 1
        rospy.Timer(rospy.Duration(pub_interval_sec), self.image_publish_callback)
      else:
        print("No image files found in folder " + Source_Folder + " not found")
    else:
      print("Folder " + Source_Folder + " not found")
    ## Initiation Complete
    rospy.loginfo("Initialization Complete")

  #######################
  ### Node Methods

  ### Add your CV2 image customization code here
  def image_publish_callback(self,timer):
    if self.current_file_ind > (self.num_files-1):
      self.current_file_ind = 0 # Start over
    file2open = self.file_list[self.current_file_ind]
    self.current_file_ind = self.current_file_ind + 1
    print("Opening File: " + file2open)
    cv_image = cv2.imread(file2open)
    shape = cv_image.shape
    print(shape)
    cv_image = cv2.resize(cv_image,(Publish_Image_Size[0],Publish_Image_Size[1]))
    #Convert image from cv2 to ros
    img_out_msg = nepi_img.cv2img_to_rosimg(cv_image,encoding=Publish_Image_Encoding)
    # Publish new image to ros
    if not rospy.is_shutdown():
      img_out_msg.header.stamp = rospy.Time.now()
      self.image_pub.publish(img_out_msg) 


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

