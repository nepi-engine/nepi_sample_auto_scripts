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
# 1. Open an image file and publish as ros image message at specified rate
# 2. Run until Stopped

import os
import time
import sys
import rospy
import numpy as np
import cv2
from nepi_edge_sdk_base import nepi_ros
from nepi_edge_sdk_base import nepi_img

from sensor_msgs.msg import Image

#########################################
# USER SETTINGS - Edit as Necessary 
#########################################
Current_Path = (os.path.dirname(os.path.realpath(__file__)) )

### Open and publish image
Image_Folder= Current_Path + "/sample_data"
Image_File = "test_image.png"

Publish_Image_Topic_Name = "test_image"
Publish_Image_Rate_Hz = 20

#########################################
# ROS NAMESPACE SETUP
#########################################

# ROS namespace setup
NEPI_BASE_NAMESPACE = nepi_ros.get_base_namespace()

#########################################
# Node Class
#########################################

class publish_image_from_file_process(object):
  #######################
  ### Node Initialization
  def __init__(self):
    rospy.loginfo("Starting Initialization Processes")
    ## Initialize Class Variables
    ## Define Class Namespaces
    ## Define Class Services Calls
    ## Create Class Sevices    
    ## Create Class Publishers
    # Open image file
    if os.path.exists(Image_Folder):
      file2open = (Image_Folder + '/' + Image_File)
      if os.path.isfile(file2open):
        print("Opening File: " + file2open)
        cv_image = cv2.imread(file2open)
        print(cv_image.shape)
        self.img_out_msg = nepi_img.cv2img_to_rosimg(cv_image)
        # Setup custom image publish topic
        IMAGE_OUTPUT_TOPIC = NEPI_BASE_NAMESPACE + "image_enahnaced"
        print("Publishing image to topic: " + Publish_Image_Topic_Name)
        self.image_pub = rospy.Publisher(Publish_Image_Topic_Name, Image, queue_size=10)
        # Start a timed image publish callback
        pub_interval_sec = float(1)/Publish_Image_Rate_Hz
        print("Publishing image at " + str(Publish_Image_Rate_Hz) + " Hz")
        rospy.Timer(rospy.Duration(pub_interval_sec), self.image_publish_callback)
      else:
        print("File not found in specified folder")
    else:
      print("Path not found")
    ## Start Node Processes
    ## Initiation Complete
    rospy.loginfo("Initialization Complete")

  #######################
  ### Node Methods

  ### Add your CV2 image customization code here
  def image_publish_callback(self,timer):
    # Publish new image to ros
    if not rospy.is_shutdown():
      self.image_pub.publish(self.img_out_msg) 



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


