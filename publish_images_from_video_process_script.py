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
Video_Folder= Current_Path + "/sample_data"
Video_File = "sample_avi_960x400_ocean.avi"

Publish_Image_Encoding = "bgr8"  # "bgr8" or "mono8"
Publish_Image_Topic_Name =  "video_images"

#########################################
# ROS NAMESPACE SETUP
#########################################

# ROS namespace setup
NEPI_BASE_NAMESPACE = nepi_ros.get_base_namespace()

#########################################
# Node Class
#########################################

class publish_images_from_video_process(object):
  count = 0
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
    if os.path.exists(Video_Folder):
      self.file2open = (Video_Folder + '/' + Video_File)
      if os.path.isfile(self.file2open):
        print("Opening File: " + self.file2open)
        self.vidcap = cv2.VideoCapture(self.file2open)
        if self.vidcap.isOpened() == True:
          success,image = self.vidcap.read()
          shape_str = str(image.shape)
          print('Image size: ' + shape_str)
          fps = self.vidcap.get(5)
          print('Frames per second : ', fps,'FPS')

          frame_count = self.vidcap.get(7)
          print('Frame count : ', frame_count)

          if success:
            # Setup custom image publish topic
            IMAGE_OUTPUT_TOPIC = NEPI_BASE_NAMESPACE + Publish_Image_Topic_Name
            print("Publishing image to topic: " + IMAGE_OUTPUT_TOPIC)
            self.image_pub = rospy.Publisher(IMAGE_OUTPUT_TOPIC, Image, queue_size=10)
            # Start a timed image publish callback
            pub_interval_sec = float(1)/fps
            print("Publishing image at " + str(fps) + " Hz")
            rospy.Timer(rospy.Duration(pub_interval_sec), self.image_publish_callback)
          else:
            print("Unable to grap image from video file")
            rospy.signal_shutdown("Unable to grap image from video file")
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
    success,cv_image = self.vidcap.read()
    if success == False:
      self.vidcap.release()
      time.sleep(1)
      self.vidcap = cv2.VideoCapture(self.file2open)
    else: 
      # Publish new image to ros
      img_out_msg = nepi_img.cv2img_to_rosimg(cv_image,encoding=Publish_Image_Encoding)
      if not rospy.is_shutdown():
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


