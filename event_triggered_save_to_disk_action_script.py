#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

# Sample NEPI Action Script. 
# 1. Waits for event_event topic message
# 2. Starts saving image data to onboard storage
# 3. Delays a specified amount of time, then stops saving
# 4. Delays next trigger event action for some set delay time



import time
import sys
import rospy
import os
import numpy as np
import cv2
import json
from nepi_edge_sdk_base import nepi_ros 

from datetime import datetime
from std_msgs.msg import UInt8, Empty, String, Bool, Float64MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from nepi_ros_interfaces.msg import NavPose
from nepi_ros_interfaces.srv import NavPoseQuery, NavPoseQueryRequest


#########################################
# USER SETTINGS - Edit as Necessary 
#########################################

SAVE_NAVPOSE_DATA_ENABLED = True # True-Save NavPose File  with each image


#Set Image Type to Save
IMAGE_INPUT_TOPIC_NAME = "color_2d_image"

###!!!!!!!! Set Automation action parameters !!!!!!!!
SAVE_DURATION_S = 5.0 # Seconds. Length of time to save data
SAVE_DATA_MAX_RATE_HZ = 1.0
SAVE_RESET_DELAY_S = 30.0 # Seconds. Delay before starting over search/save process
SAVE_FOLDER_NAME = "event_event/" # Use "" to ignore
SAVE_FILE_PREFIX = "event_event" # Use "" to ignore
SAVE_IMAGE_TYPE = "jpg"

TIGGER_RESET_DELAY_S = 1.0



#########################################
# ROS NAMESPACE SETUP
#########################################

# ROS namespace setup
NEPI_BASE_NAMESPACE = nepi_ros.get_base_namespace()

#########################################
# Node Class
#########################################

class event_triggered_save_to_disk_action(object):

  #######################
  ### Node Initialization
  def __init__(self):
    rospy.loginfo("Starting Initialization Processes")
    ## Initialize Class Variables
    self.save_folder = "/mnt/nepi_storage/data/" + SAVE_FOLDER_NAME
    self.save_data_min_interval_s = float(1.0)/SAVE_DATA_MAX_RATE_HZ
    self.save_data_enable = False
    self.last_save_time_s = None
    self.image_topic_to_save = None
    ## Define Class Namespaces
    EVENT_TOPIC = NEPI_BASE_NAMESPACE + "event_trigger"
    # NEPI Get NAVPOSE Solution Service Name
    self.NAVPOSE_SERVICE_NAME = NEPI_BASE_NAMESPACE + "nav_pose_query"
    ## Define Class Services Calls
    ## Create Class Sevices    
    ## Create Class Publishers
    ## Start Class Subscribers
    rospy.loginfo("Waiting for topic type: " + IMAGE_INPUT_TOPIC_NAME)
    topic_name=nepi_ros.wait_for_topic(IMAGE_INPUT_TOPIC_NAME)
    rospy.loginfo("Found topic: " + topic_name)
    image_topic_to_save = topic_name
    # Start image saver callback
    rospy.Subscriber(image_topic_to_save, Image, self.image_saver_callback, queue_size = 1)
    # Set up event event callback
    rospy.Subscriber(EVENT_TOPIC, Empty, self.event_event_callback, queue_size = 1)
    # Set up event event callback
    rospy.Subscriber(EVENT_TOPIC, Empty, self.event_event_callback, queue_size = 1)
    rospy.loginfo("Subscribed to : " + EVENT_TOPIC)
    ## Start Node Processes
    # Create save folder if needed
    rospy.loginfo("Checking Save Folder")
    save_folder_exists = os.path.exists(self.save_folder)
    if not save_folder_exists:
      rospy.loginfo("Creating save folder")
      access = 0o755
      os.makedirs(self.save_folder,access)
    else:
      rospy.loginfo("Save folder exists")
    ## Initiation Complete
    rospy.loginfo("Initialization Complete")

  #######################
  ### Node Methods

  # Action upon detection of event event trigger
  def event_event_callback(self,event):
    # Start saving data
    rospy.loginfo("Enabling data saving")
    self.save_data_enable = True
    rospy.loginfo("Saving for " + str(SAVE_DURATION_S) + " secs")
    nepi_ros.sleep(SAVE_DURATION_S,100)
    rospy.loginfo("Disabling data saving")
    self.save_data_enable = False
    # Delay next trigger
    rospy.loginfo("Delaying next trigger for " + str(TIGGER_RESET_DELAY_S) + " secs")
    nepi_ros.sleep(TIGGER_RESET_DELAY_S,100)
    rospy.loginfo("Waiting for next event event trigger")


  ### Callback to save images
  def image_saver_callback(self,img_msg):
    if self.save_data_enable:
      # Save data
      if self.last_save_time_s is None:
        self.last_save_time_s =  time.time() - self.save_data_min_interval_s - 1
      timer = time.time()- self.last_save_time_s
      if timer > self.save_data_min_interval_s: # Save Image
        #Convert image from ros to cv2
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
        # Saving image to file type
        dt_str=datetime.utcnow().strftime('%Y%m%d-%H%M%S%f')[:-3]
        image_filename=self.save_folder + SAVE_FILE_PREFIX + '_' + dt_str + '.' + SAVE_IMAGE_TYPE
        rospy.loginfo("Saving image to file")
        rospy.loginfo(image_filename)
        cv2.imwrite(image_filename,cv_image)
        if SAVE_NAVPOSE_DATA_ENABLED: # Save a NavPose JSON file with image     
          #Get Current Lat, Long, Alt, Heading
          if not rospy.is_shutdown():
            # Get current NEPI NavPose data from NEPI ROS nav_pose_query service call
            try:
              get_navpose_service = rospy.ServiceProxy(self.NAVPOSE_SERVICE_NAME, NavPoseQuery)
              nav_pose_response = get_navpose_service(NavPoseQueryRequest())
              #rospy.loginfo(nav_pose_response)
              # Set current navpose
              current_navpose = nav_pose_response.nav_pose
              # Get current location vector (lat, long, alt) in geopoint data with AMSL height
              fix_amsl = nav_pose_response.nav_pose.fix
              # Set current heading in degrees
              current_heading_deg = nav_pose_response.nav_pose.heading.heading
              # Data to be written
              navpose_data = {
                  "Latitude": fix_amsl.latitude,
                  "Longitude": fix_amsl.longitude,
                  "Heading": current_heading_deg,
               }
              # Serializing json
              json_object = json.dumps(navpose_data, indent=2)
              # Writing to sample.json
              navpose_filename=self.save_folder + SAVE_FILE_PREFIX + '_' + dt_str + '.nav'
              rospy.loginfo("Saving nav to file")
              rospy.loginfo(navpose_filename)
              with open(navpose_filename, "w") as outfile:
                  outfile.write(json_object)
            except rospy.ServiceException as e:
              rospy.loginfo("NavPose service call failed: %s"%e)
              nepi_ros.sleep(1,20)
        self.last_save_time_s =  time.time() # Reset last save time
    else:
      self.last_save_time = None
      


  
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











