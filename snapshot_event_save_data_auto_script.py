#!/usr/bin/env python

# Sample NEPI Automation Script. 
# Uses onboard ROS python library to
# 1. Waits for snapshot_event topic message
# 2. Start saving image data to onboard storage
# 3. Delays a specified amount of time, then stops saving
# 4. Delays next trigger event action for some set delay time

import time
import sys
import rospy
import os
import numpy as np
import cv2
from datetime import datetime

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import UInt8, Empty, String, Bool


###########################################################################
# SETUP - Edit as Necessary 
###########################################################################

###!!!!!!!! Set Image ROS Topic Name to Save  !!!!!!!!
IMAGE_INPUT_TOPIC = "/nepi/s2x/nexigo_n60_fhd_webcam_audio/idx/color_2d_image"
#IMAGE_INPUT_TOPIC = "/nepi/s2x/see3cam_cu81/idx/color_2d_image"
#IMAGE_INPUT_TOPIC = "/nepi/s2x/sidus_ss400/idx/color_2d_image"
#IMAGE_INPUT_TOPIC = "/nepi/s2x/onwote_hd_poe/idx/color_2d_image"

###!!!!!!!! Set Automation action parameters !!!!!!!!
SAVE_DURATION_S = 5.0 # Seconds. Length of time to save data
SAVE_DATA_MAX_RATE_HZ = 1.0
SAVE_RESET_DELAY_S = 5.0 # Seconds. Delay before starting over search/save process
SAVE_FOLDER_NAME = "snapshot_event/"
SAVE_FILE_PREFIX = "snapshot_event"
SAVE_IMAGE_TYPE = "jpg"

# NEPI ROS namespace setup
NEPI_BASE_NAMESPACE = "/nepi/s2x/"
### Snapshot Topic Name
SNAPSHOT_TOPIC = NEPI_BASE_NAMESPACE + "snapshot_event"

#####################################################################################
# Globals
#####################################################################################
save_folder = "/mnt/nepi_storage/data/" + SAVE_FOLDER_NAME
save_data_min_interval_s = float(1.0)/SAVE_DATA_MAX_RATE_HZ
save_data_enable = False
last_save_time_s = None

#####################################################################################
# Methods
#####################################################################################

### System Initialization processes
def initialize_actions():
  global save_folder
  print("")
  print("Starting Initialization")  
  # Check if camera topic is publishing
  topic_list=rospy.get_published_topics(namespace='/')
  topic_to_connect=[IMAGE_INPUT_TOPIC, 'sensor_msgs/Image']
  while topic_to_connect not in topic_list:
    print("!!!!! Image topic not found, waiting 1 second")
    time.sleep(1)
  print("Image topic found")
  # Create save folder if needed
  print("Checking Save Folder")
  Save_Folder_Exist = os.path.exists(save_folder)
  if not Save_Folder_Exist:
    print("Creating save folder")
    os.makedirs(save_folder)
  else:
    print("Save folder exists")
  print("Initialization Complete")
  print("Waiting for snapshot event trigger topic to publish on:")
  print(SNAPSHOT_TOPIC)
  

# Action upon detection of snapshot event trigger
def snapshot_event_callback(event):
  global save_data_enable
  # Start saving data
  print("Enabling data saving")
  save_data_enable = True
  print("Saving for " + str(SAVE_DURATION_S) + " secs")
  time.sleep(SAVE_DURATION_S)
  print("Disabling data saving")
  save_data_enable = False
  # Delay next trigger
  print("Delaying next trigger for " + str(SAVE_RESET_DELAY_S) + " secs")
  time.sleep(SAVE_RESET_DELAY_S)
  print("Waiting for next snapshot event trigger")


### Callback to save images
def image_saver_callback(img_msg):
  global save_data_enable
  global save_data_min_interval_s
  global last_save_time_s
  global save_folder
  if save_data_enable:
    if last_save_time_s is None:
      last_save_time_s =  time.time() - save_data_min_interval_s - 1
    timer = time.time()- last_save_time_s
    if timer > save_data_min_interval_s: # Save Image
      #Convert image from ros to cv2
      bridge = CvBridge()
      cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
      # Saving image to file type
      dt_str=datetime.utcnow().strftime('%Y%m%d-%H%M%S%f')[:-3]
      filename=save_folder + SAVE_FILE_PREFIX + '_' + dt_str + '.' + SAVE_IMAGE_TYPE
      print("Saving image to file")
      print(filename)
      cv2.imwrite(filename,cv_image)
      last_save_time_s =  time.time() # Reset last save time
  else:
    last_save_time = None
    

### Cleanup processes on node shutdown
def cleanup_actions():
  print("Shutting down: Executing script cleanup actions")
  time.sleep(.1)

### Script Entrypoint
def startNode():
  rospy.loginfo("Starting Snapshot Event Detect and Save automation script", disable_signals=True) # Disable signals so we can force a shutdown
  rospy.init_node(name="snapshot_event_detect_save_auto_script")
  # Run Initialization processes
  initialize_actions()
  # Start image saver callback
  rospy.Subscriber(IMAGE_INPUT_TOPIC, Image, image_saver_callback, queue_size = 1)
  # Set up snapshot event callback
  rospy.Subscriber(SNAPSHOT_TOPIC, Empty, snapshot_event_callback, queue_size = 1)
  #Set up cleanup on node shutdown
  rospy.on_shutdown(cleanup_actions)
  # Spin forever (until object is detected)
  rospy.spin()


#####################################################################################
# Main
#####################################################################################

if __name__ == '__main__':
  startNode()

