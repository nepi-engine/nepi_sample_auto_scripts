#!/usr/bin/env python

# Sample NEPI Automation Script. 
# Uses onboard ROS python library to
### Expects Classifier to be running ###
# 1. Wait for specific object to be detected and centered
# 2. Publishes a snapshot event trigger out
# 3. Delays trigger event for some set delay time

# Requires the following additional scripts are running
# a)ai_detector_setup_start_auto_script.py
# b)snapshot_event_save_data_auto_script.py
# c) (Optional) snapshot_event_send_to_cloud_auto_script.py for cloud portal support
# These scripts are available for download at:
# [link text](https://github.com/numurus-nepi/nepi_sample_auto_scripts)

import time
import sys
import rospy   

from std_msgs.msg import Empty
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBoxes

#####################################################################################
# SETUP - Edit as Necessary ##################################
##########################################

###!!!!!!!! Set Automation action parameters !!!!!!!!
OBJ_LABEL_OF_INTEREST = "person"
OBJ_CENTERED_BUFFER_RATIO = 0.5 # acceptable band about center of image for saving purposes
RESET_DELAY_S = 5 # Min delay between triggers

# ROS namespace setup
BASE_NAMESPACE = "/nepi/s2x/"
CL_BOUNDING_BOXES_TOPIC = BASE_NAMESPACE + "classifier/bounding_boxes"
CL_DETECTION_IMAGE_TOPIC = BASE_NAMESPACE + "classifier/detection_image"
SNAPSHOT_TOPIC = BASE_NAMESPACE + "snapshot_event"

#####################################################################################
# Globals
#####################################################################################
snapshot_trigger_pub = rospy.Publisher(SNAPSHOT_TOPIC, Empty, queue_size = 1)

img_width = 0 # Updated on receipt of first image
img_height = 0 # Updated on receipt of first image

#####################################################################################
# Methods
#####################################################################################

### System Initialization processes
def initialize_actions():
  global img_height
  global img_width
  print("")
  rospy.loginfo("Initializing " )
  rospy.loginfo("Connecting to ROS Topic " + CL_DETECTION_IMAGE_TOPIC )
  ### Check if camera topic is publishing exists then initialize saving parameters
  #Wait to get the image dimensions
  img_sub = rospy.Subscriber(CL_DETECTION_IMAGE_TOPIC, Image, image_callback)
  while img_width is 0 and img_height is 0:
    print("Waiting for initial image to determine dimensions")
    time.sleep(1)
  print("Initialization Complete")
 

### Simple callback to get image height and width
def image_callback(img_msg):
  # This is just to get the image size for ratio purposes
  global img_height
  global img_width
  if (img_height == 0 and img_width == 0):
    print("Initial input image received. Size = " + str(img_msg.width) + "x" + str(img_msg.height))
    img_height = img_msg.height
    img_width = img_msg.width


# Action upon detection of object of interest
def object_detected_callback(bounding_box_msg):
  global img_height
  global img_width
  global snapshot_trigger_pub
  # Iterate over all of the objects reported by the detector
  for box in bounding_box_msg.bounding_boxes:
    # Check for the object of interest and take appropriate actions
    if box.Class == OBJ_LABEL_OF_INTEREST:
      box_of_interest=box
      print(box_of_interest.Class)
      # Calculate the box center in image ratio terms
      object_loc_y_pix = box_of_interest.ymin + ((box_of_interest.ymax - box_of_interest.ymin)  / 2) 
      object_loc_x_pix = box_of_interest.xmin + ((box_of_interest.xmax - box_of_interest.xmin)  / 2)
      object_loc_y_ratio = float(object_loc_y_pix) / img_height
      object_loc_x_ratio = float(object_loc_x_pix) / img_width
      print("Object Detected " + OBJ_LABEL_OF_INTEREST + " with box center (" + str(object_loc_x_ratio) + ", " + str(object_loc_y_ratio) + ")")
      # check if we are close enough to center in either dimension to stop motion: Hysteresis band
      box_abs_error_x_ratio = 2.0 * abs(object_loc_x_ratio - 0.5)
      box_abs_error_y_ratio = 2.0 * abs(object_loc_y_ratio - 0.5)
      print("Object Detection Error Ratios Horz: " "%.2f" % (box_abs_error_x_ratio) + " Vert: " + "%.2f" % (box_abs_error_y_ratio))
      if (box_abs_error_y_ratio <= OBJ_CENTERED_BUFFER_RATIO ) and \
         (box_abs_error_x_ratio <= OBJ_CENTERED_BUFFER_RATIO ):
        print("Detected a " + OBJ_LABEL_OF_INTEREST + " close to image center")
        print("Sending snapshot event trigger")
        snapshot_trigger_pub.publish(Empty())
        print("Delaying next trigger for " + str(RESET_DELAY_S) + " secs")
        time.sleep(RESET_DELAY_S)

### Cleanup processes on node shutdown
def cleanup_actions():
    print("Shutting down: Executing script cleanup actions")
    time.sleep(2)


### Script Entrypoint
def startNode():
  rospy.loginfo("Starting Obj_Detect and Snapshot automation script", disable_signals=True) # Disable signals so we can force a shutdown
  rospy.init_node
  rospy.init_node(name="obj_detect_and_snapshot_auto_script")
  # Run Initialization processes
  initialize_actions()
  # Set up object detector subscriber
  rospy.loginfo("Starting object detection subscriber: Object of interest = " + OBJ_LABEL_OF_INTEREST + "...")
  rospy.Subscriber(CL_BOUNDING_BOXES_TOPIC, BoundingBoxes, object_detected_callback, queue_size = 1)
  #Set up cleanup on node shutdown
  rospy.on_shutdown(cleanup_actions)
  # Spin forever (until object is detected)
  rospy.spin()


#####################################################################################
# Main
#####################################################################################

if __name__ == '__main__':
  startNode()

