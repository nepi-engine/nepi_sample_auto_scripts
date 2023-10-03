#!/usr/bin/env python

# Sample NEPI Automation Script. 
# Uses onboard ROS python library to
# 1. Start camera in LOW resolution mode
# 2. Start classifier
# 3. Wait for specific object to be detected and centered in image
# 4. Send one image to NEPI-CONNECT cloud portal (www.nepi.io)
# 5. Cleanup and Exit

import time
import sys
import rospy

from sensor_msgs.msg import Image
from std_msgs.msg import UInt8, Empty, String, Bool, Float32
from nepi_ros_interfaces.msg import ClassifierSelection, StringArray
from darknet_ros_msgs.msg import BoundingBoxes

#####################################################################################
# SETUP - Edit as Necessary ##################################
##########################################

# ROS namespace setup
BASE_NAMESPACE = "/nepi/s2x/"

###!!!!!!!! Set Camera topics and parameters !!!!!!!!
CAMERA_NAME = "nexigo_n60_fhd_webcam_audio/"
#CAMERA_NAME = "sidus_ss400/"
#CAMERA_NAME = "onwote_hd_poe/"
#CAMERA_NAME = "see3cam_cu81/"
CAMERA_NAMESPACE = BASE_NAMESPACE + CAMERA_NAME

RESOLUTION_ADJ_TOPIC = CAMERA_NAMESPACE + "idx/set_resolution_mode"
COLOR_2D_IMG_TOPIC = CAMERA_NAMESPACE + "idx/color_2d_image"
COLOR_2D_IMG_TOPIC_SHORT = CAMERA_NAME + "idx/color_2d_image" # Used for NEPI-Connect

LOW_RES_VALUE = 0
MED_RES_VALUE = 1
HIGH_RES_VALUE = 2
ULTRA_RES_VALUE = 3

###!!!!!!!! Set Classifier topics and parameters !!!!!!!!
BOUNDING_BOXES_TOPIC = BASE_NAMESPACE + "classifier/bounding_boxes"
FOUND_OBJECT_TOPIC = BASE_NAMESPACE + "classifier/found_object"
DETECT_IMAGE_TOPIC = BASE_NAMESPACE + "classifier/detection_image"
DETECT_IMAGE_TOPIC_SHORT = "classifier/detection_image" # Used for NEPI-Connect

START_CLASSIFIER_TOPIC = BASE_NAMESPACE + "start_classifier"
STOP_CLASSIFIER_TOPIC = BASE_NAMESPACE + "stop_classifier"
DETECTION_MODEL = "common_object_detection_fast"
DETECTION_THRESHOLD = 0.5
OBJ_CENTERED_BUFFER_RATIO = 0.3 # error band about center of image for capture and send
MIN_BOX_AREA = 50 # Minimum detection box area (px^2) to track

# NEPI Link data topics and parameters
NEPI_LINK_NAMESPACE = BASE_NAMESPACE + "nepi_link_ros_bridge/"
NEPI_LINK_ENABLE_TOPIC = NEPI_LINK_NAMESPACE + "enable"
NEPI_LINK_SET_DATA_SOURCES_TOPIC = NEPI_LINK_NAMESPACE + "lb/select_data_sources"
NEPI_LINK_COLLECT_DATA_TOPIC = NEPI_LINK_NAMESPACE + "lb/create_data_set_now"
NEPI_LINK_CONNECT_TOPIC = NEPI_LINK_NAMESPACE + "connect_now"


###!!!!!!!! Set Automation action parameteCOLOR_2D_IMG_TOPIC_SHORTrs !!!!!!!!
OBJ_LABEL_OF_INTEREST = "person"



#####################################################################################
# Globals
#####################################################################################

res_adj_pub = rospy.Publisher(RESOLUTION_ADJ_TOPIC, UInt8, queue_size=10)
stop_classifier_pub = rospy.Publisher(STOP_CLASSIFIER_TOPIC, Empty, queue_size=10)
nepi_link_enable_pub = rospy.Publisher(NEPI_LINK_ENABLE_TOPIC, Bool, queue_size=10)
nepi_link_set_data_sources = rospy.Publisher(NEPI_LINK_SET_DATA_SOURCES_TOPIC, StringArray, queue_size=10)
nepi_link_collect_data_pub = rospy.Publisher(NEPI_LINK_COLLECT_DATA_TOPIC, Empty, queue_size=10)
nepi_link_connect_now_pub = rospy.Publisher(NEPI_LINK_CONNECT_TOPIC, Empty, queue_size=10)

img_width = 0 # Updated on receipt of first image
img_height = 0 # Updated on receipt of first image
send_resolution = LOW_RES_VALUE
reset_resolution = MED_RES_VALUE


#####################################################################################
# Methods
#####################################################################################


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
  box_of_interest = None
  # Iterate over all of the objects reported by the detector and return center of largest box in degrees relative to img center
  largest_box_area=0 # Initialize largest box area
  for box in bounding_box_msg.bounding_boxes:
    # Check for the object of interest and take appropriate actions
    if box.Class == OBJ_LABEL_OF_INTEREST:
      # Check if largest box
      box_area=(box.xmax-box.xmin)*(box.ymax-box.ymin)
      if box_area > largest_box_area:
        largest_box_area=box_area
        box_of_interest = box
      if largest_box_area > MIN_BOX_AREA:

        # Calculate the box center in image ratio terms
        object_loc_y_pix = box_of_interest.ymin + ((box_of_interest.ymax - box_of_interest.ymin)  / 2) 
        object_loc_x_pix = box_of_interest.xmin + ((box_of_interest.xmax - box_of_interest.xmin)  / 2)
        object_loc_y_ratio = float(object_loc_y_pix) / img_height
        object_loc_x_ratio = float(object_loc_x_pix) / img_width
        print("Object Detected " + OBJ_LABEL_OF_INTEREST + " with box center (" + str(object_loc_x_ratio) + ", " + str(object_loc_y_ratio) + ")")
        # check if we are close enough to center in either dimension to stop motion: Hysteresis band
        box_abs_error_x_ratio = 2.0 * abs(object_loc_x_ratio - 0.5)
        box_abs_error_y_ratio = 2.0 * abs(object_loc_y_ratio - 0.5)
        #print("Object Detection Error Ratios pan: " "%.2f" % (box_abs_error_x_ratio) + " tilt: " + "%.2f" % (box_abs_error_y_ratio))
        if (box_abs_error_y_ratio <= OBJ_CENTERED_BUFFER_RATIO ) or \
           (box_abs_error_x_ratio <= OBJ_CENTERED_BUFFER_RATIO ):
          print(1.0, "Object is centered in frame in at least one axis: sending data") 
    
          print("Stopping object detector")
          stop_classifier_pub.publish()

          print("Setting Send camera resolution")
          res_adj_pub.publish(send_resolution)
      
          print("Setting NEPI CONNECT data sources")
          nepi_link_set_data_sources.publish([COLOR_2D_IMG_TOPIC_SHORT])
          time.sleep

          print("Starting data collection for NEPI CONNECT")
          nepi_link_collect_data_pub.publish()

          print("Kicking off NEPI CONNECT cloud connection")
          nepi_link_connect_now_pub.publish()
          time.sleep(1)

          #rospy.loginfo("Disabling NEPI CONNECT locally... current connection will complete")
          #nepi_link_enable_pub.publish(False)
          #time.sleep(2)

   
          print("Script completed successfully -- terminating")
          rospy.signal_shutdown("Script complete")


### Cleanup processes on node shutdown
def cleanup_actions():
    print("Shutting down: Executing script cleanup actions")

### Script Entrypoint
def startNode():
  rospy.loginfo("Starting Obj_Detect_And_Cloud_Send automation script", disable_signals=True) # Disable signals so we can force a shutdown
  rospy.init_node
  rospy.init_node(name="obj_detect_and_send_cloud_auto_script")

  # Camera initialization
  res_adj_pub.publish(LOW_RES_VALUE)
  #Wait to get the image dimensions
  img_sub = rospy.Subscriber(COLOR_2D_IMG_TOPIC, Image, image_callback)
  while img_width is 0 and img_height is 0:
    print("Waiting for initial image to determine dimensions")
    time.sleep(1)
  img_sub.unregister() # Don't need it anymore
  
  # Classifier initialization
  start_classifier_pub = rospy.Publisher(START_CLASSIFIER_TOPIC, ClassifierSelection, queue_size=10)
  classifier_selection = ClassifierSelection(img_topic=COLOR_2D_IMG_TOPIC, classifier=DETECTION_MODEL, detection_threshold=DETECTION_THRESHOLD)
  time.sleep(1) # Important to sleep between publisher constructor and publish()
  rospy.loginfo("Starting object detector: " + str(start_classifier_pub.name))
  start_classifier_pub.publish(classifier_selection)
  # Set up object detector subscriber
  rospy.loginfo("Starting object detection subscriber: Object of interest = " + OBJ_LABEL_OF_INTEREST + "...")
  rospy.Subscriber(BOUNDING_BOXES_TOPIC, BoundingBoxes, object_detected_callback)

  rospy.on_shutdown(cleanup_actions)
  
  # Spin forever (until object is detected)
  rospy.spin()


#####################################################################################
# Main
#####################################################################################

if __name__ == '__main__':
  startNode()

