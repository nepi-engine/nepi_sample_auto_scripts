#!/usr/bin/env python

# Sample NEPI Automation Script. 
# Uses onboard ROS python library to
# 1. Checks if camera image topic exists, exit if no
# 2. Start classifier
# 3. Wait for specific object to be detected and centered in image
# 4. Send one image to NEPI-CONNECT cloud portal (www.nepi.io)
# 5. Delay a next detect and send process for some set delay time

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

###!!!!!!!! Set Automation action parameters !!!!!!!!
OBJ_LABEL_OF_INTEREST = "person"
CAM_MONITOR_RES=1 # Number 0-3, 0 Low, 1 Med, 2 High, 3 Ultra
IMAGE_SEND_RES=0 # Number 0-3, 0 Low, 1 Med, 2 High, 3 Ultra
SEND_RESET_DELAY_S = 10.0 # Seconds. Delay before starting over search/save process

###!!!!!!!! Set Camera ROS Topic Name !!!!!!!!
CAMERA_NAME = "nexigo_n60_fhd_webcam_audio/"
#CAMERA_NAME = "see3cam_cu81/"
#CAMERA_NAME = "sidus_ss400/"
#CAMERA_NAME = "onwote_hd_poe/"

# ROS namespace setup
BASE_NAMESPACE = "/nepi/s2x/"
CAMERA_NAMESPACE = BASE_NAMESPACE + CAMERA_NAME
RESOLUTION_ADJ_TOPIC = CAMERA_NAMESPACE + "idx/set_resolution_mode"
IMAGE_INPUT_TOPIC = CAMERA_NAMESPACE + "idx/color_2d_image"
IMAGE_SEND_TOPIC = CAMERA_NAMESPACE + "idx/color_2d_image" # Used for NEPI-Connect

###!!!!!!!! Set Classifier topics and parameters !!!!!!!!
BOUNDING_BOXES_TOPIC = BASE_NAMESPACE + "classifier/bounding_boxes"
FOUND_OBJECT_TOPIC = BASE_NAMESPACE + "classifier/found_object"
START_CLASSIFIER_TOPIC = BASE_NAMESPACE + "start_classifier"
STOP_CLASSIFIER_TOPIC = BASE_NAMESPACE + "stop_classifier"
DETECTION_MODEL = "common_object_detection"
DETECTION_THRESHOLD = 0.5
OBJ_CENTERED_BUFFER_RATIO = 0.5 # acceptable band about center of image for saving purposes


# NEPI Link data topics and parameters
NEPI_LINK_NAMESPACE = BASE_NAMESPACE + "nepi_link_ros_bridge/"
NEPI_LINK_ENABLE_TOPIC = NEPI_LINK_NAMESPACE + "enable"
NEPI_LINK_SET_DATA_SOURCES_TOPIC = NEPI_LINK_NAMESPACE + "lb/select_data_sources"
NEPI_LINK_COLLECT_DATA_TOPIC = NEPI_LINK_NAMESPACE + "lb/create_data_set_now"
NEPI_LINK_CONNECT_TOPIC = NEPI_LINK_NAMESPACE + "connect_now"


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

#####################################################################################
# Methods
#####################################################################################

### System Initialization processes
def initialize_actions():
  global img_height
  global img_width
  print("")
  rospy.loginfo("Initializing " + CAMERA_NAMESPACE )
  rospy.loginfo("Connecting to ROS Topic" + IMAGE_INPUT_TOPIC )
  # Check if camera topic is publishing
  topic_list=rospy.get_published_topics(namespace='/')
  topic_to_connect=[IMAGE_INPUT_TOPIC, 'sensor_msgs/Image']
  if topic_to_connect in topic_list: 
    print("Camera topic found, starting initializing process")
    res_adj_pub.publish(CAM_MONITOR_RES)
    time.sleep(1)
    #Wait to get the image dimensions
    img_sub = rospy.Subscriber(IMAGE_INPUT_TOPIC, Image, image_callback)
    while img_width is 0 and img_height is 0:
      print("Waiting for initial image to determine dimensions")
      time.sleep(1)
    img_sub.unregister() # Don't need it anymore
    # Classifier initialization
    start_classifier_pub = rospy.Publisher(START_CLASSIFIER_TOPIC, ClassifierSelection, queue_size=10)
    classifier_selection = ClassifierSelection(img_topic=IMAGE_INPUT_TOPIC, classifier=DETECTION_MODEL, detection_threshold=DETECTION_THRESHOLD)
    time.sleep(1) # Important to sleep between publisher constructor and publish()
    rospy.loginfo("Starting object detector: " + str(start_classifier_pub.name))
    start_classifier_pub.publish(classifier_selection)
    print("Initialization Complete")
  else: 
    print("!!!!! Camera topic not found, shutting down")
    time.sleep(1)
    rospy.signal_shutdown("Camera topic not found")


### Simple callback to get image height and width
def image_callback(img_msg):
  # This is just to get the image size for ratio purposes
  global img_height
  global img_width
  if (img_height == 0 and img_width == 0):
    print("Initial input image received. Size = " + str(img_msg.width) + "x" + str(img_msg.height))
    img_height = img_msg.height
    img_width = img_msg.width

    
### Action upon detection of object of interest
def object_detected_callback(bounding_box_msg):
  global img_height
  global img_width
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
      print("Object Detection Error Ratios pan: " "%.2f" % (box_abs_error_x_ratio) + " tilt: " + "%.2f" % (box_abs_error_y_ratio))
      if (box_abs_error_y_ratio <= OBJ_CENTERED_BUFFER_RATIO ) and \
         (box_abs_error_x_ratio <= OBJ_CENTERED_BUFFER_RATIO ):
          print("Object is centered in frame in at least one axis: sending data") 
          print("Setting Send camera resolution")
          res_adj_pub.publish(IMAGE_SEND_RES)
          print("Setting NEPI CONNECT data sources")
          nepi_link_set_data_sources.publish([IMAGE_SEND_TOPIC])
          time.sleep
          print("Starting data collection for NEPI CONNECT")
          nepi_link_collect_data_pub.publish()
          print("Kicking off NEPI CONNECT cloud connection")
          nepi_link_connect_now_pub.publish()
          time.sleep(1)
          print("Reset Monitor Resolution")
          res_adj_pub.publish(CAM_MONITOR_RES)
          print("Delaying " + str(SEND_RESET_DELAY_S) + " secs")
          time.sleep(SEND_RESET_DELAY_S)
          print("Waiting for next detection")


### Cleanup processes on node shutdown
def cleanup_actions():
    print("Shutting down: Executing script cleanup actions")
    # Stop Classifier
    stop_classifier_pub.publish()
    time.sleep(1)



### Script Entrypoint
def startNode():
  rospy.loginfo("Starting Obj_Detect_And_Send automation script", disable_signals=True) # Disable signals so we can force a shutdown
  rospy.init_node
  rospy.init_node(name="obj_detect_and_send_to_cloud_auto_script")
  # Run Initialization processes
  initialize_actions()
  # Set up object detector subscriber
  rospy.loginfo("Starting object detection subscriber: Object of interest = " + OBJ_LABEL_OF_INTEREST + "...")
  rospy.Subscriber(BOUNDING_BOXES_TOPIC, BoundingBoxes, object_detected_callback, queue_size = 1)
  #Set up cleanup on node shutdown
  rospy.on_shutdown(cleanup_actions)
  # Spin forever (until object is detected)
  rospy.spin()


#####################################################################################
# Main
#####################################################################################

if __name__ == '__main__':
  startNode()

