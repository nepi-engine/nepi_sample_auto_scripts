#!/usr/bin/env python

# Sample NEPI Automation Script. 
# Uses onboard ROS python library to
# 1. Checks if camera image topic exists, exit if no
# 2. Start classifier
# 3. Wait for specific object to be detected
# 5. Start saving classifier output imagery to onboard storage
# 6. Delay a specified amount of time, then stop saving imagery and stop classifier
# 7. Delay a next detect and save process for some set delay time

import time
import sys
import rospy   

from sensor_msgs.msg import Image
from std_msgs.msg import UInt8, Empty, String, Bool
from nepi_ros_interfaces.msg import ClassifierSelection, SaveData, SaveDataRate, StringArray
from darknet_ros_msgs.msg import BoundingBoxes

#####################################################################################
# SETUP - Edit as Necessary ##################################
##########################################

###!!!!!!!! Set Automation action parameters !!!!!!!!
OBJ_LABEL_OF_INTEREST = "person"
SAVE_DURATION_S = 5.0 # Seconds. Length of time to save data
SAVE_DATA_RATE_HZ = 1.0
SAVE_RESET_DELAY_S = 10.0 # Seconds. Delay before starting over search/save process
SAVE_DATA_PREFIX = "obj_detect_save_automation/" # Trailing slash makes this a subdirectory name, not a filename prefix
#SAVE_DATA_PREFIX = "obj_detect_auto_" # No trailing slash makes this a filename prefix

CAM_RES=1 # Number 0-3, 0 Low, 1 Med, 2 High, 3 Ultra

###!!!!!!!! Set Camera ROS Topic Name !!!!!!!!
#CAMERA_NAME = "nexigo_n60_fhd_webcam_audio/"
CAMERA_NAME = "see3cam_cu81/"
#CAMERA_NAME = "sidus_ss400/"
#CAMERA_NAME = "onwote_hd_poe/"

# ROS namespace setup
BASE_NAMESPACE = "/nepi/s2x/"
CAMERA_NAMESPACE = BASE_NAMESPACE + CAMERA_NAME
RESOLUTION_ADJ_TOPIC = CAMERA_NAMESPACE + "idx/set_resolution_mode"
IMAGE_INPUT_TOPIC = CAMERA_NAMESPACE + "idx/color_2d_image"

###!!!!!!!! Set Classifier topics and parameters !!!!!!!!
BOUNDING_BOXES_TOPIC = BASE_NAMESPACE + "classifier/bounding_boxes"
FOUND_OBJECT_TOPIC = BASE_NAMESPACE + "classifier/found_object"
START_CLASSIFIER_TOPIC = BASE_NAMESPACE + "start_classifier"
STOP_CLASSIFIER_TOPIC = BASE_NAMESPACE + "stop_classifier"
DETECTION_MODEL = "common_object_detection"
DETECTION_THRESHOLD = 0.5
OBJ_CENTERED_BUFFER_RATIO = 0.5 # acceptable band about center of image for saving purposes

###!!!!!!!! Set Save topics and parameters !!!!!!!!
SAVE_DATA_RATE_TOPIC = CAMERA_NAMESPACE + "save_data_rate"
SAVE_DATA_PREFIX_TOPIC = CAMERA_NAMESPACE + "save_data_prefix"
SAVE_DATA_TOPIC = CAMERA_NAMESPACE + "save_data"
SAVE_DATA_PRODUCT = "color_2d_image"


#####################################################################################
# Globals
#####################################################################################
res_adj_pub = rospy.Publisher(RESOLUTION_ADJ_TOPIC, UInt8, queue_size=10)
save_data_pub = rospy.Publisher(SAVE_DATA_TOPIC, SaveData, queue_size=10)
stop_classifier_pub = rospy.Publisher(STOP_CLASSIFIER_TOPIC, Empty, queue_size=10)
save_data_rate_pub = rospy.Publisher(SAVE_DATA_RATE_TOPIC, SaveDataRate, queue_size=10)
save_data_prefix_pub = rospy.Publisher(SAVE_DATA_PREFIX_TOPIC, String, queue_size=10)

img_width = 0 # Updated on receipt of first image
img_height = 0 # Updated on receipt of first image

#####################################################################################
# Methods
#####################################################################################

### System Initialization processes
def initialize_actions():
  global img_height
  global img_width
  global res_adj_pub
  global save_data_rate_pub
  global save_data_prefix_pub
  global start_classifier_pub
  print("")
  rospy.loginfo("Initializing " + CAMERA_NAMESPACE )
  rospy.loginfo("Connecting to ROS Topic " + IMAGE_INPUT_TOPIC )
  ### Check if camera topic is publishing exists then initialize saving parameters
  topic_list=rospy.get_published_topics(namespace='/')
  topic_to_connect=[IMAGE_INPUT_TOPIC, 'sensor_msgs/Image']
  if topic_to_connect in topic_list: 
    print("Camera topic found, starting initializing process")
    res_adj_pub.publish(CAM_RES)
    time.sleep(1)
    #Wait to get the image dimensions
    img_sub = rospy.Subscriber(IMAGE_INPUT_TOPIC, Image, image_callback)
    while img_width is 0 and img_height is 0:
      print("Waiting for initial image to determine dimensions")
      time.sleep(1)
    img_sub.unregister() # Don't need it anymore
    
    ### Set up data saving rate and prefix
    # First, disable all data products
    save_data_rate_pub.publish(data_product=SaveDataRate.ALL_DATA_PRODUCTS, save_rate_hz=0.0)
    time.sleep(1) # Just for good measure
    # Then turn on just the one data product we care about
    rospy.loginfo("Setting save data rate to " + str(SAVE_DATA_RATE_HZ) + " hz")
    save_data_rate_pub.publish(data_product=SAVE_DATA_PRODUCT, save_rate_hz=SAVE_DATA_RATE_HZ)
    # Set up data saving prefix
    rospy.loginfo("Setting save data prefix to " + SAVE_DATA_PREFIX)
    save_data_prefix_pub.publish(SAVE_DATA_PREFIX)
    time.sleep(1) # Just for good measure
  
    ### Classifier initialization, and wait for it to publish
    start_classifier_pub = rospy.Publisher(START_CLASSIFIER_TOPIC, ClassifierSelection, queue_size=10)
    classifier_selection = ClassifierSelection(img_topic=IMAGE_INPUT_TOPIC, classifier=DETECTION_MODEL, detection_threshold=DETECTION_THRESHOLD)
    time.sleep(2) # Important to sleep between publisher constructor and publish()
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


# Action upon detection of object of interest
def object_detected_callback(bounding_box_msg):
  global img_height
  global img_width
  global res_adj_pub
  global save_data_pub
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
        print("Detected a " + OBJ_LABEL_OF_INTEREST + "close to image center")
        print("Enabling data saving")
        save_data_pub.publish(save_continuous=True, save_raw=False)
        print("Delaying " + str(SAVE_DURATION_S) + " secs")
        time.sleep(SAVE_DURATION_S)
        print("Disabling data saving")
        save_data_pub.publish(save_continuous=False, save_raw=False)
        print("Delaying " + str(SAVE_RESET_DELAY_S) + " secs")
        time.sleep(SAVE_RESET_DELAY_S)

### Cleanup processes on node shutdown
def cleanup_actions():
    global stop_classifier_pub
    global save_data_pub
    global save_data_rate_pub
    global save_data_prefix_pub
    print("Shutting down: Executing script cleanup actions")
    # Stop Classifier
    stop_classifier_pub.publish()
    # Restor some startup defaults
    # Disabling data saving 
    save_data_pub.publish(save_continuous=False, save_raw=False)
    # Restoring data save setup defaults
    save_data_rate_pub.publish(data_product=SaveDataRate.ALL_DATA_PRODUCTS, save_rate_hz=1.0)
    # And remove the prefix
    save_data_prefix_pub.publish("")
    time.sleep(2)


### Script Entrypoint
def startNode():
  rospy.loginfo("Starting Obj_Detect_And_Save automation script", disable_signals=True) # Disable signals so we can force a shutdown
  rospy.init_node
  rospy.init_node(name="obj_detect_and_save_auto_script")
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

