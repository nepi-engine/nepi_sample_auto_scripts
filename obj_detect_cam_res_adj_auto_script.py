#!/usr/bin/env python

# Sample NEPI Automation Script. 
# Uses onboard ROS python library to
# 1. Start camera in LOW resolution mode
# 2. Start classifier
# 3. Wait for specific object to be detected
# 4. Increase camera resolution to ULTRA
# 5. Start saving classifier output imagery to onboard storage
# 6. Delay a specified amount of time, then stop saving imagery and stop classifier
# 7. Exit

import time
import sys
import rospy

from std_msgs.msg import UInt8, Empty, String, Bool
from nepi_ros_interfaces.msg import ClassifierSelection, SaveData, SaveDataRate, StringArray
from darknet_ros_msgs.msg import BoundingBoxes

######################## SETUP - Edit as Necessary ##################################
# ROS namespace setup
BASE_NAMESPACE = "/nepi/s2x/"
#
#
# Edit for your camera ros topic name
CAMERA_NAME = "nexigo_n60_fhd_webcam_audio/"
#CAMERA_NAME = "sidus_ss400/"
#CAMERA_NAME = "onwote_hd_poe/"
#
#
#

print(CAMERA_NAME)
CAMERA_NAMESPACE = BASE_NAMESPACE + CAMERA_NAME

# Camera topics and parameters
RESOLUTION_ADJ_TOPIC = CAMERA_NAMESPACE + "idx/set_resolution_mode"
COLOR_2D_IMG_TOPIC_SHORT = CAMERA_NAME + "idx/color_2d_image"
COLOR_2D_IMG_TOPIC = CAMERA_NAMESPACE + "idx/color_2d_image"

LOW_RES_VALUE = 0
ULTRA_RES_VALUE = 3

# Classifier topics and parameters
BOUNDING_BOXES_TOPIC = BASE_NAMESPACE + "classifier/bounding_boxes"
START_CLASSIFIER_TOPIC = BASE_NAMESPACE + "start_classifier"
STOP_CLASSIFIER_TOPIC = BASE_NAMESPACE + "stop_classifier"
DETECTION_MODEL = "common_object_detection"
DETECTION_THRESHOLD = 0.5

# Save data topics and parameters
SAVE_DATA_RATE_TOPIC = CAMERA_NAMESPACE + "save_data_rate"
SAVE_DATA_PREFIX_TOPIC = CAMERA_NAMESPACE + "save_data_prefix"
SAVE_DATA_TOPIC = CAMERA_NAMESPACE + "save_data"

# NEPI Link data topics and parameters
NEPI_LINK_NAMESPACE = BASE_NAMESPACE + "nepi_link_ros_bridge/"
NEPI_LINK_ENABLE_TOPIC = NEPI_LINK_NAMESPACE + "enable"
NEPI_LINK_SET_DATA_SOURCES_TOPIC = NEPI_LINK_NAMESPACE + "lb/select_data_sources"
NEPI_LINK_COLLECT_DATA_TOPIC = NEPI_LINK_NAMESPACE + "lb/create_data_set_now"
NEPI_LINK_CONNECT_TOPIC = NEPI_LINK_NAMESPACE + "connect_now"

# Parameters for actions upon detection of object of interest
OBJ_LABEL_OF_INTEREST = "person"
RES_ADJ_DURATION = 20.0 # Seconds. Length of time in ultra resolution mode while saving data
#SAVE_DATA_PREFIX = "obj_detect_cam_adjust_automation/" # Trailing slash makes this a subdirectory name, not a filename prefix
SAVE_DATA_PREFIX = "obj_detect_auto_"
SAVE_DATA_RATE_HZ = 1.0

#####################################################################################

# Globals
res_adj_pub = rospy.Publisher(RESOLUTION_ADJ_TOPIC, UInt8, queue_size=10)
save_data_pub = rospy.Publisher(SAVE_DATA_TOPIC, SaveData, queue_size=10)
stop_classifier_pub = rospy.Publisher(STOP_CLASSIFIER_TOPIC, Empty, queue_size=10)
save_data_rate_pub = rospy.Publisher(SAVE_DATA_RATE_TOPIC, SaveDataRate, queue_size=10)
save_data_prefix_pub = rospy.Publisher(SAVE_DATA_PREFIX_TOPIC, String, queue_size=10)
nepi_link_enable_pub = rospy.Publisher(NEPI_LINK_ENABLE_TOPIC, Bool, queue_size=10)
nepi_link_set_data_sources = rospy.Publisher(NEPI_LINK_SET_DATA_SOURCES_TOPIC, StringArray, queue_size=10)
nepi_link_collect_data_pub = rospy.Publisher(NEPI_LINK_COLLECT_DATA_TOPIC, Empty, queue_size=10)
nepi_link_connect_now_pub = rospy.Publisher(NEPI_LINK_CONNECT_TOPIC, Empty, queue_size=10)

def cleanup_actions():
    rospy.loginfo("Shutting down: Executing script cleanup actions")
    # Fine to call these anytime -- they just restore some startup defaults
    rospy.loginfo("Disabling data saving")
    save_data_pub.publish(save_continuous=False, save_raw=False)

    rospy.loginfo("Restoring to LOW resolution")
    res_adj_pub.publish(LOW_RES_VALUE)

    # Restore data saving
    rospy.loginfo("Restoring data save setup defaults")
    # First, reenable all data products
    save_data_rate_pub.publish(data_product=SaveDataRate.ALL_DATA_PRODUCTS, save_rate_hz=1.0)
    # And remove the prefix
    save_data_prefix_pub.publish("")

# Action upon detection of object of interest
def object_detected_callback(bounding_box_msg):
  # Iterate over all of the objects reported by the detector
  for box in bounding_box_msg.bounding_boxes:
    # Check for the object of interest and take appropriate actions
    if box.Class == OBJ_LABEL_OF_INTEREST:
      rospy.loginfo("Detected a " + OBJ_LABEL_OF_INTEREST)

      rospy.loginfo("Stopping object detector")
      stop_classifier_pub.publish()
      
      rospy.loginfo("Increasing camera resolution to ULTRA")
      res_adj_pub.publish(ULTRA_RES_VALUE)
      
      rospy.loginfo("Enabling data saving")
      save_data_pub.publish(save_continuous=True, save_raw=False)

      rospy.loginfo("Enabling NEPI CONNECT")
      nepi_link_enable_pub.publish(True)

      rospy.loginfo("Setting NEPI CONNECT data sources")
      nepi_link_set_data_sources.publish([COLOR_2D_IMG_TOPIC_SHORT])
      time.sleep(1)

      rospy.loginfo("Starting data collection for NEPI CONNECT")
      nepi_link_collect_data_pub.publish()

      rospy.loginfo("Delaying " + str(RES_ADJ_DURATION) + " secs")
      time.sleep(RES_ADJ_DURATION)

      rospy.loginfo("Kicking off NEPI CONNECT cloud connection")
      nepi_link_connect_now_pub.publish()
      time.sleep(1)

      rospy.loginfo("Disabling NEPI CONNECT locally... current connection will complete")
      nepi_link_enable_pub.publish(False)
      time.sleep(2)
      
      rospy.loginfo("Script completed successfully -- terminating")
      rospy.signal_shutdown("Script complete")
    else:
      rospy.loginfo_throttle(1.0, "No " + OBJ_LABEL_OF_INTEREST + " detected")

# Script Entrypoint
def startNode():
  rospy.loginfo("Starting Obj_Detect_Cam_Adjust automation script", disable_signals=True) # Disable signals so we can force a shutdown
  rospy.init_node
  rospy.init_node(name="obj_detect_cam_adjust_auto_script")

  # Camera initialization
  rospy.loginfo("Initializing " + CAMERA_NAMESPACE + " to LOW resolution")
  time.sleep(1)
  res_adj_pub.publish(LOW_RES_VALUE)

  # Classifier initialization
  start_classifier_pub = rospy.Publisher(START_CLASSIFIER_TOPIC, ClassifierSelection, queue_size=10)
  classifier_selection = ClassifierSelection(img_topic=COLOR_2D_IMG_TOPIC, classifier=DETECTION_MODEL, detection_threshold=DETECTION_THRESHOLD)
  time.sleep(1) # Important to sleep between publisher constructor and publish()
  rospy.loginfo("Starting object detector: " + str(start_classifier_pub.name))
  start_classifier_pub.publish(classifier_selection)

  # Set up data saving rate
  # First, disable all data products
  save_data_rate_pub.publish(data_product=SaveDataRate.ALL_DATA_PRODUCTS, save_rate_hz=0.0)
  # Then turn on just the one data product we care about
  time.sleep(1) # Just for good measure
  rospy.loginfo("Setting color_2d_image save data rate to " + str(SAVE_DATA_RATE_HZ) + "hz")
  save_data_rate_pub.publish(data_product='color_2d_image', save_rate_hz=SAVE_DATA_RATE_HZ)
  # Set up data saving prefix
  time.sleep(1)
  rospy.loginfo("Setting save data prefix to " + SAVE_DATA_PREFIX)
  save_data_prefix_pub.publish(SAVE_DATA_PREFIX)

  # Set up object detector subscriber
  rospy.loginfo("Starting object detection subscriber: Object of interest = " + OBJ_LABEL_OF_INTEREST + "...")
  rospy.Subscriber(BOUNDING_BOXES_TOPIC, BoundingBoxes, object_detected_callback)

  rospy.on_shutdown(cleanup_actions)
  
  # Spin forever (until object is detected)
  rospy.spin()

if __name__ == '__main__':
  startNode()

