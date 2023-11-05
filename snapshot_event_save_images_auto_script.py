#!/usr/bin/env python

# Sample NEPI Automation Script. 
# Uses onboard ROS python library to
# 1. Confirms IDX driver supported camera topic is publishing
# 2. Waits for snapshot_event topic message 
# 3. Start saving camera and navpose data to onboard storage
# 4. Delay a specified amount of time, then stop saving
# 5. Delay a next detect and save process for some set delay time

import time
import sys
import rospy   

from std_msgs.msg import UInt8, Empty, String, Bool
from nepi_ros_interfaces.msg import SaveData, SaveDataRate, StringArray


#####################################################################################
# SETUP - Edit as Necessary ##################################
##########################################
BASE_NAMESPACE = "/nepi/s2x/"

###!!!!!!!! Set Automation action parameters !!!!!!!!
SAVE_DURATION_S = 5.0 # Seconds. Length of time to save data
SAVE_DATA_RATE_HZ = 1.0
SAVE_RESET_DELAY_S = 5.0 # Seconds. Delay before starting over search/save process
SAVE_DATA_PREFIX = "snapshot_event/" # Trailing slash makes this a subdirectory name, not a filename prefix
#SAVE_IMAGE_PREFIX = "snapshot_event_" # No trailing slash makes this a filename prefix

###!!!!!!!! Set NEPI IDXZ Supported Camera ROS Topic Name !!!!!!!!
CAMERA_NAME = "nexigo_n60_fhd_webcam_audio/"
#CAMERA_NAME = "see3cam_cu81/"
#CAMERA_NAME = "sidus_ss400/"
#CAMERA_NAME = "onwote_hd_poe/"

###!!!!!!!! Set Save topics and parameters !!!!!!!!
CAMERA_SAVE_IMAGE_NAME = "color_2d_image"
CAMERA_IMAGE_TOPIC = BASE_NAMESPACE + CAMERA_NAME + 'idx/' + CAMERA_SAVE_IMAGE_NAME
CAMERA_SAVE_TOPIC = BASE_NAMESPACE + CAMERA_NAME + "save_data"
CAMERA_SAVE_RATE_TOPIC = BASE_NAMESPACE + CAMERA_NAME + "save_data_rate"
CAMERA_SAVE_PREFIX_TOPIC = BASE_NAMESPACE + CAMERA_NAME + "save_data_prefix"

# NOTE:NavPose data will not publish until values are updating from valid input source
NAVPOSE_NAME = "nav_pose_mgr/"
NAVPOSE_SAVE_TOPIC = BASE_NAMESPACE + NAVPOSE_NAME + "save_data"
NAVPOSE_SAVE_RATE_TOPIC = BASE_NAMESPACE + NAVPOSE_NAME + "save_data_rate"
NAVPOSE_SAVE_PREFIX_TOPIC = BASE_NAMESPACE + NAVPOSE_NAME + "save_data_prefix"

### Snapshot Topic Name
SNAPSHOT_TOPIC = BASE_NAMESPACE + "snapshot_event"

#####################################################################################
# Globals
#####################################################################################
save_image_pub = rospy.Publisher(CAMERA_SAVE_TOPIC, SaveData, queue_size=10)
save_image_rate_pub = rospy.Publisher(CAMERA_SAVE_RATE_TOPIC, SaveDataRate, queue_size=10)
save_image_prefix_pub = rospy.Publisher(CAMERA_SAVE_PREFIX_TOPIC, String, queue_size=10)

save_navpose_pub = rospy.Publisher(NAVPOSE_SAVE_TOPIC, SaveData, queue_size=10)
save_navpose_rate_pub = rospy.Publisher(NAVPOSE_SAVE_RATE_TOPIC, SaveDataRate, queue_size=10)
save_navpose_prefix_pub = rospy.Publisher(NAVPOSE_SAVE_PREFIX_TOPIC, String, queue_size=10)

#####################################################################################
# Methods
#####################################################################################

### System Initialization processes
def initialize_actions():
  global save_image_rate_pub
  global save_image_prefix_pub
  global save_navpose_rate_pub
  global save_navpose_prefix_pub
  print("")
  rospy.loginfo("Initializing " + CAMERA_NAME )
  rospy.loginfo("Connecting to ROS Topic " + CAMERA_IMAGE_TOPIC )
  ### Check if camera topic is publishing exists then initialize saving parameters
  topic_list=rospy.get_published_topics(namespace='/')
  topic_to_connect=[CAMERA_IMAGE_TOPIC, 'sensor_msgs/Image']
  if topic_to_connect in topic_list: 
    print("Camera topic found, starting initializing process")
    ### Set up data saving rate and prefix
    # First, disable all data products
    save_image_rate_pub.publish(data_product=SaveDataRate.ALL_DATA_PRODUCTS, save_rate_hz=0.0)
    save_navpose_rate_pub.publish(data_product=SaveDataRate.ALL_DATA_PRODUCTS, save_rate_hz=0.0)
    time.sleep(1) # Just for good measure
    # Then turn on just the one data product we care about
    rospy.loginfo("Setting save data rate to " + str(SAVE_DATA_RATE_HZ) + " hz")
    save_image_rate_pub.publish(data_product=CAMERA_SAVE_IMAGE_NAME, save_rate_hz=SAVE_DATA_RATE_HZ)
    save_navpose_rate_pub.publish(data_product=SaveDataRate.ALL_DATA_PRODUCTS, save_rate_hz=SAVE_DATA_RATE_HZ)
    time.sleep(1) # Just for good measure
    # Set up data saving prefix
    rospy.loginfo("Setting save data prefix to " + SAVE_DATA_PREFIX)
    save_image_prefix_pub.publish(SAVE_DATA_PREFIX)
    save_navpose_prefix_pub.publish(SAVE_DATA_PREFIX)
    time.sleep(1) # Just for good measure
    print("Initialization Complete")
    print("Waiting for " + str(SNAPSHOT_TOPIC) + ' event trigger')
  else: 
    print("!!!!! Camera topic not found, shutting down")
    time.sleep(1)
    rospy.signal_shutdown("Camera topic not found")

# Action upon detection of object of interest
def snapshot_event_callback(event):
  global save_image_pub
  global save_navpose_pub
  print("Enabling data saving")
  print(CAMERA_SAVE_TOPIC)
  save_image_pub.publish(save_continuous=True, save_raw=False)
  save_navpose_pub.publish(save_continuous=True, save_raw=False)
  print("Saving for " + str(SAVE_DURATION_S) + " secs")
  time.sleep(SAVE_DURATION_S)
  print("Disabling data saving")
  save_image_pub.publish(save_continuous=False, save_raw=False)
  save_navpose_pub.publish(save_continuous=False, save_raw=False)
  print("Delaying next trigger for " + str(SAVE_RESET_DELAY_S) + " secs")
  time.sleep(SAVE_RESET_DELAY_S)
  print("Waiting for next " + str(SNAPSHOT_TOPIC) + ' event trigger')

### Cleanup processes on node shutdown
def cleanup_actions():
  global save_image_pub
  global save_image_rate_pub
  global save_image_prefix_pub
  global save_navpose_pub
  global save_navpose_rate_pub
  global save_navpose_prefix_pub
  print("Shutting down: Executing script cleanup actions")
  # Restor some startup defaults
  # Disabling data saving 
  save_image_pub.publish(save_continuous=False, save_raw=False)
  save_navpose_pub.publish(save_continuous=False, save_raw=False)
  # Restoring data save setup defaults
  save_image_rate_pub.publish(data_product=SaveDataRate.ALL_DATA_PRODUCTS, save_rate_hz=1.0)
  save_navpose_rate_pub.publish(data_product=SaveDataRate.ALL_DATA_PRODUCTS, save_rate_hz=1.0)
  # And remove the prefix
  save_image_prefix_pub.publish("")
  time.sleep(2)


### Script Entrypoint
def startNode():
  rospy.loginfo("Starting Snapshot Event Detect and Save automation script", disable_signals=True) # Disable signals so we can force a shutdown
  rospy.init_node(name="snapshot_event_detect_save_auto_script")
  # Run Initialization processes
  initialize_actions()
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

