#!/usr/bin/env python

# Sample NEPI Automation Script. 
# Uses onboard ROS python library to
# 1. Start camera in MEDIUM resolution mode
# 2. Start classifier
# 3. Configure and start pan and tilt
# 4. Set and start pan and tilt search scan
# 5. Wait for specific object to be detected and start tracking largest detection box
# 4. Return to search scan if no detected objects are being detected

import time
import sys
import rospy
import numpy as np
import cv2
import subprocess
import os

from std_msgs.msg import UInt8, Empty, Float32
from sensor_msgs.msg import Image
from nepi_ros_interfaces.msg import ClassifierSelection, PanTiltLimits, PanTiltPosition, PanTiltStatus, StringArray
from darknet_ros_msgs.msg import BoundingBoxes, ObjectCount


############################################################################
# SETUP - User Settings
############################################################################

# Minimum detection box area as a ration of image size
MIN_DETECT_BOX_AREA_RATIO = 0.15 # Filters background targets.
OBJ_LABEL_OF_INTEREST = "person"


# Set Camera topics
CAMERA_NAME = "nexigo_n60_fhd_webcam_audio/"
#CAMERA_NAME = "sidus_ss400/"
#CAMERA_NAME = "onwote_hd_poe/"
#CAMERA_NAME = "see3cam_cu81/"
CAM_RES=1 # Number 0-3, 0 Low, 1 Med, 2 High, 3 Ultra

# Pan and Tilt topics and parameters
PT_NAME = "iqr_pan_tilt/"
PT_REVERSE_PAN = False # Relative to image pixel location reporting
PT_REVERSE_TILT = True # Relative to image pixel location reporting
PT_SCAN_PAN_LIMITS = 40 # +- Pan Angle Limits
PT_SCAN_TILT_RATIO = 0.55 # Tilt Angle During Scanning

############################################################################
# SETUP - Env Settings
############################################################################
# ROS namespace setup
BASE_NAMESPACE = "/nepi/s2x/"
CAMERA_NAMESPACE = BASE_NAMESPACE + CAMERA_NAME
RESOLUTION_ADJ_TOPIC = CAMERA_NAMESPACE + "idx/set_resolution_mode"
IMAGE_INPUT_TOPIC = CAMERA_NAMESPACE + "idx/color_2d_image"

# Classifier topics and parameters
BOUNDING_BOXES_TOPIC = BASE_NAMESPACE + "classifier/bounding_boxes"
FOUND_OBJECT_TOPIC = BASE_NAMESPACE + "classifier/found_object"
START_CLASSIFIER_TOPIC = BASE_NAMESPACE + "start_classifier"
STOP_CLASSIFIER_TOPIC = BASE_NAMESPACE + "stop_classifier"
DETECTION_MODEL = "common_object_detection"
DETECTION_THRESHOLD = 0.5
MIN_BOX_AREA = 50000 # Minimum detection box area (px^2) to track
OBJ_CENTERED_BUFFER_RATIO = 0.3 # Hysteresis band about center of image for tracking purposes



PT_NAMESPACE = BASE_NAMESPACE + PT_NAME
PT_GET_STATUS_TOPIC = PT_NAMESPACE + "ptx/status"
PT_SET_SPEED_RATIO_TOPIC = PT_NAMESPACE + "ptx/set_speed_ratio"
PT_GOHOME_TOPIC = PT_NAMESPACE + "ptx/go_home"
PT_STOP_TOPIC = PT_NAMESPACE + "ptx/stop_moving"
PT_GOTO_PAN_RATIO_TOPIC = PT_NAMESPACE + "ptx/jog_to_yaw_ratio"
PT_GOTO_TILT_RATIO_TOPIC = PT_NAMESPACE + "ptx/jog_to_pitch_ratio"
PT_SET_SOFT_LIMITS_TOPIC = PT_NAMESPACE + "ptx/set_soft_limits"

# Parameters for pan and tilt settings
PT_SCAN_CHECK_INTERVAL = 0.25
PT_SCAN_SPEED = 0.6
PT_MAX_TRACK_SPEED = 1.0
PT_MIN_TRACK_SPEED = 0.05
PT_OBJECT_TILT_OFFSET_RATIO = 0.7 # Adjust tilt tracking ratio location within box





#####################################################################################
# Globals
#####################################################################################

res_adj_pub = rospy.Publisher(RESOLUTION_ADJ_TOPIC, UInt8, queue_size=10)
stop_classifier_pub = rospy.Publisher(STOP_CLASSIFIER_TOPIC, Empty, queue_size=10)
send_pt_home_pub = rospy.Publisher(PT_GOHOME_TOPIC, Empty, queue_size=10)
set_pt_speed_ratio_pub = rospy.Publisher(PT_SET_SPEED_RATIO_TOPIC, Float32, queue_size=10)
set_pt_pan_ratio_pub = rospy.Publisher(PT_GOTO_PAN_RATIO_TOPIC, Float32, queue_size=10)
set_pt_tilt_ratio_pub = rospy.Publisher(PT_GOTO_TILT_RATIO_TOPIC, Float32, queue_size=10)
set_pt_soft_limits_pub = rospy.Publisher(PT_SET_SOFT_LIMITS_TOPIC, PanTiltLimits, queue_size=10)

img_width = 0 # Updated on receipt of first image
img_height = 0 # Updated on receipt of first image
img_area = 0 # Updated on receipt of first image

pt_stop_motion_pub = rospy.Publisher(PT_STOP_TOPIC, Empty, queue_size=10)
pan_scan_direction = 1 # Keep track of current scan direction (1: Positive Limit, -1: Negative Limit)
img_width = 0 # Updated on receipt of first image
img_height = 0 # Updated on receipt of first image
pt_forward_pan_limit_ratio = 1.0 if PT_REVERSE_PAN is False else 0.0
pt_backward_pan_limit_ratio = 1.0 - pt_forward_pan_limit_ratio
pt_forward_tilt_limit_ratio = 1.0 if PT_REVERSE_TILT is False else 0.0
pt_backward_tilt_limit_ratio = 1.0 - pt_forward_tilt_limit_ratio
pt_tilt_scan_ratio = PT_SCAN_TILT_RATIO if PT_REVERSE_TILT is False else 1-PT_SCAN_TILT_RATIO
object_detected=False
last_object_pan_ratio=0
pt_yaw_now_deg=0
pt_pitch_now_deg=0
pt_yaw_now_ratio=0
pt_pitch_now_ratio=0
pt_speed_now_ratio=0



#####################################################################################
# Methods
#####################################################################################


### System Initialization processes
def initialize_actions():
  global res_adj_pub
  global set_pt_speed_ratio_pub
  global set_pt_pan_ratio_pub
  global set_pt_tilt_ratio_pub
  global set_pt_soft_limits_pub
  global pt_tilt_scan_ratio
  print("")
  rospy.loginfo("Initializing " + CAMERA_NAMESPACE )
  rospy.loginfo("Connecting to ROS Topic " + IMAGE_INPUT_TOPIC )
  # Check if camera topic is publishing
  topic_list=rospy.get_published_topics(namespace='/')
  topic_to_connect=[IMAGE_INPUT_TOPIC, 'sensor_msgs/Image']
  if topic_to_connect in topic_list: 
    # Camera initialization
    print("Initializing Camera resolution")
    res_adj_pub.publish(CAM_RES)
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
    print("Starting object detector: " + str(start_classifier_pub.name))
    start_classifier_pub.publish(classifier_selection)
    # Start PT Status Callback
    print("Starting Pan Tilt Stutus callback")
    rospy.Subscriber(PT_GET_STATUS_TOPIC, PanTiltStatus, pt_status_callback)  
    # Pan/Tilt initialization: Center both axes
    print("Setting pan/tilt to initial position and speed")
    print("Scan tilt ratio set to: " + "%.2f" % (pt_tilt_scan_ratio))
    set_pt_tilt_ratio_pub.publish(pt_tilt_scan_ratio)
    set_pt_pan_ratio_pub.publish(0.5)
    set_pt_speed_ratio_pub.publish(PT_SCAN_SPEED)
    time.sleep(2) # Give time to center  
    # Set up the timer that start scanning when no objects are detected
    print("Setting up pan/tilt scan check timer")
    rospy.Timer(rospy.Duration(PT_SCAN_CHECK_INTERVAL), pt_scan_timer_callback)
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
  global img_area
  if (img_height == 0 and img_width == 0):
    print("Initial input image received. Size = " + str(img_msg.width) + "x" + str(img_msg.height))
    img_height = img_msg.height
    img_width = img_msg.width
    img_area = img_height*img_width



### Simple callback to get pt status info
def pt_status_callback(PanTiltStatus):
  # This is just to get the current pt positions
  global pt_yaw_now_deg
  global pt_pitch_now_deg
  global pt_yaw_now_ratio
  global pt_pitch_now_ratio
  global pt_speed_now_ratio
  pt_yaw_now_deg=PanTiltStatus.yaw_now_deg
  pt_pitch_now_deg=PanTiltStatus.pitch_now_deg
  yaw_limit_min=PanTiltStatus.yaw_min_softstop_deg
  yaw_limit_max=PanTiltStatus.yaw_max_softstop_deg
  pitch_limit_min=PanTiltStatus.pitch_min_softstop_deg
  pitch_limit_max =PanTiltStatus.pitch_max_softstop_deg
  pt_yaw_now_ratio=(pt_yaw_now_deg-yaw_limit_min)/(yaw_limit_max-yaw_limit_min)
  if PT_REVERSE_PAN:
    pt_yaw_now_ratio=1-pt_yaw_now_ratio
  pt_pitch_now_ratio=(pt_pitch_now_deg-pitch_limit_min)/(pitch_limit_max-pitch_limit_min)
  if PT_REVERSE_TILT:
    pt_pitch_now_ratio=1-pt_pitch_now_ratio
  pt_speed_now_ratio=PanTiltStatus.speed_ratio



### Setup a regular background scan process based on timer callback
def pt_scan_timer_callback(timer):
  # Called periodically no matter what as a Timer object callback
  global set_pt_speed_ratio_pub
  global set_pt_pan_ratio_pub
  global set_pt_tilt_ratio_pub
  global pan_scan_direction
  global pt_tilt_scan_ratio
  global pt_yaw_now_deg
  global pt_pitch_now_deg
  global pt_pitch_now_ratio
  global pt_speed_now_ratio
  global object_detected
  print("Entering Scan Callback, Object Detection Value: " + str(object_detected))
  print("Current pan_ratio: " + "%.2f" % (pt_yaw_now_ratio))
  print("Current tilt_ratio: " + "%.2f" % (pt_pitch_now_ratio))
  print("Current speed_ratio: " + "%.2f" % (pt_speed_now_ratio))
  if not object_detected: # if not tracking, return to scan mode
    set_pt_speed_ratio_pub.publish(PT_SCAN_SPEED)
    print("No Targets Found, Entering Scan Mode")
    if pt_yaw_now_deg > PT_SCAN_PAN_LIMITS:
      #print("Soft Pan Limit Reached, Reversing Scan Direction")
      pan_scan_direction = -1
    elif pt_yaw_now_deg < (-1 * PT_SCAN_PAN_LIMITS):
      #print("Soft Pan Limit Reached, Reversing Scan Direction")
      pan_scan_direction = 1
    pt_scan_pan_ratio = pan_scan_direction
    if pt_scan_pan_ratio < 0:
      pt_scan_pan_ratio=0
    print("Current pan_scan_to_ratio: " + "%.2f" % (pt_scan_pan_ratio))
    print("Current tilt_scan_to_ratio: " + "%.2f" % (pt_tilt_scan_ratio))
    set_pt_pan_ratio_pub.publish(pt_scan_pan_ratio)
    set_pt_tilt_ratio_pub.publish(pt_tilt_scan_ratio)



  
### Detection and localization of object of interest relative to FOV center in degrees
def objects_detected_callback(bounding_box_msg):
  global pan_scan_direction
  global last_object_pan_ratio
  global object_detected
  global img_area
  box_of_interest = None
  print("Entering Detection Callback")
  # Iterate over all of the objects reported by the detector and return center of largest box in degrees relative to img center
  largest_box_area_ratio=0 # Initialize largest box area
  for box in bounding_box_msg.bounding_boxes:
    # Check for the object of interest and take appropriate actions
    if box.Class == OBJ_LABEL_OF_INTEREST:
      # Check if largest box
      box_area=(box.xmax-box.xmin)*(box.ymax-box.ymin)
      box_area_ratio = float(box_area) / img_area
      print("")
      print("box_area_ratio: " + "%.6f" % (box_area_ratio) )
      print("")
      if box_area_ratio > largest_box_area_ratio:
        largest_box_area_ratio=box_area_ratio
        largest_box=box
  if largest_box_area_ratio > MIN_DETECT_BOX_AREA_RATIO:
    box_of_interest = largest_box
    object_detected=True
    # Calculate the box center in image ratio terms
    object_tilt_ratio=PT_OBJECT_TILT_OFFSET_RATIO
    if PT_REVERSE_TILT:
      object_tilt_ratio=1-object_tilt_ratio
    object_loc_y_pix = box_of_interest.ymin + ((box_of_interest.ymax - box_of_interest.ymin)  * object_tilt_ratio) 
    object_loc_x_pix = box_of_interest.xmin + ((box_of_interest.xmax - box_of_interest.xmin)  / 2)
    object_loc_y_ratio = float(object_loc_y_pix) / img_height
    object_loc_x_ratio = float(object_loc_x_pix) / img_width
    #print("Object Detected " + OBJ_LABEL_OF_INTEREST + " with box center (" + str(object_loc_x_ratio) + ", " + str(object_loc_y_ratio) + ")")
    # Call the tracking algorithm
    pt_track_box(object_loc_y_ratio, object_loc_x_ratio)
    # Set next scan direction in direction of target
    pan_scan_direction = np.sign(last_object_pan_ratio-object_loc_x_ratio) # direction used if object lost an scan mode start up
    last_object_pan_ratio=object_loc_x_ratio
  else:
    # Object of interest not detected, so reset object_detected
    object_detected=False  # will start scan mode on next timer event
  
def found_object_callback(found_obj_msg):
  # Must reset object_detected in the event of no objects to restart scan mode
  global object_detected
  if found_obj_msg.count == 0:
    print("No objects found")
    object_detected=False


### Track box process based on current box center relative ratio of image
def pt_track_box(object_loc_y_ratio, object_loc_x_ratio):
  global object_detected
  global set_pt_speed_ratio_pub
  global set_pt_pan_ratio_pub
  global set_pt_tilt_ratio_pub
  print("Entering Track Callback, Object Detection Value: " + str(object_detected)) 
  if object_detected:
    print("Target Found, Entering Track Mode")
    # Simple bang/bang positional control with hysteresis band and error-proportional speed control
    # First check if we are close enough to center in either dimension to stop motion: Hysteresis band
    box_abs_error_x_ratio = 2.0 * abs(object_loc_x_ratio - 0.5)
    box_abs_error_y_ratio = 2.0 * abs(object_loc_y_ratio - 0.5)
    #print("Object Detection Error Ratios pan: " "%.2f" % (box_abs_error_x_ratio) + " tilt: " + "%.2f" % (box_abs_error_y_ratio))
    if (box_abs_error_y_ratio <= OBJ_CENTERED_BUFFER_RATIO ) or \
       (box_abs_error_x_ratio <= OBJ_CENTERED_BUFFER_RATIO ):
      #print_throttle(1.0, "Object is centered in frame in at least one axis: Stopping any p/t motion") 
      pt_stop_motion_pub.publish()
    # Now set the speed proportional to average error
    speed_control_value = PT_MIN_TRACK_SPEED + (PT_MAX_TRACK_SPEED-PT_MIN_TRACK_SPEED) * box_abs_error_x_ratio
    #print("Current track speed ratio: " + "%.2f" % (speed_control_value))
    set_pt_speed_ratio_pub.publish(speed_control_value)
    # Per-axis bang/bang based on direction
    if box_abs_error_y_ratio > OBJ_CENTERED_BUFFER_RATIO:
      print("Tilt Object Error To High, Adjusitng TilT") 
      tilt_axis_ratio_target = pt_forward_tilt_limit_ratio if object_loc_y_ratio < 0.5 else pt_backward_tilt_limit_ratio
      set_pt_tilt_ratio_pub.publish(tilt_axis_ratio_target)
      print("Current tilt_track_to_ratio: " + "%.2f" % (tilt_axis_ratio_target))
    if box_abs_error_x_ratio > OBJ_CENTERED_BUFFER_RATIO:
      print("Pan Object Error To High, Adjusitng Pan")     
      pan_axis_ratio_target = pt_forward_pan_limit_ratio if object_loc_x_ratio < 0.5 else pt_backward_pan_limit_ratio
      set_pt_pan_ratio_pub.publish(pan_axis_ratio_target)
      print("Current pan_track_to_ratio: " + "%.2f" % (pan_axis_ratio_target))
 

### Cleanup processes on node shutdown
def cleanup_actions():
  time.sleep(1)
  global stop_classifier_pub
  global send_pt_home_pub
  print("Shutting down: Executing script cleanup actions")
  print("Stopping object detector")
  stop_classifier_pub.publish()
  print("Send pan and tilt to home position")
  send_pt_home_pub.publish()
  time.sleep(2)


### Script Entrypoint
def startNode():
  rospy.loginfo("Starting PT_2D_CAMERA_PERSON_TRACKER automation script", disable_signals=True) # Disable signals so we can force a shutdown
  rospy.init_node(name="pt_2d_camera_person_tracker_auto_script")
  #initialize system including pan scan process
  initialize_actions()
  #Set up object detector subscriber which only updates on AI detection
  print("Starting object detection subscriber")
  rospy.Subscriber(BOUNDING_BOXES_TOPIC, BoundingBoxes, objects_detected_callback, queue_size = 1)
  #Set up found object subscriber which monitors all AI outputs
  print("Starting found object subscriber")
  rospy.Subscriber(FOUND_OBJECT_TOPIC, ObjectCount, found_object_callback, queue_size = 1)
  #Set up cleanup on node shutdown
  rospy.on_shutdown(cleanup_actions)
  # Spin forever (until object is detected)
  rospy.spin()


#####################################################################################
# Main
#####################################################################################
if __name__ == '__main__':
  startNode()   #initialize system


