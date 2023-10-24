#!/usr/bin/env python

# Sample NEPI Automation Script. 
# Uses onboard ROS python library to
# 1. Runs a process that convert zed stereo camera depthmap depth_array and depth_image
# 2. Runs a process to load an AI model and connect to left camera image
# 3. Runs a process to calculate range and bearing of detected targets
# 2. Runs until Stopped

import time
import sys
import rospy
import numpy as np
import cv2 as cv

from sensor_msgs.msg import Image
from rospy.numpy_msg import numpy_msg
from cv_bridge import CvBridge
from std_msgs.msg import UInt8, Empty, String, Bool
from nepi_ros_interfaces.msg import ClassifierSelection, StringArray
from darknet_ros_msgs.msg import BoundingBoxes, ObjectCount

#####################################################################################
# SETUP - Edit as Necessary ##################################
##########################################

#Set Runtime User Settings
DEPTH_IMAGE_MIN_RANGE_METERS=0.25 #Set to meter value to adjust depth image active range
DEPTH_IMAGE_MAX_RANGE_METERS=10.0 #Set to meter value to adjust depth image active range

TARGET_BOX_PERCENT=50 # Sets the percent of target box around center to use for range calc
TARGET_BOX_METERS=0.3 # Sets the depth filter around mean depth to use for range calc


# ROS namespace setup
BASE_NAMESPACE = "/nepi/s2x/"

###!!!!!!!! Set data input stream topics and parameters !!!!!!!!
IMAGE_INPUT_TOPIC = BASE_NAMESPACE + "zed2/zed_node/left/image_rect_color"
FOV_VERT_DEG=70 # Camera Vertical Field of View (FOV)
FOV_HORZ_DEG=110 # Camera Horizontal Field of View (FOV)

DEPTH_DATA_INPUT_TOPIC = BASE_NAMESPACE + "zed2/zed_node/depth/depth_registered"
DEPTH_IMAGE_OUTPUT_TOPIC = BASE_NAMESPACE + "zed2/zed_node/depth/depth_image"

###!!!!!!!! Set data output stream topics and parameters !!!!!!!!
TARGET_DATA_OUTPUT_TOPIC = BASE_NAMESPACE + "targeting/target_range_bearing_data"
TARGET_IMAGE_OUTPUT_TOPIC = BASE_NAMESPACE + "targeting/targeting_image"

### Classifier topics and parameters
BOUNDING_BOXES_TOPIC = BASE_NAMESPACE + "classifier/bounding_boxes"
FOUND_OBJECT_TOPIC = BASE_NAMESPACE + "classifier/found_object"
START_CLASSIFIER_TOPIC = BASE_NAMESPACE + "start_classifier"
STOP_CLASSIFIER_TOPIC = BASE_NAMESPACE + "stop_classifier"
DETECTION_MODEL = "common_object_detection"
DETECTION_THRESHOLD = 0.5

#####################################################################################
# Globals
#####################################################################################

depth_image_pub = rospy.Publisher(DEPTH_IMAGE_OUTPUT_TOPIC, Image, queue_size=10)
#target_data_pub = rospy.Publisher(TARGET_DATA_OUTPUT_TOPIC, , queue_size=10)
target_overlay_pub = rospy.Publisher(TARGET_IMAGE_OUTPUT_TOPIC, Image, queue_size=10)
stop_classifier_pub = rospy.Publisher(STOP_CLASSIFIER_TOPIC, Empty, queue_size=10)

np_depth_array_m=None # will be replaced when depthmap is recieved and converted
img_height=0 # Updated on receipt of first image
img_width=0 # Updated on receipt of first image
detect_boxes=None

#####################################################################################
# Methods
#####################################################################################

### System Initialization processes
def initialize_actions():
  print("")
  rospy.loginfo("Initializing " + CAMERA_NAMESPACE )
  rospy.loginfo("Connecting to ROS Topic" + IMAGE_INPUT_TOPIC )
  # Check if camera topic is publishing
  topic_list=rospy.get_published_topics(namespace='/')
  topic_to_connect=[IMAGE_INPUT_TOPIC, 'sensor_msgs/Image']
  if topic_to_connect in topic_list: 
    print("Camera topic found, starting initializing process")
    # Classifier initialization
    start_classifier_pub = rospy.Publisher(START_CLASSIFIER_TOPIC, ClassifierSelection, queue_size=10)
    classifier_selection = ClassifierSelection(img_topic=CAMERA_IMAGE_INPUT_TOPIC, classifier=DETECTION_MODEL, detection_threshold=DETECTION_THRESHOLD)
    time.sleep(1) # Important to sleep between publisher constructor and publish()
    print("Starting target detector: " + str(start_classifier_pub.name))
    start_classifier_pub.publish(classifier_selection)
    print("Detector initialization complete")
  else: 
    print("!!!!! Camera topic not found, shutting down")
    time.sleep(1)
    rospy.signal_shutdown("Camera topic not found")
    


### callback to get depthmap, convert to global float array and rgb image, then publish depth_image
def convert_depthmap_callback(depth_data):
  global np_depth_array_m
  # Zed depth data is floats in m, but passed as 4 bytes each that must be converted to floats
  # Use cv_bridge() to convert the ROS image to OpenCV format
  #Convert the depth 4xbyte data to global float meter array
  bridge = CvBridge()
  cv_depth_image = bridge.imgmsg_to_cv2(depth_data, desired_encoding="passthrough")
  np_depth_array_m = (np.array(cv_depth_image, dtype=np.float32)) # replace nan values
  np_depth_array_m[np.isnan(np_depth_array_m)] = 0
  #img_height, img_width = np_depth_array_m.shape[:2]
  # Create thresholded and 255 scaled version
  min_range_m=DEPTH_IMAGE_MIN_RANGE_METERS
  max_range_m=DEPTH_IMAGE_MAX_RANGE_METERS
  np_depth_array_scaled = np_depth_array_m
  np_depth_array_scaled[np_depth_array_scaled < min_range_m] = max_range_m # put to max
  np_depth_array_scaled[np_depth_array_scaled > max_range_m] = max_range_m # put to max
  np_depth_array_scaled=np_depth_array_scaled-min_range_m
  depth_scaler=np.amax(np_depth_array_m) # use max range as scalar
  np_depth_array_scaled = np.array(255*np_depth_array_m/depth_scaler,np.uint8)
  max_value=np.max(np_depth_array_scaled)
  np_depth_array_scaled=np.array(np.abs(np_depth_array_scaled-float(max_value)),np.uint8) # Reverse for colormaping
  # Convert to CV color image using colormap
  cv_depth_image_color = cv.applyColorMap(np_depth_array_scaled, cv.COLORMAP_JET)
  ros_depth_image = bridge.cv2_to_imgmsg(cv_depth_image_color,"bgr8")
  depth_image_pub.publish(ros_depth_image)

### Monitor Output of AI model to clear detection status
def found_object_callback(found_obj_msg):
  # Must reset target lists if no targets are detected
  global detect_boxes
  if found_obj_msg.count == 0:
    print("No objects found")
    detect_boxes=None

### If object(s) detected, save bounding box info to global
def object_detected_callback(bounding_box_msg):
  global detect_boxes
  detect_boxes=bounding_box_msg


### calculate range and bearing of AI detected targets
def object_targeting_callback(img_msg):
  global detect_boxes
  global np_depth_array_m
  global img_height
  global img_width
  img_height = img_msg.height
  img_width = img_msg.width
  target_labels=[]
  target_ranges_m=[]
  target_bearing_horz_deg=[]
  target_bearing_vert_deg=[]
  # Convert ROS image to OpenCV for editing
  bridge = CvBridge()
  cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
  # Iterate over all of the objects and calculate range and bearing data
  if detect_boxes is not None:
    for box in detect_boxes.bounding_boxes:
      # Get target label
      target_labels.append(box.Class)
      # Calculate target range
      if np_depth_array_m is not None:
        print(np_depth_array_m.shape)
        # Caclulate target reduced target window based on user settings
        print([box.ymin,box.ymax])
        box_reduction_y_pix=float(box.ymax - box.ymin)*(1-TARGET_BOX_PERCENT/100)/2
        print("y_reduction_pixs: " + "%.2f" % box_reduction_y_pix)
        ymin_adj=int(box.ymin + box_reduction_y_pix)
        ymax_adj=int(box.ymax - box_reduction_y_pix)
        print([box.xmin,box.xmax])
        box_reduction_x_pix=float(box.xmax - box.xmin)*(1-TARGET_BOX_PERCENT/100)/2
        print("x_reduction_pixs: " + "%.2f" % box_reduction_x_pix)
        xmin_adj=int(box.xmin + box_reduction_x_pix)
        xmax_adj=int(box.xmax - box_reduction_x_pix)
        print([ymin_adj,ymax_adj,xmin_adj,xmax_adj])
        # Get target range from cropped and filtered depth data
        depth_box_adj= np_depth_array_m[ymin_adj:ymax_adj,xmin_adj:xmax_adj]
        print(depth_box_adj.shape)
        depth_mean_val=np.mean(depth_box_adj)
        depth_box_adj[depth_box_adj > (depth_mean_val+TARGET_BOX_METERS/2)]=np.nan
        depth_box_adj[depth_box_adj < (depth_mean_val-TARGET_BOX_METERS/2)]=np.nan
        target_range_m=np.mean(depth_box_adj)
        print("target range: " + "%.2f" % target_range_m)
        target_ranges_m.append(target_range_m)
      else:
        target_ranges_m.append(np.nan)        
      # Calculate target bearings
      object_loc_y_pix = float(box.ymin + ((box.ymax - box.ymin))  / 2) 
      object_loc_x_pix = float(box.xmin + ((box.xmax - box.xmin))  / 2)
      object_loc_y_ratio_from_center = float(object_loc_y_pix - img_height/2) / float(img_height/2)
      object_loc_x_ratio_from_center = float(object_loc_x_pix - img_width/2) / float(img_width/2)
      print("x_ratio: " + "%.2f" % object_loc_x_ratio_from_center)
      print("y_ratio: " + "%.2f" % object_loc_y_ratio_from_center)


      target_vert_angle = (object_loc_y_ratio_from_center * float(FOV_VERT_DEG/2))
      target_horz_angle = (object_loc_x_ratio_from_center * float(FOV_HORZ_DEG/2))
      target_bearing_vert_deg.append(target_vert_angle)
      target_bearing_horz_deg.append(target_horz_angle)
      print("horz angle: " + "%.2f" % target_horz_angle)
      print("vert angle: " + "%.2f" % target_vert_angle)
      # Overlay data on OpenCV image
      font                   = cv.FONT_HERSHEY_SIMPLEX
      fontScale              = 0.5
      fontColor              = (0, 255, 0)
      thickness              = 1
      lineType               = 1

      text2overlay="%.1f" % target_range_m + "m," + "%.f" % target_horz_angle + "d," + "%.f" % target_vert_angle + "d"
      print(text2overlay)
      bottomLeftCornerOfText = (int(object_loc_x_pix),int(object_loc_y_pix))
      print(bottomLeftCornerOfText)
      print("")
      cv.putText(cv_image,text2overlay, 
        bottomLeftCornerOfText, 
        font, 
        fontScale,
        fontColor,
        thickness,
        lineType)
  #Convert OpenCV image to ROS image
  img_out_msg = bridge.cv2_to_imgmsg(cv_image,"bgr8")#desired_encoding='passthrough')
  # Publish new image
  target_overlay_pub.publish(img_out_msg)
  

### Cleanup processes on node shutdown
def cleanup_actions():
  print("Starting cleanup") 
  print("Stopping object detector")
  stop_classifier_pub.publish()
  time.sleep(2)


### Script Entrypoint
def startNode():
  rospy.loginfo("Starting Image Depthmap to Image script", disable_signals=True) # Disable signals so we can force a shutdown
  rospy.init_node
  rospy.init_node(name="zed_targeting_auto_script")


  #initialize system and start processes
  initialize_actions()
  rospy.Subscriber(FOUND_OBJECT_TOPIC, ObjectCount, found_object_callback)
  print("Starting object detection subscriber")
  rospy.Subscriber(BOUNDING_BOXES_TOPIC, BoundingBoxes, object_detected_callback)
  print("Starting targeteting subscriber")
  rospy.Subscriber(IMAGE_INPUT_TOPIC, Image, object_targeting_callback)
  print("Starting convert depthmap subscriber")
  rospy.Subscriber(DEPTH_DATA_INPUT_TOPIC, numpy_msg(Image), convert_depthmap_callback)
  start_target_detector()
  

  # run cleanup actions on shutdown
  rospy.on_shutdown(cleanup_actions)
  # Spin forever
  rospy.spin()


#####################################################################################
# Main
#####################################################################################

if __name__ == '__main__':
  startNode()

