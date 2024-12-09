#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

# Sample NEPI Process Script. 
# 1. Waits for ai detection topic
# 2. Waits for pan and tilt device
# 3. Tracks set target class

import os
#### ROS namespace setup
ros_namespace = '/nepi/s2x/'
os.environ["ROS_NAMESPACE"] = ros_namespace[0:-1] # remove to run as automation script
import rospy
import time
import sys
import copy
import threading
import statistics
import numpy as np
import cv2

from nepi_edge_sdk_base import nepi_ros 
from nepi_edge_sdk_base import nepi_msg

from std_msgs.msg import Bool, UInt8, Empty, Float32, String
from nepi_ros_interfaces.msg import PanTiltLimits, PanTiltPosition, SingleAxisTimedMove, PanTiltStatus, StringArray

from nepi_ros_interfaces.msg import BoundingBox, BoundingBoxes, ObjectCount, RangeWindow

from nepi_ros_interfaces.srv import ImageClassifierStatusQuery, ImageClassifierStatusQueryRequest, PTXCapabilitiesQuery



#########################################
# USER SETTINGS - Edit as Necessary 
#########################################

OBJECT_LABEL_OF_INTEREST = "person"
MIN_DETECT_BOX_AREA_RATIO = 0.01 # Filters background targets.

IMAGE_FOV_HORZ = 110
IMAGE_FOV_VERT = 70

PT_NAMESPACE = "ptx"
# Pan and Tilt scan settings
PT_SCAN_PAN_LIMIT_DEG = 40 # +- Pan Angle Limits
PT_SCAN_TILT_LIMIT_DEG = 30 # +- Tilt Angle Limits
PT_SCAN_TILT_RATIO = 0.15 # Tilt Angle During Scanning + Up
PT_SCAN_SPEED_RATIO = 0.8
PT_SCAN_CHECK_INTERVAL = 0.5

# Pan and Tilt tracking settings
PT_TRACK_SPEED_RATIO = 1.0
PT_OBJECT_TILT_OFFSET_RATIO = 0.15 # Adjust tilt center to lower or raise the calculated object center
PT_TRACK_ERROR_GOAL_DEG = 8 # Hysteresis band about center of image for tracking purposes


#########################################
# Node Class
#########################################

class pantilt_object_tracker():
  pt_connected = False
  has_position_feedback = False
  has_adjustable_speed = False
  pt_status_msg = None
  last_sel_pt = ""
  pt_status_msg = None
  current_scan_dir = 1
  
  pt_status_topic = ""
  pt_status_sub = None
  send_pt_home_pub = None
  set_pt_speed_ratio_pub  = None
  set_pt_position_pub  = None
  set_pt_pan_ratio_pub  = None
  set_pt_tilt_ratio_pub  = None
  set_pt_pan_jog_pub  = None
  set_pt_tilt_jog_pub  = None
  set_pt_soft_limits_pub  = None
  pt_stop_motion_pub  = None

  last_track_dir = -1
  last_pan_pos = 0
  is_moving = False

  no_object_count = 0
  lost_target_count = 0

  target_box_acquire = False
  target_box = None
  target_box_lock = threading.Lock()

  is_scanning = False
  is_tracking = False

  pitch_yaw_errors_deg = [0.0,0.0]
  
  #######################
  ### Node Initialization
  DEFAULT_NODE_NAME = "pantilt_object_tracker" 
  def __init__(self):
    #### APP NODE INIT SETUP ####
    nepi_ros.init_node(name= self.DEFAULT_NODE_NAME)
    self.node_name = nepi_ros.get_node_name()
    self.base_namespace = nepi_ros.get_base_namespace()
    nepi_msg.createMsgPublishers(self)
    nepi_msg.publishMsgInfo(self,"Starting Initialization Processes")
    ##############################
    ## Initialize Class Variables

    self.object_label_of_interest = OBJECT_LABEL_OF_INTEREST
    self.min_area_ratio = MIN_DETECT_BOX_AREA_RATIO
    self.img_width = 0 # Updated on receipt of first image
    self.img_height = 0 # Updated on receipt of first image
    self.img_area = 0 # Updated on receipt of first image
    self.object_detected = False
    self.lost_target_count = 0
    self.target_detected = False
    #self.target_box_lock.acquire()
    self.target_box = None      

    ## Define Class Namespaces
    # Wait for ptx status topic to publish
    pt_status_topic = os.path.join(PT_NAMESPACE,"status")
    nepi_msg.publishMsgInfo(self,"Looking for topic name: " + pt_status_topic)
    self.pt_status_topic=nepi_ros.wait_for_topic(pt_status_topic)
    nepi_msg.publishMsgInfo(self,"Found ptx status topic: " + self.pt_status_topic)
    ptx_namespace = self.pt_status_topic.replace("status","")
    nepi_msg.publishMsgInfo(self,"Found ptx namespace: " + ptx_namespace)
    self.pt_namespace = ptx_namespace.split("/ptx")[0]
    # PanTilt Status Topics
    # PanTilt Control Publish Topics
    PT_SET_SPEED_RATIO_TOPIC = ptx_namespace + "set_speed_ratio"
    PT_GOHOME_TOPIC = ptx_namespace + "go_home"
    PT_STOP_TOPIC = ptx_namespace + "stop_moving"
    PT_GOTO_PAN_RATIO_TOPIC = ptx_namespace + "jog_to_yaw_ratio"
    PT_GOTO_TILT_RATIO_TOPIC = ptx_namespace + "jog_to_pitch_ratio"
    PT_JOG_PAN_TOPIC = ptx_namespace + "jog_timed_yaw"
    PT_JOG_TILT_TOPIC = ptx_namespace + "jog_timed_pitch"
    PT_JOG_POSITION_TOPIC = ptx_namespace + "jog_to_position"
    PT_SET_SOFT_LIMITS_TOPIC = ptx_namespace + "set_soft_limits"

    ## Get PTX capabilities info
    ptx_capabilities_service_topic = ptx_namespace + "capabilities_query"
    try:
      ptx_caps_service = rospy.ServiceProxy(ptx_capabilities_service_topic, PTXCapabilitiesQuery)
      time.sleep(1)
      ptx_caps = ptx_caps_service()
      self.has_position_feedback = ptx_caps.absolute_positioning
      self.has_adjustable_speed =  ptx_caps.adjustable_speed
    except Exception as e:
      #nepi_msg.publishMsgWarn(self,"Failed to call PTX capabilities service: " + ptx_capabilities_service_topic + " " + str(e))
      self.has_position_feedback = False
      self.has_adjustable_speed =  False

    ## Create Publishers
    self.send_pt_home_pub = rospy.Publisher(PT_GOHOME_TOPIC, Empty, queue_size=10)
    self.set_pt_speed_ratio_pub = rospy.Publisher(PT_SET_SPEED_RATIO_TOPIC, Float32, queue_size=10)
    self.set_pt_position_pub = rospy.Publisher(PT_JOG_POSITION_TOPIC, PanTiltPosition, queue_size=10)
    self.set_pt_pan_ratio_pub = rospy.Publisher(PT_GOTO_PAN_RATIO_TOPIC, Float32, queue_size=10)
    self.set_pt_tilt_ratio_pub = rospy.Publisher(PT_GOTO_TILT_RATIO_TOPIC, Float32, queue_size=10)
    self.set_pt_pan_jog_pub = rospy.Publisher(PT_JOG_PAN_TOPIC, SingleAxisTimedMove, queue_size=10)
    self.set_pt_tilt_jog_pub = rospy.Publisher(PT_JOG_TILT_TOPIC, SingleAxisTimedMove, queue_size=10)
    self.set_pt_soft_limits_pub = rospy.Publisher(PT_SET_SOFT_LIMITS_TOPIC, PanTiltLimits, queue_size=10)
    self.pt_stop_motion_pub = rospy.Publisher(PT_STOP_TOPIC, Empty, queue_size=10)
    time.sleep(1)
    ## Create Subscribers
    nepi_msg.publishMsgInfo(self,"Subscribing to PTX Status Msg: " + self.pt_status_topic)
    self.pt_status_sub = rospy.Subscriber(self.pt_status_topic, PanTiltStatus, self.ptStatusCb, queue_size = 1)
    #self.pt_connected = True # Set in pt_status callback


    # Set up object detector subscriber
    # AI Detector Subscriber Topics
    AI_BOUNDING_BOXES_TOPIC = self.base_namespace + "ai_detector_mgr/bounding_boxes"
    AI_DETECTION_IMAGE_TOPIC = self.base_namespace + "ai_detector_mgr/detection_image"
    AI_FOUND_OBJECT_TOPIC = self.base_namespace + "ai_detector_mgr/found_object"
    ## Class subscribers
    rospy.loginfo("Connecting to NEPI Detector Image Topic")
    rospy.loginfo(AI_DETECTION_IMAGE_TOPIC )
    rospy.loginfo("Waiting for topic: " + AI_DETECTION_IMAGE_TOPIC)
    nepi_ros.wait_for_topic(AI_DETECTION_IMAGE_TOPIC)
    rospy.loginfo("Starting object detection subscriber: Object of interest = " + self.object_label_of_interest + "...")
    rospy.Subscriber(AI_BOUNDING_BOXES_TOPIC, BoundingBoxes, self.objectDetectedCb, queue_size = 1)
    #Set up found object subscriber which monitors all AI outputs
    rospy.loginfo("Starting found object subscriber")
    rospy.Subscriber(AI_FOUND_OBJECT_TOPIC, ObjectCount, self.foundObjectCb, queue_size = 1)
    ## Start Node Processes
    # Set up the timer that start scanning when no objects are detected
    nepi_msg.publishMsgInfo(self,"Setting up pan/tilt scan check timer")
    rospy.Timer(rospy.Duration(PT_SCAN_CHECK_INTERVAL), self.scanTrackCb)

    ## Initiation Complete
    nepi_msg.publishMsgInfo(self," Initialization Complete")
    # Spin forever (until object is detected)
    rospy.spin()
    ##############################



  #######################
  ### Node Methods

  ### Simple callback to get pt pt_status_msg info
  def ptStatusCb(self,pt_status_msg):
    # This is just to get the current pt positions
    self.pt_status_msg = pt_status_msg
    if self.pt_status_msg.yaw_now_deg != self.last_pan_pos:
      self.last_pan_pos = self.pt_status_msg.yaw_now_deg
      self.is_moving = True
    else:
      self.is_moving = False
    self.pt_connected = True
    
   ### If object(s) detected, save bounding box info to global
  def objectDetectedCb(self,bounding_boxes_msg):
    selected_class = self.object_label_of_interest
    min_area_ratio =  self.min_area_ratio
    ros_timestamp = bounding_boxes_msg.header.stamp
    bb_list = bounding_boxes_msg.bounding_boxes
    self.img_height = bounding_boxes_msg.image_height
    self.img_width = bounding_boxes_msg.image_width
    # Iterate over all of the objects reported by the detector and return center of largest box in degrees relative to img center
    largest_target = None
    largest_box_area_ratio=0 # Initialize largest box area
    for box in bb_list:
      # Check for the object of interest and take appropriate actions
      ##nepi_msg.publishMsgWarn(self,"Looking for selected target: " + str(selected_class))
      if box.Class == selected_class:
        # Check if largest box
        box_area_ratio = box.area_ratio
        if box_area_ratio > largest_box_area_ratio:
          largest_box_area_ratio=box_area_ratio
          largest_target=box
    if largest_target == None:
          self.lost_target_count += 1
          self.target_detected = False
          #self.target_box_lock.acquire()
          self.target_box = None      
          #self.target_box_lock.release()
    elif largest_box_area_ratio < self.min_area_ratio and self.min_area_ratio != 0:
          self.lost_target_count += 1
          self.target_detected = False
          #self.target_box_lock.acquire()
          self.target_box = None      
          #self.target_box_lock.release()
    else:
          #nepi_msg.publishMsgWarn(self,"Got target loc: " + str(largest_target))
          self.lost_target_count = 0
          self.target_detected = True
          #self.target_box_lock.acquire()
          self.target_box = largest_target      
          #self.target_box_lock.release()

            
  ### Monitor Output of AI model to clear detection status
  def foundObjectCb(self,found_obj_msg):
    #Clean Up
    if found_obj_msg.count == 0:      
      self.lost_target_count += 1
      self.target_detected = False
      #self.target_box_lock.acquire()
      self.target_box = None      
      #self.target_box_lock.release()


  def scanTrackCb(self,timer):
    ##nepi_msg.publishMsgWarn(self,"Starting scan track process loop")
    #self.target_box_lock.acquire()
    box = copy.deepcopy(self.target_box)      
    #self.target_box_lock.release()

    #nepi_msg.publishMsgWarn(self,"2")

    min_pan = -1 * PT_SCAN_PAN_LIMIT_DEG
    max_pan = PT_SCAN_PAN_LIMIT_DEG
    min_tilt = -1 * PT_SCAN_TILT_LIMIT_DEG
    max_tilt = PT_SCAN_TILT_LIMIT_DEG

    #nepi_msg.publishMsgWarn(self,"3")

    lost_target = self.lost_target_count > 5

    was_tracking = copy.deepcopy(self.is_tracking)
    was_scanning = copy.deepcopy(self.is_scanning)

    #nepi_msg.publishMsgWarn(self,"Running scan track process with pt_connected: " + str(self.pt_connected))
    #nepi_msg.publishMsgWarn(self,"Running scan track process with pt_status valid: " + str(self.pt_status_msg is not None))

    if box is not None:
      #nepi_msg.publishMsgWarn(self,"Tracking on target box: " + str(box))
      self.is_tracking = True
      self.is_scanning = False
      error_goal = PT_TRACK_ERROR_GOAL_DEG
      track_speed_ratio = PT_TRACK_SPEED_RATIO
      track_tilt_offset = PT_OBJECT_TILT_OFFSET_RATIO
      if self.has_adjustable_speed == True and self.pt_connected == True:
        try:
          self.set_pt_speed_ratio_pub.publish(track_speed_ratio)
        except:
          pass

      tilt_cur = self.pt_status_msg.pitch_now_deg
      tilt_goal = self.pt_status_msg.pitch_goal_deg

      pan_cur = self.pt_status_msg.yaw_now_deg
      pan_goal = self.pt_status_msg.yaw_goal_deg


      [pan_error,tilt_error] = self.get_target_bearings(box)
      tilt_error = tilt_error + track_tilt_offset
      self.pitch_yaw_errors_deg = [pan_error,tilt_error]
      #nepi_msg.publishMsgWarn(self,"Error Goal set to: " + str(error_goal))
      #nepi_msg.publishMsgWarn(self,"Got Targets Errors pan tilt: " + str(self.pitch_yaw_errors_deg))

      if abs(tilt_error) < error_goal and abs(pan_error) < error_goal and self.pt_connected == True:
          #nepi_msg.publishMsgWarn(self,"Tracking within error bounds")
          try:
            self.pt_stop_motion_pub.publish(Empty())
          except:
            pass
      else:
          # Set pan angle goal
          if abs(pan_error) > error_goal:
              pan_to_goal = pan_cur + pan_error/3
          else:
              pan_to_goal = pan_cur
          if pan_to_goal < min_pan:
              pan_to_goal = min_pan
          if pan_to_goal > max_pan:
              pan_to_goal = max_pan
          # Set tilt angle goal
          if abs(tilt_error) > error_goal:
              tilt_to_goal = tilt_cur + tilt_error/3 
          else:
              tilt_to_goal = tilt_cur
          if tilt_to_goal < min_tilt:
              tilt_to_goal = min_tilt
          if tilt_to_goal > max_tilt:
              tilt_to_goal = max_tilt

          #nepi_msg.publishMsgWarn(self,"Current Pos: " + str([pan_cur,tilt_cur]))
          #nepi_msg.publishMsgWarn(self,"Track to Pos: " + str([pan_to_goal,tilt_to_goal]))
          if self.has_position_feedback == True:
            # Send angle goal
            pt_pos_msg = PanTiltPosition()
            pt_pos_msg.yaw_deg = pan_to_goal
            pt_pos_msg.pitch_deg = tilt_to_goal
            if not nepi_ros.is_shutdown() and self.pt_connected == True:
              try:
                self.set_pt_position_pub.publish(pt_pos_msg)   
              except:
                pass
            else:
              pass # add timed jog controls


          if pan_error > 0:
              self.last_track_dir = 1
          else: 
              self.last_track_dir = -1
          #nepi_msg.publishMsgWarn(self,"Track dir: " + str(self.last_track_dir))
    elif lost_target == True:
      self.is_tracking = False
      self.is_scanning = True
      if was_tracking == True or was_scanning == False:
          start_scanning = True
          self.current_scan_dir = self.last_track_dir

      self.pitch_yaw_errors_deg = [0,0]
      
      tilt_cur = self.pt_status_msg.pitch_now_deg
      tilt_goal = self.pt_status_msg.pitch_goal_deg
      pan_cur = self.pt_status_msg.yaw_now_deg
      pan_goal = self.pt_status_msg.yaw_goal_deg

      start_scanning = False


      if self.has_adjustable_speed == True and self.pt_connected == True:
        try:
          self.set_pt_speed_ratio_pub.publish(PT_SCAN_SPEED_RATIO)
        except:
          pass

      #nepi_msg.publishMsgWarn(self,"Scanning in direction: " + str(self.current_scan_dir))

      # Check if scan dir change needed
      check_str = str([pan_cur,min_pan + 5,max_pan - 5,self.current_scan_dir])
      #nepi_msg.publishMsgWarn(self,"Scanning with scan checks: " + check_str)


      if self.has_position_feedback == True and self.pt_connected == True:
        if ((pan_cur < (min_pan + 5)) and self.current_scan_dir != 1):
          pan_tilt_pos_msg = PanTiltPosition()
          pan_tilt_pos_msg.yaw_deg = max_pan
          pan_tilt_pos_msg.pitch_deg = PT_SCAN_TILT_RATIO
          try:
            self.set_pt_position_pub.publish(pan_tilt_pos_msg)
          except:
            pass
          self.current_scan_dir = 1
          #nepi_msg.publishMsgWarn(self,"Changed to scan dir: " + str(self.current_scan_dir))

        elif (pan_cur > (max_pan - 5)) and self.current_scan_dir != -1:
          pan_tilt_pos_msg = PanTiltPosition()
          pan_tilt_pos_msg.yaw_deg = min_pan
          pan_tilt_pos_msg.pitch_deg = PT_SCAN_TILT_RATIO
          try:
            self.set_pt_position_pub.publish(pan_tilt_pos_msg)
          except:
            pass
          self.current_scan_dir = -1
          #nepi_msg.publishMsgInfo(self,"Changed to scan dir: " + str(self.current_scan_dir))

        elif (start_scanning == True or self.is_moving == False) and self.pt_connected == True:
          if self.current_scan_dir > 0:
            pan_tilt_pos_msg = PanTiltPosition()
            pan_tilt_pos_msg.yaw_deg = max_pan
            pan_tilt_pos_msg.pitch_deg = PT_SCAN_TILT_RATIO
            try:
              self.set_pt_position_pub.publish(pan_tilt_pos_msg)
            except:
              pass
          else:
            pan_tilt_pos_msg = PanTiltPosition()
            pan_tilt_pos_msg.yaw_deg = min_pan
            pan_tilt_pos_msg.pitch_deg = PT_SCAN_TILT_RATIO
            try:
              self.set_pt_position_pub.publish(pan_tilt_pos_msg)
            except:
              pass
            nepi_msg.publishMsgInfo(self,"Changed to scan dir: " + str(self.current_scan_dir))
      else:
        pass # Add timed jog controls



  def get_target_bearings(self,box):
      target_vert_angle_deg = 0
      target_horz_angle_deg = 0
      if self.img_height != 0 and self.img_width != 0:
        # Iterate over all of the objects and calculate range and bearing data
        image_fov_vert = IMAGE_FOV_VERT
        image_fov_horz = IMAGE_FOV_HORZ
        box_y = box.ymin + (box.ymax - box.ymin)
        box_x = box.xmin + (box.xmax - box.xmin)
        box_center = [box_y,box_x]
        y_len = (box.ymax - box.ymin)
        x_len = (box.xmax - box.xmin)
        # Calculate target bearings
        object_loc_y_pix = float(box.ymin + ((box.ymax - box.ymin))  / 2) 
        object_loc_x_pix = float(box.xmin + ((box.xmax - box.xmin))  / 2)
        object_loc_y_ratio_from_center = float(object_loc_y_pix - self.img_height/2) / float(self.img_height/2)
        object_loc_x_ratio_from_center = float(object_loc_x_pix - self.img_width/2) / float(self.img_width/2)
        target_vert_angle_deg = (object_loc_y_ratio_from_center * float(image_fov_vert/2))
        target_horz_angle_deg = -1* (object_loc_x_ratio_from_center * float(image_fov_horz/2))
      return target_horz_angle_deg, target_vert_angle_deg



  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    global led_intensity_pub
    rospy.loginfo("Shutting down: Executing script cleanup actions")



#########################################
# Main
#########################################
if __name__ == '__main__':
  pantilt_object_tracker()


