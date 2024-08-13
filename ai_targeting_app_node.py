#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#


import os
# ROS namespace setup
NEPI_BASE_NAMESPACE = '/nepi/s2x/'
os.environ["ROS_NAMESPACE"] = NEPI_BASE_NAMESPACE[0:-1]
import rospy



import time
import sys
import numpy as np
import cv2
import copy
import math

from nepi_edge_sdk_base import nepi_ros
from nepi_edge_sdk_base import nepi_pc 
from nepi_edge_sdk_base import nepi_img 

from std_msgs.msg import UInt8, Int32, Float32, Empty, String, Bool, Header
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image, PointCloud2
from rospy.numpy_msg import numpy_msg
from cv_bridge import CvBridge
from nepi_ros_interfaces.msg import ClassifierSelection,  \
                                    BoundingBox, BoundingBoxes, BoundingBox3D, BoundingBoxes3D, \
                                    ObjectCount, ClassifierSelection, \
                                    AiTargetingStatus, AiTargetingAddClass, \
                                    StringArray, TargetLocalization, TargetLocalizations
from nepi_ros_interfaces.srv import ImageClassifierStatusQuery, ImageClassifierStatusQueryRequest

from nepi_edge_sdk_base.save_data_if import SaveDataIF
from nepi_edge_sdk_base.save_cfg_if import SaveCfgIF




#########################################

#########################################
# Node Class
#########################################

class NepiAiTargetingApp(object):

  #Set Initial Values
  FACTORY_TARGETING_ENBABLED=True
  FACTORY_FOV_VERT_DEG=70 # Camera Vertical Field of View (FOV)
  FACTORY_FOV_HORZ_DEG=110 # Camera Horizontal Field of View (FOV)
  FACTORY_TARGET_BOX_REDUCTION_PERCENT=50 # Sets the percent of target box around center to use for range calc
  FACTORY_TARGET_DEPTH_METERS=0.3 # Sets the depth filter around mean depth to use for range calc
  FACTORY_TARGET_MIN_POINTS=10 # Sets the minimum number of valid points to consider for a valid range
  FACTORY_TARGET_MAX_AGE_SEC=10 # Remove lost targets from dictionary if older than this age

  NONE_CLASSES_DICT = dict()
  NONE_CLASSES_DICT["None"] = {'depth': FACTORY_TARGET_DEPTH_METERS}

  EMPTY_TARGET_DICT = dict()
  EMPTY_TARGET_DICT["None"] = {'class': 'None', 
                                    'last_center_px': [0,0],
                                    'last_velocity_pxps': [0,0],
                                    'last_center_m': [0,0,0],
                                    'last_velocity_mps': [0,0,0],
                                    'last_detection_timestamp': rospy.Duration(0)                              
                                    }

  data_products = ["detection_image","targeting_image","bounding_boxes_2d","bounding_boxes_3d","tartgeting_data"]
  
  current_classifier = "None"
  current_classifier_state = "None"
  current_classifier_classes = "['None']"

  current_image_topic = "None"
  current_image_header = Header()
  img_width = 0
  img_height = 0
  image_sub = None

  depth_map_topic = "None"
  depth_map_header = Header()
  depth_map_sub = None
  np_depth_array_m = None
  pointcloud_topic = ""
  pointcloud_sub = None

  detect_boxes_msg = None
  bounding_box_3d_list = None

  last_targeting_enable = False
  last_image_topic = "None"
  
  selected_classes_dict = dict()
  current_targets_dict = dict()
  lost_targets_dict = dict()
  selected_target = "All"

  targeting_class_list = []
  targeting_target_list = []

  seq = 0

  #######################
  ### App Config Functions

  def resetAppCb(self,msg):
    self.resetApp()

  def resetApp(self):
    rospy.set_param('~targeting_enabled', self.FACTORY_TARGETING_ENBABLED)
    rospy.set_param('~last_classifier', "")
    rospy.set_param('~selected_classes_dict', dict())
    rospy.set_param('~image_fov_vert',  self.FACTORY_FOV_VERT_DEG)
    rospy.set_param('~image_fov_horz', self.FACTORY_FOV_HORZ_DEG)
    rospy.set_param('~target_box_reduction',  self.FACTORY_TARGET_BOX_REDUCTION_PERCENT)
    rospy.set_param('~default_target_depth',  self.FACTORY_TARGET_DEPTH_METERS)
    rospy.set_param('~target_min_points', self.FACTORY_TARGET_MIN_POINTS)
    rospy.set_param('~target_age_filter', self.FACTORY_TARGET_MAX_AGE_SEC)
    self.current_targets_dict = dict()
    self.lost_targets_dict = dict()
    self.publish_status()

  def saveConfigCb(self, msg):  # Just update class init values. Saving done by Config IF system
    pass # Left empty for sim, Should update from param server

  def setCurrentAsDefault(self):
    self.initParamServerValues(do_updates = False)

  def updateFromParamServer(self):
    #rospy.logwarn("Debugging: param_dict = " + str(param_dict))
    #Run any functions that need updating on value change
    # Don't need to run any additional functions
    pass

  def initParamServerValues(self,do_updates = True):
      rospy.loginfo("AI_APP: Setting init values to param values")
      self.init_enable_targeting = rospy.get_param('~targeting_enabled', self.FACTORY_TARGETING_ENBABLED)
      self.init_last_classifier = rospy.get_param("~last_classifier", "")
      self.init_selected_classes_dict = rospy.get_param('~selected_classes_dict', dict())
      self.init_image_fov_vert = rospy.get_param('~image_fov_vert',  self.FACTORY_FOV_VERT_DEG)
      self.init_image_fov_horz = rospy.get_param('~image_fov_horz', self.FACTORY_FOV_HORZ_DEG)
      self.init_target_box_reduction = rospy.get_param('~target_box_reduction',  self.FACTORY_TARGET_BOX_REDUCTION_PERCENT)
      self.init_default_target_depth = rospy.get_param('~default_target_depth',  self.FACTORY_TARGET_DEPTH_METERS)
      self.init_target_min_points = rospy.get_param('~target_min_points', self.FACTORY_TARGET_MIN_POINTS)
      self.init_target_age_filter = rospy.get_param('~target_age_filter', self.FACTORY_TARGET_MAX_AGE_SEC)
      self.resetParamServer(do_updates)

  def resetParamServer(self,do_updates = True):
      rospy.get_param('~targeting_enabled', self.init_enable_targeting)
      rospy.set_param('~last_classiier', self.init_last_classifier)
      rospy.set_param('~selected_classes_dict', self.init_selected_classes_dict)
      rospy.set_param('~image_fov_vert',  self.init_image_fov_vert)
      rospy.set_param('~image_fov_horz', self.init_image_fov_horz)
      rospy.set_param('~target_box_reduction',  self.init_target_box_reduction)
      rospy.set_param('~default_target_depth',  self.init_default_target_depth)
      rospy.set_param('~target_min_points', self.init_target_min_points)
      rospy.set_param('~target_age_filter', self.init_target_age_filter)
      if do_updates:
          self.updateFromParamServer()
          self.publish_status()



  ###################
  ## AI Manager Passthrough Callbacks

  def startClassifierCb(self,msg):
    ##rospy.loginfo(msg)
    self.start_classifier_pub.publish(msg)

  def stopClassifierCb(self,msg):
    ##rospy.loginfo(msg)
    self.stop_classifier_pub.publish(msg)

  def setThresholdCb(self,msg):
    ##rospy.loginfo(msg)
    self.set_threshold_pub.publish(msg)

  ###################
  ## AI App Callbacks

  def enableTargetingCb(self,msg):
    ##rospy.loginfo(msg)
    rospy.set_param('~targeting_enabled', msg.data)
    self.publish_status()

  def addAllClassesCb(self,msg):
    ##rospy.loginfo(msg)
    classes = self.current_classifier_classes
    depth = rospy.get_param('~default_target_depth',self.init_default_target_depth)
    selected_dict = dict()
    for Class in classes:
      selected_dict[Class] = {'depth': depth }
    rospy.set_param('~selected_classes_dict', selected_dict)
    self.publish_status()

  def removeAllClassesCb(self,msg):
    ##rospy.loginfo(msg)
    rospy.set_param('~selected_classes_dict', dict())
    self.publish_status()


  def addClassCb(self,msg):
    ##rospy.loginfo(msg)
    class_name = msg.class_name
    class_depth_m = msg.class_depth_m
    if class_name == msg.class_name:
      selected_classes_dict = rospy.get_param('~selected_classes_dict', self.init_selected_classes_dict)
      selected_classes_dict[class_name] = {'depth': class_depth_m}
      rospy.set_param('~selected_classes_dict', selected_classes_dict)
    self.publish_status()

  def removeClassCb(self,msg):
    ##rospy.loginfo(msg)
    class_name = msg.data
    selected_classes_dict = rospy.get_param('~selected_classes_dict', self.init_selected_classes_dict)
    if class_name in selected_classes_dict.keys():
      del selected_classes_dict[class_name]
      rospy.set_param('~selected_classes_dict', selected_classes_dict)
    self.publish_status()

  def selectTargetCb(self,msg):
    ##rospy.loginfo(msg)
    target_name = msg.data
    if target_name == 'All' or target_name in self.current_targets_dict.keys():
      self.selected_target = target_name
    self.publish_status()

  def setVertFovCb(self,msg):
    ##rospy.loginfo(msg)
    fov = msg.data
    if fov > 0:
      rospy.set_param('~image_fov_vert',  fov)
    self.publish_status()


  def setHorzFovCb(self,msg):
    ##rospy.loginfo(msg)
    fov = msg.data
    if fov > 0:
      rospy.set_param('~image_fov_horz',  fov)
    self.publish_status()
    
  def setBoxReductionCb(self,msg):
    #rospy.loginfo(msg)
    val = msg.data
    if val >= 0 and val <= 1:
      rospy.set_param('~target_box_reduction',val)
    self.publish_status()   
      
  def setDefaultTargetDepthCb(self,msg):
    #rospy.loginfo(msg)
    val = msg.data
    if val >= 0:
      rospy.set_param('~default_target_depth',val)
    self.publish_status()   

  def setTargetMinPointsCb(self,msg):
    #rospy.loginfo(msg)
    val = msg.data
    if val >= 0:
      rospy.set_param('~target_min_points',val)
    self.publish_status() 

  def setAgeFilterCb(self,msg):
    #rospy.loginfo(msg)
    val = msg.data
    if val >= 0:
      rospy.set_param('~target_age_filter',val)
    self.publish_status()


  #######################
  ### Node Initialization
  def __init__(self):
   
    rospy.loginfo("AI_APP: Starting Initialization Processes")
    self.initParamServerValues(do_updates = False)
    self.resetParamServer(do_updates = False)
   

    # Set up save data and save config services ########################################################
    self.save_data_if = SaveDataIF(data_product_names = self.data_products)
    # Temp Fix until added as NEPI ROS Node
    self.save_cfg_if = SaveCfgIF(updateParamsCallback=self.initParamServerValues, 
                                 paramsModifiedCallback=self.updateFromParamServer)


    ## App Setup ########################################################
    app_reset_app_sub = rospy.Subscriber('~reset_app', Empty, self.resetAppCb, queue_size = 10)
    self.initParamServerValues(do_updates=False)

    # AI Management Scubscirbers and publishers
    '''
    rospy.Subscriber('~start_classifier', ClassifierSelection, self.startClassifierCb)
    self.start_classifier_pub = rospy.Publisher(NEPI_BASE_NAMESPACE + "ai_detector_mgr/start_classifier", ClassifierSelection, queue_size=1)
    rospy.Subscriber('~stop_classifier', Empty, self.stopClassifierCb)
    self.stop_classifier_pub = rospy.Publisher(NEPI_BASE_NAMESPACE + "ai_detector_mgr/stop_classifier", Empty, queue_size=1)
    rospy.Subscriber('~set_threshold', Float32, self.setThresholdCb) 
    self.set_threshold_pub = rospy.Publisher(NEPI_BASE_NAMESPACE + "ai_detector_mgr/set_threshold", Float32, queue_size=1)
    '''

    # App Specific Subscribers
    
    enable_targeting_sub = rospy.Subscriber('~targeting_enabled', Bool, self.addAllClassesCb, queue_size = 10)
    add_all_sub = rospy.Subscriber('~add_all_target_classes', Empty, self.addAllClassesCb, queue_size = 10)
    remove_all_sub = rospy.Subscriber('~remove_all_target_classes', Empty, self.removeAllClassesCb, queue_size = 10)
    add_class_sub = rospy.Subscriber('~add_target_class', AiTargetingAddClass, self.addClassCb, queue_size = 10)
    remove_class_sub = rospy.Subscriber('~remove_target_class', String, self.removeClassCb, queue_size = 10)
    select_target_sub = rospy.Subscriber('~select_target', String, self.selectTargetCb, queue_size = 10)
    vert_fov_sub = rospy.Subscriber("~set_image_fov_vert", Float32, self.setVertFovCb, queue_size = 10)
    horz_fov_sub = rospy.Subscriber("~set_image_fov_horz", Float32, self.setHorzFovCb, queue_size = 10)
    box_reduction_sub = rospy.Subscriber("~set_box_reduction_percent", Float32, self.setBoxReductionCb, queue_size = 10)
    default_target_depth_sub = rospy.Subscriber("~set_default_target_detpth", Float32, self.setDefaultTargetDepthCb, queue_size = 10)
    target_min_points_sub = rospy.Subscriber("~set_target_min_points", Int32, self.setTargetMinPointsCb, queue_size = 10)
    age_filter_sub = rospy.Subscriber("~set_age_filter", Float32, self.setAgeFilterCb, queue_size = 10)

    # Start an AI manager status monitoring thread
    AI_MGR_STATUS_SERVICE_NAME = NEPI_BASE_NAMESPACE + "ai_detector_mgr/img_classifier_status_query"
    self.get_ai_mgr_status_service = rospy.ServiceProxy(AI_MGR_STATUS_SERVICE_NAME, ImageClassifierStatusQuery)
    time.sleep(1)
    rospy.Timer(rospy.Duration(1), self.getAiMgrStatusCb)

    # Start AI Manager Subscribers
    FOUND_OBJECT_TOPIC = NEPI_BASE_NAMESPACE + "ai_detector_mgr/found_object"
    rospy.Subscriber(FOUND_OBJECT_TOPIC, ObjectCount, self.found_object_callback, queue_size = 1)
    BOUNDING_BOXES_TOPIC = NEPI_BASE_NAMESPACE + "ai_detector_mgr/bounding_boxes"
    rospy.Subscriber(BOUNDING_BOXES_TOPIC, BoundingBoxes, self.object_detected_callback, queue_size = 1)
    DETECTION_IMAGE_TOPIC = NEPI_BASE_NAMESPACE + "ai_detector_mgr/detection_image"
    rospy.Subscriber(DETECTION_IMAGE_TOPIC, Image, self.detectionImageCb, queue_size = 1)
    #self.detection_image_pub = rospy.Publisher("~detection_image",Image,queue_size=1)
    # Setup Node Publishers
    self.status_pub = rospy.Publisher("~status", AiTargetingStatus, queue_size=1, latch=True)
    self.bounding_boxes_2d_pub = rospy.Publisher("~bounding_boxes_2d", BoundingBoxes, queue_size=1)
    self.bounding_boxes_3d_pub = rospy.Publisher("~bounding_boxes_3d", BoundingBoxes3D, queue_size=1)
    self.target_localizations_pub = rospy.Publisher("~targeteting_data", TargetLocalizations, queue_size=1)
    self.image_pub = rospy.Publisher("~targeting_image",Image,queue_size=1)
    self.pointcloud_pub = rospy.Publisher("~target_pointcloud",PointCloud2,queue_size=1)
    time.sleep(1)
    ## Initiation Complete
    rospy.loginfo("AI_APP: Initialization Complete")
    self.publish_status()


  #######################
  ### AI Magnager Callbacks

  def getAiMgrStatusCb(self,timer):
    update_status = False
    try:
      ai_mgr_status_response = self.get_ai_mgr_status_service()
    except Exception as e:
      rospy.loginfo("AI_APP: Failed to call AI MGR STATUS service " + str(e))
      return
    #status_str = str(ai_mgr_status_response)
    #rospy.logwarn("AI_APP: got ai manager status: " + status_str)
    self.current_classifier = ai_mgr_status_response.selected_classifier
    self.current_classifier_state = ai_mgr_status_response.classifier_state
    classes_string = ai_mgr_status_response.selected_classifier_classes
    self.current_classifier_classes = nepi_ros.parse_string_list_msg_data(classes_string)
    #classes_str = str(self.current_classifier_classes)
    #rospy.logwarn("AI_APP: got ai manager status: " + classes_str)
    self.current_image_topic = ai_mgr_status_response.selected_img_topic

    selected_classes_dict = rospy.get_param('~selected_classes_dict', self.init_selected_classes_dict)
    last_classifier = rospy.get_param('~last_classiier', self.init_last_classifier)
    if last_classifier != self.current_classifier and self.current_classifier != "None":
      selected_classes_dict = dict() # Reset classes to all on new classifier
      for target_class in self.current_classifier_classes:
        selected_classes_dict[target_class] = {'depth': self.FACTORY_TARGET_DEPTH_METERS}
      update_status = True
    rospy.set_param('~selected_classes_dict', selected_classes_dict)
    rospy.set_param('~last_classiier', self.current_classifier)

    # Check for targeting enabled
    targeting_enabled = rospy.get_param('~targeting_enabled', self.init_enable_targeting)
    if targeting_enabled == True:
      if self.last_image_topic != self.current_image_topic and self.current_image_topic != "None":
        image_topic = nepi_ros.find_topic(self.current_image_topic)
        #rospy.logwarn(depth_map_topic)
        if image_topic != "":
          if self.image_sub != None:
            self.image_sub.Unregister()
            time.sleep(1)
          self.image_sub = rospy.Subscriber(image_topic, Image, self.targetingImageCb, queue_size = 10)
          self.current_image_topic = image_topic
        self.last_image_topic = self.current_image_topic

        depth_map_topic = self.current_image_topic.rsplit('/',1)[0] + "/depth_map"
        depth_map_topic = nepi_ros.find_topic(depth_map_topic)
        self.depth_map_topic = depth_map_topic
        #rospy.logwarn(depth_map_topic)
        if depth_map_topic != "":
          if self.depth_map_sub != None:
            self.depth_map_sub.Unregister()
            time.sleep(1)
          self.depth_map_sub = rospy.Subscriber(depth_map_topic, Image, self.depthMapCb, queue_size = 10)
          update_status = True

        pointcloud_topic = self.current_image_topic.rsplit('/',1)[0] + "/pointcloud"
        pointcloud_topic = nepi_ros.find_topic(pointcloud_topic)
        self.pointcloud_topic = pointcloud_topic
        if pointcloud_topic != "":
          if self.pointcloud_sub != None:
            self.pointcloud_sub.Unregister()
            time.sleep(1)
          self.pointcloud_sub = rospy.Subscriber(pointcloud_topic, PointCloud2, self.pointcloudCb, queue_size = 10)
          update_status = True

    else:  # Turn off targeting subscribers and reset last image topic
      if self.image_sub != None:
        self.image_sub.Unregister()
        self.current_image_topic = "None"
        self.current_image_header = Header()
      if self.depth_map_sub != None:
        self.depth_map_sub.Unregister()
        self.depth_map_topic = "None"
        self.depth_map_header = Header()
      if self.pointcloud_sub != None:
        self.pointcloud_sub.Unregister()
        self.pointcloud_topic = ""

      self.last_image_topic = "None"
      time.sleep(1)
    if update_status == True:
      self.publish_status()
    if self.pointcloud_topic == "":
        o3d_pc = nepi_pc.create_empty_o3dpc()
        ros_pc = nepi_pc.o3dpc_to_rospc(o3d_pc,rospy.Time.now())
        self.pointcloud_pub.publish(ros_pc)
    


  ### Monitor Output of AI model to clear detection status
  def found_object_callback(self,found_obj_msg):
    # Must reset target lists if no targets are detected
    if found_obj_msg.count == 0:
      #print("No objects detected")
      self.detect_boxes_msg = None
      self.bounding_box_3d_list = None


  ### If object(s) detected, save bounding box info to global
  def object_detected_callback(self,bounding_boxes_msg):
    self.detect_boxes_msg=bounding_boxes_msg

  def detectionImageCb(self,img_msg):
    ros_timestamp = img_msg.header.stamp
    # Convert ROS image to OpenCV for editing
    cv2_bridge = CvBridge()
    cv2_img = cv2_bridge.imgmsg_to_cv2(img_msg, "bgr8")
    self.save_img2file('detection_image',cv2_img,ros_timestamp)    
    #self.detection_image_pub.publish(img_msg)

  def targetingImageCb(self,img_msg):  
    self.current_image_header = img_msg.header
    ros_timestamp = img_msg.header.stamp
    now_timestamp = rospy.Time.now()
    # Convert ROS image to OpenCV for editing
    self.img_height = img_msg.height
    self.img_width = img_msg.width
    cv2_bridge = CvBridge()
    cv2_img = cv2_bridge.imgmsg_to_cv2(img_msg, "bgr8")
    self.save_img2file('detection_image',cv2_img,ros_timestamp)

    selected_classes_dict = rospy.get_param('~selected_classes_dict', self.init_selected_classes_dict)
    # Iterate over all of the objects and calculate range and bearing data
    image_fov_vert = rospy.get_param('~image_fov_vert',  self.init_image_fov_vert)
    image_fov_horz = rospy.get_param('~image_fov_horz', self.init_image_fov_horz)
    target_reduction_percent = rospy.get_param('~target_box_reduction',  self.init_target_box_reduction)
    target_min_points = rospy.get_param('~target_min_points',  self.init_target_min_points)
    detected_uids = []
    bbs2d = []
    tls = []
    bbs3d = []
    detect_boxes_msg = copy.deepcopy(self.detect_boxes_msg)
    target_boxes_msg = copy.deepcopy(detect_boxes_msg)
    if detect_boxes_msg is not None:
      target_boxes_msg.bounding_boxes = [] 
      detect_header = detect_boxes_msg.header
      ros_timestamp = detect_header.stamp
      detect_img_header = detect_boxes_msg.image_header
      detect_img_topic = detect_boxes_msg.image_topic
      for i, box in enumerate(detect_boxes_msg.bounding_boxes):
        if box.Class in selected_classes_dict.keys():
          self.seq += 1 
          target_depth_m = selected_classes_dict[box.Class]['depth']
          # Get target label
          target_label=box.Class
          # reduce target box based on user settings
          box_reduction_y_pix=int(float((box.ymax - box.ymin))*float(target_reduction_percent )/100/2)
          box_reduction_x_pix=int(float((box.xmax - box.xmin))*float(target_reduction_percent )/100/2)
          ymin_adj=int(box.ymin + box_reduction_y_pix)
          ymax_adj=int(box.ymax - box_reduction_y_pix)
          xmin_adj=box.xmin + box_reduction_x_pix
          xmax_adj=box.xmax - box_reduction_x_pix
          # Calculate target range
          np_depth_array_m = copy.deepcopy(self.np_depth_array_m)
          if np_depth_array_m is not None and self.depth_map_topic != "None":
            depth_map_header = copy.deepcopy(self.depth_map_header)
            # Get target range from cropped and filtered depth data
            depth_box_adj= np_depth_array_m[ymin_adj:ymax_adj,xmin_adj:xmax_adj]
            depth_mean_val=np.mean(depth_box_adj)
            depth_array=depth_box_adj.flatten()
            min_filter=depth_mean_val-target_depth_m/2
            max_filter=depth_mean_val+target_depth_m/2
            depth_array=depth_array[depth_array > min_filter]
            depth_array=depth_array[depth_array < max_filter]
            depth_len=len(depth_array)
            if depth_len > target_min_points:
              target_range_m=np.mean(depth_box_adj)
            else:
              target_range_m=float(-999) # NEPI standard unset value
          else:
            target_range_m=float(-999)  # NEPI standard unset value
          # Calculate target bearings
          object_loc_y_pix = float(box.ymin + ((box.ymax - box.ymin))  / 2) 
          object_loc_x_pix = float(box.xmin + ((box.xmax - box.xmin))  / 2)
          object_loc_y_ratio_from_center = float(object_loc_y_pix - self.img_height/2) / float(self.img_height/2)
          object_loc_x_ratio_from_center = float(object_loc_x_pix - self.img_width/2) / float(self.img_width/2)
          target_vert_angle_deg = (object_loc_y_ratio_from_center * float(image_fov_vert/2))
          target_horz_angle_deg = - (object_loc_x_ratio_from_center * float(image_fov_horz/2))
          ### Print the range and bearings for each detected object
    ##      print(target_label)
    ##      print(str(depth_box_adj.shape) + " detection box size")
    ##      print(str(depth_len) + " valid depth readings")
    ##      print("%.2f" % target_range_m + "m : " + "%.2f" % target_horz_angle_deg + "d : " + "%.2f" % target_vert_angle_deg + "d : ")
    ##      print("")

          #### NEED TO Calculate Unique IDs
          uid_suffix = 0
          target_uid = box.Class + "_" + str(box.id) + "_"  + str(uid_suffix)# Need to add unque id tracking
          while target_uid in detected_uids:
            uid_suffix += 1
            target_uid = box.Class + "_" + str(box.id) + "_"  + str(uid_suffix)

          if self.selected_target == "All" or self.selected_target == target_uid:
            # Updated Bounding Box 2d
            bounding_box_msg = BoundingBox()
            bounding_box_msg.Class = box.Class
            bounding_box_msg.id = box.id
            bounding_box_msg.uid = target_uid
            bounding_box_msg.probability = box.probability
            bbs2d.append(bounding_box_msg)

            # Create Targeting_Data
            target_data_msg=TargetLocalization()
            target_data_msg.Class = box.Class
            target_data_msg.id = box.id 
            target_data_msg.uid = target_uid
            target_data_msg.range_m=target_range_m
            target_data_msg.azimuth_deg=target_horz_angle_deg
            target_data_msg.elevation_deg=target_vert_angle_deg
            tls.append(target_data_msg)

            # Create Bounding Box 3d
            bounding_box_3d_msg = None
            if target_range_m != -999:
              # Calculate Bounding Box 3D
              bounding_box_3d_msg = BoundingBox3D()
              bounding_box_3d_msg.Class = box.Class
              bounding_box_3d_msg.id = box.id 
              bounding_box_3d_msg.uid = target_uid
              bounding_box_3d_msg.probability = box.probability
              # Calculate the Box Center
              # Ref www.stackoverflow.com/questions/30619901/calculate-3d-point-coordinates-using-horizontal-and-vertical-angles-and-slope-di
              bbc = Vector3()
              theta_deg = (target_vert_angle_deg + 90)  #  Vert Angle 0 - 180 from top
              theta_rad = theta_deg * math.pi/180 #  Vert Angle 0 - PI from top
              phi_deg =  (target_horz_angle_deg) # Horz Angle 0 - 360 from X axis counter clockwise
              phi_rad = phi_deg * math.pi/180 # Horz Angle 0 - 2 PI from from X axis counter clockwise
              rospy.logwarn([theta_deg,phi_deg])
              bbc.x = target_range_m * math.sin(theta_rad) * math.cos(phi_rad)
              bbc.y = target_range_m * math.cos(theta_rad) * math.sin(phi_rad)
              bbc.z = target_range_m * math.cos(theta_rad)
              bounding_box_3d_msg.box_center_m.x = bbc.x
              bounding_box_3d_msg.box_center_m.y = bbc.y
              bounding_box_3d_msg.box_center_m.z = bbc.z
              # Calculate the Box Extent
              bbe = Vector3()  
              mpp_vert_at_range = 2 * target_range_m * math.sin(image_fov_vert/2 * math.pi/180) / self.img_height
              mpp_horz_at_range = 2* target_range_m * math.sin(image_fov_horz/2 * math.pi/180) / self.img_width
              mpp_at_range = (mpp_vert_at_range + mpp_horz_at_range) / 2  #  ToDo: More accurate calc
              bbe.x = selected_classes_dict[box.Class]['depth']
              bbe.y = mpp_at_range * (box.xmax-box.xmin)
              bbe.z = mpp_at_range * (box.ymax-box.ymin)
              bounding_box_3d_msg.box_extent_xyz_m.x = bbe.x
              bounding_box_3d_msg.box_extent_xyz_m.y = bbe.y
              bounding_box_3d_msg.box_extent_xyz_m.z = bbe.z
              # Target Rotation Unknown
              bbr = Vector3()
              bbr.x = 0
              bbr.y = 0
              bbr.z = 0
              bounding_box_3d_msg.box_rotation_rpy_deg.x = bbr.x
              bounding_box_3d_msg.box_rotation_rpy_deg.y = bbr.y
              bounding_box_3d_msg.box_rotation_rpy_deg.z = bbr.z
              # To Do Add Bounding Box 3D Data
              bbs3d.append(bounding_box_3d_msg)
        
          ###### Apply Image Overlays and Publish Targeting_Image ROS Message
          # Overlay adjusted detection boxes on image 
          start_point = (xmin_adj, ymin_adj)
          end_point = (xmax_adj, ymax_adj)
          cv2.rectangle(cv2_img, start_point, end_point, color=(255,0,0), thickness=2)
          # Overlay text data on OpenCV image
          font                   = cv2.FONT_HERSHEY_DUPLEX
          fontScale, thickness  = self.optimal_font_dims(cv2_img,font_scale = 1.5e-3, thickness_scale = 2e-3)
          fontColor              = (0, 255, 0)
          lineType               = 1
        # Overlay Label
          text2overlay=target_uid
          bottomLeftCornerOfText = (int(object_loc_x_pix),int(object_loc_y_pix))
          cv2.putText(cv2_img,text2overlay, 
            bottomLeftCornerOfText, 
            font, 
            fontScale,
            fontColor,
            thickness,
            lineType)
          # Overlay Data
          if target_range_m == -999:
            t_range_m = np.nan
          else:
            t_range_m = target_range_m
          text2overlay="%.1f" % t_range_m + "m," + "%.f" % target_horz_angle_deg + "d," + "%.f" % target_vert_angle_deg + "d"
          bottomLeftCornerOfText = (int(object_loc_x_pix),int(object_loc_y_pix)+15)
          cv2.putText(cv2_img,text2overlay, 
            bottomLeftCornerOfText, 
            font, 
            fontScale,
            fontColor,
            thickness,
            lineType)


          # Update Current Targets List
          if bounding_box_3d_msg is not None:
            center_m = [bbc.x,bbc.y,bbc.z]
          else:
            center_m = [-999,-999,-999]
          self.current_targets_dict[target_uid] = {'class': box.Class, 
                          'last_center_px': [box.xmax-box.xmin,box.ymax-box.ymin],
                          'last_velocity_pxps': [0,0],
                          'last_center_m': center_m,
                          'last_velocity_mps': [0,0,0],
                          'last_detection_timestamp': ros_timestamp                              
                          }


    # Purge Current Targets List based on Age
    purge_list = []
    age_filter_sec = rospy.get_param('~target_age_filter', self.init_target_age_filter)
    for target in self.current_targets_dict.keys():
      last_timestamp = self.current_targets_dict[target]['last_detection_timestamp']
      #rospy.logwarn(target)
      #rospy.logwarn(now_timestamp)
      #rospy.logwarn(last_timestamp)
      age = now_timestamp - last_timestamp
      if age > rospy.Duration(age_filter_sec):
        purge_list.append(target)
    for target in purge_list: 
        self.lost_targets_dict[target] = self.current_targets_dict[target]
        del self.current_targets_dict[target]


    # Publish and Save 2D Bounding Boxes
    if len(bbs2d) > 0:
      target_boxes_msg.bounding_boxes = bbs2d
      if not rospy.is_shutdown():
        self.bounding_boxes_2d_pub.publish(detect_boxes_msg)
      # Save Data if it is time.
      self.save_data2file("bounding_boxes_2d",detect_boxes_msg,ros_timestamp)

    # Publish and Save Target Localizations
    if len(tls) > 0:
      target_locs_msg = TargetLocalizations()
      target_locs_msg.header = detect_header
      target_locs_msg.image_topic = self.current_image_topic
      target_locs_msg.image_header = self.current_image_header
      target_locs_msg.depth_topic = self.depth_map_topic
      target_locs_msg.depth_header = self.depth_map_header
      target_locs_msg.target_localizations = tls
      if not rospy.is_shutdown():
        self.target_localizations_pub.publish(target_locs_msg)
      # Save Data if Time
      self.save_data2file('targeting_data',target_locs_msg,ros_timestamp)

    # Publish and Save 3D Bounding Boxes
    self.bounding_box_3d_list = bbs3d
    rospy.logwarn("")
    rospy.logwarn(bbs3d)
    if len(bbs3d) > 0:
      bounding_boxes_3d_msg = BoundingBoxes3D()
      bounding_boxes_3d_msg.header = detect_header
      bounding_boxes_3d_msg.image_topic = self.current_image_topic
      bounding_boxes_3d_msg.image_header = self.current_image_header
      bounding_boxes_3d_msg.depth_map_header = self.depth_map_header
      bounding_boxes_3d_msg.depth_map_topic = self.depth_map_topic
      bounding_boxes_3d_msg.bounding_boxes_3d = bbs3d
      if not rospy.is_shutdown():
        self.bounding_boxes_3d_pub.publish(bounding_boxes_3d_msg)
      # Save Data if Time
      self.save_data2file('bounding_boxes_3d',bounding_boxes_3d_msg,ros_timestamp)


    #Convert OpenCV image to ROS image
    img_out_msg = cv2_bridge.cv2_to_imgmsg(cv2_img,"bgr8")#desired_encoding='passthrough')
    # Publish new image to ros
    if not rospy.is_shutdown():
      self.image_pub.publish(img_out_msg)

    # Save Data if Time
    self.save_img2file('targeting_image',cv2_img,ros_timestamp)


  def optimal_font_dims(self, img, font_scale = 2e-3, thickness_scale = 5e-3):
    h, w, _ = img.shape
    font_scale = min(w, h) * font_scale
    thickness = math.ceil(min(w, h) * thickness_scale)
    return font_scale, thickness

  def depthMapCb(self,depth_map_msg):
    self.current_detph_map_header = depth_map_msg.header
    # Zed depth data is floats in m, but passed as 4 bytes each that must be converted to floats
    # Use cv2_bridge() to convert the ROS image to OpenCV format
    #Convert the depth 4xbyte data to global float meter array
    self.depth_map_header = depth_map_msg.header
    cv2_bridge = CvBridge()
    cv2_depth_image = cv2_bridge.imgmsg_to_cv2(depth_map_msg, desired_encoding="passthrough")
    self.np_depth_array_m = (np.array(cv2_depth_image, dtype=np.float32)) # replace nan values
    self.np_depth_array_m[np.isnan(self.np_depth_array_m)] = 0 # zero pixels with no value
    self.np_depth_array_m[np.isinf(self.np_depth_array_m)] = 0 # zero pixels with inf value

  def pointcloudCb(self,pointcloud2_msg):
      bounding_box_3d_list = copy.deepcopy(self.bounding_box_3d_list)
      bb3d = None
      if bounding_box_3d_list is not None:
        for i in range(len(bounding_box_3d_list)):
          if self.selected_target != "All" and bounding_box_3d_list[i].uid == self.selected_target:
            bb3d = bounding_box_3d_list[i]
        if bb3d != None:
          o3d_pc = rospc_to_o3dpc(pointcloud2_msg, remove_nans=True)
          bounding_box_center = [bb3d.box_center_m.x,bb3d.box_center_m.y,bb3d.box_center_m.z]
          bounding_box_extent = [bb3d.box_extent_m.x,bb3d.box_extent_m.y,bb3d.box_extent_m.z]
          bounding_box_rotation = [bb3d.box_rotation_rpy_deg.x,bb3d.box_rotation_rpy_deg.y,bb3d.box_rotation_rpy_deg.z]
          o3d_pc_clipped = nepi_pc.clip_bounding_box(o3d_pc_clipped, bb3d)
          ros_pc = nepi_pc.o3dpc_to_rospc(o3d_pc_clipped)
          ros_pc.header = msg.header
          self.pointcloud_pub.publish(ros_pc)





        




  ###################
  ## Status Publisher
  def publish_status(self):
    status_msg = AiTargetingStatus()

    status_msg.targeting_enabled = rospy.get_param('~targeting_enabled', self.init_enable_targeting)

    status_msg.classifier_name = self.current_classifier
    status_msg.classifier_state = self.current_classifier_state

    status_msg.image_topic = self.current_image_topic
    status_msg.depth_map_topic = self.depth_map_topic
    status_msg.pointcloud_topic = self.pointcloud_topic

    status_msg.available_classes_list = str(self.current_classifier_classes)
    selected_classes_dict = rospy.get_param('~selected_classes_dict', self.init_selected_classes_dict)
    classes_list = []
    depth_list = []
    for key in selected_classes_dict.keys():
      classes_list.append(key)
      depth_list.append(selected_classes_dict[key]['depth'])
    status_msg.selected_classes_list = str(classes_list)
    status_msg.selected_classes_depth_list = str(depth_list)

    status_msg.selected_target = self.selected_target
    targets_list = ["All"]
    if self.selected_target != "All" and self.selected_target not in self.current_targets_dict.keys():
      targets_list.append(self.selected_target)
    for target in self.current_targets_dict.keys():
      targets_list.append(target) 
    status_msg.available_targets_list = str(targets_list)

    status_msg.image_fov_vert_degs = rospy.get_param('~image_fov_vert',  self.init_image_fov_vert)
    status_msg.image_fov_horz_degs = rospy.get_param('~image_fov_horz', self.init_image_fov_horz)

    status_msg.target_box_reduction_percent = rospy.get_param('~target_box_reduction',  self.init_target_box_reduction)
    status_msg.default_target_depth_m = rospy.get_param('~default_target_depth',  self.init_default_target_depth)
    status_msg.target_min_points = rospy.get_param('~target_min_points', self.init_target_min_points)
    status_msg.target_age_filter = rospy.get_param('~target_age_filter', self.init_target_age_filter)

    self.status_pub.publish(status_msg)

 
      
    
  #######################
  # Data Saving Funcitons
 



  def save_data2file(self,data_product,data,ros_timestamp):
      if self.save_data_if is not None:
          saving_is_enabled = self.save_data_if.data_product_saving_enabled(data_product)
          snapshot_enabled = self.save_data_if.data_product_snapshot_enabled(data_product)
          # Save data if enabled
          if saving_is_enabled or snapshot_enabled:
              if data is not None:
                  if (self.save_data_if.data_product_should_save(data_product) or snapshot_enabled):
                      full_path_filename = self.save_data_if.get_full_path_filename(nepi_ros.get_datetime_str_from_stamp(ros_timestamp), 
                                                                                              "ai_detector_mgr-" + data_product, 'txt')
                      if os.path.isfile(full_path_filename) is False:
                          with open(full_path_filename, 'w') as f:
                              f.write('input_image: ' + self.current_image_topic + '\n')
                              f.write(str(msg))

  def save_img2file(self,data_product,cv2_img,ros_timestamp):
      if self.save_data_if is not None:
          saving_is_enabled = self.save_data_if.data_product_saving_enabled(data_product)
          snapshot_enabled = self.save_data_if.data_product_snapshot_enabled(data_product)
          # Save data if enabled
          if saving_is_enabled or snapshot_enabled:
              if cv2_img is not None:
                  if (self.save_data_if.data_product_should_save(data_product) or snapshot_enabled):
                      full_path_filename = self.save_data_if.get_full_path_filename(nepi_ros.get_datetime_str_from_stamp(ros_timestamp), 
                                                                                              "ai_detector_mgr-" + data_product, 'png')
                      if os.path.isfile(full_path_filename) is False:
                          cv2.imwrite(full_path_filename, cv2_img)
                          self.save_data_if.data_product_snapshot_reset(data_product)

  def save_pc2file(self,data_product,o3d_pc,ros_timestamp):
      if self.save_data_if is not None:
          saving_is_enabled = self.save_data_if.data_product_saving_enabled(data_product)
          snapshot_enabled = self.save_data_if.data_product_snapshot_enabled(data_product)
          # Save data if enabled
          if saving_is_enabled or snapshot_enabled:
              if o3d_pc is not None:
                  if (self.save_data_if.data_product_should_save(data_product) or snapshot_enabled):
                      full_path_filename = self.save_data_if.get_full_path_filename(nepi_ros.get_datetime_str_from_stamp(ros_timestamp), 
                                                                                              "ai_detector_mgr-" + data_product, 'pcd')
                      if os.path.isfile(full_path_filename) is False:
                          nepi_pc.save_pointcloud(o3d_pc,full_path_filename)
                          self.save_data_if.data_product_snapshot_reset(data_product)

                
    
  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    rospy.loginfo("AI_APP: Shutting down: Executing script cleanup actions")


#########################################
# Main
#########################################
if __name__ == '__main__':
  node_name = "ai_targeting_app"
  rospy.init_node(name=node_name)
  #Launch the node
  rospy.loginfo("AI_APP: Launching node named: " + node_name)
  node = NepiAiTargetingApp()
  #Set up node shutdown
  rospy.on_shutdown(node.cleanup_actions)
  # Spin forever (until object is detected)
  rospy.spin()





