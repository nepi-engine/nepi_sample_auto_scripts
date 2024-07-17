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
# 1. Converts ROS pointcloud2 to Open3D pointcloud
# 2. Blank area for custom code
# 3. Converts Open3D pointcloud back to ROS pointcloud2


import time
import sys
import rospy
import numpy as np
import open3d as o3d
import os

from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
from std_msgs.msg import UInt8, Empty, String, Bool
from nepi_ros_interfaces.msg import IDXStatus, RangeWindow

# For Testing 
from std_msgs.msg import Header
from sensor_msgs.msg import PointField
from sensor_msgs import point_cloud2

from nepi_edge_sdk_base import nepi_ros
from nepi_edge_sdk_base import nepi_pc 


#########################################
# USER SETTINGS - Edit as Necessary 
#########################################
POINTCLOUD_INPUT_TOPIC_NAME = "pointcloud"

# These will be overwritten by IDX controls if enabled
Range_Clip_Enable = False # True
Range_Clip_Min_Range_M = 0
Range_Clip_Max_Range_M = 1000

# Create Bounding Box Enable
Create_Bounding_Box_Enable = False # True
Create_Bounding_Box_Center = [0, 0, 0]
Create_Bounding_Box_Extent_M = [1, 1, 1]
Create_Bounding_Box_Rotation_Deg = [0, 0, 0]
Create_Bounding_Box_Has_Color = True
Create_Bounding_Box_Color = [1, 0, 0]

# Clip Bounding Box Enable
Clip_Bounding_Box_Enable = False #  True
Clip_Bounding_Box_Center = [0,0,0]
Clip_Bounding_Box_Extent_M = [2.9, 2.9, 2.9]
Clip_Bounding_Box_Rotation_Deg = [0, 0, 0]

# Voxel Down Sample Enable
Voxel_Down_Sample_Enable = False # True
Voxel_Size_M = 0.03

# Uniform Down Sample Enable
Uniform_Down_Sample_Enable = False # True
Uniform_Down_Sample_Every_K_Points = 5 

# Statistical Outlier Removal Enable
Statistical_Outlier_Removal_Enable = False # True
Statistical_Outlier_Removal_Nb_Neighbors = 20
Statistical_Outlier_Removal_Std_Ratio = 2.0

# Radius Outlier Removal Enable
Radius_Outlier_Removal_Enable = False # True
Radius_Outlier_Removal_Nb_Points = 16
Radius_Outlier_Removal_Search_Radius_M = 0.05

# Rotate PCD Enable
Rotate_Pointcloud_Enable = False # True
Rotate_Pointcloud_Angles_Deg = [0, 180, 0]

# Translate PCD Enable
Translate_Pointcloud_Enable = False # True
Translate_Pointcloud_Translation_Vector_M = [5, 0, 0] 

# Save Pointcloud   
Save_Enable = False
Save_Path = '/mnt/nepi_storage/data/'
Save_Prefix = 'Pointcloud_Processed_'
Save_Source_Name = 'pointcloud_mod'
Save_Timestamp = True
Save_Rate_Max_HZ = 0.1


#########################################
# Node Class
#########################################

class pointcloud_processing_process(object):
  range_clip_enable = Range_Clip_Enable
  range_clip_min_range_m = Range_Clip_Min_Range_M
  range_clip_max_range_m = Range_Clip_Max_Range_M
  
  create_bounding_box_enable = Create_Bounding_Box_Enable
  create_bounding_box_center = Create_Bounding_Box_Center
  create_bounding_box_extent_m = Create_Bounding_Box_Extent_M
  create_bounding_box_rotation_deg = Create_Bounding_Box_Rotation_Deg
  create_bounding_box_color = Create_Bounding_Box_Color
  create_bounding_box_has_color = Create_Bounding_Box_Has_Color
  
  clip_bounding_box_enable = Clip_Bounding_Box_Enable
  clip_bounding_box_center = Clip_Bounding_Box_Center
  clip_bounding_box_extent_m = Clip_Bounding_Box_Extent_M
  clip_bounding_box_rotation_deg = Clip_Bounding_Box_Rotation_Deg
  
  voxel_down_sample_enable = Voxel_Down_Sample_Enable
  voxel_size_m = Voxel_Size_M
  
  uniform_down_sample_enable = Uniform_Down_Sample_Enable
  uniform_down_sample_every_k_points = Uniform_Down_Sample_Every_K_Points
  
  statistical_outlier_removal_enable = Statistical_Outlier_Removal_Enable
  statistical_outlier_removal_nb_neighbors = Statistical_Outlier_Removal_Nb_Neighbors
  statistical_outlier_removal_std_ratio = Statistical_Outlier_Removal_Std_Ratio
  
  radius_outlier_removal_enable = Radius_Outlier_Removal_Enable
  radius_outlier_removal_nb_points = Radius_Outlier_Removal_Nb_Points
  radius_outlier_removal_search_radius_m = Radius_Outlier_Removal_Search_Radius_M
  
  rotate_pointcloud_enable = Rotate_Pointcloud_Enable
  rotate_pointcloud_angles_deg = Rotate_Pointcloud_Angles_Deg
  
  translate_pointcloud_enable = Translate_Pointcloud_Enable
  translate_pointcloud_translation_vector_m = Translate_Pointcloud_Translation_Vector_M
  
  save_enable = Save_Enable
  save_path = Save_Path
  save_prefix = Save_Prefix
  save_source_name =  Save_Source_Name
  save_timestamp = Save_Timestamp
  save_rate_max_hz = Save_Rate_Max_HZ
  save_delay_s = float(1)/save_rate_max_hz
  last_save_time = time.time()

  
  #######################
  ### Node Initialization
  def __init__(self):
    rospy.loginfo("Starting Initialization Processes")
    ## Initialize Class Variables
    ## Define Class Namespaces
    # ROS namespace setup
    NEPI_BASE_NAMESPACE = nepi_ros.get_base_namespace()
    # Wait for topic
    rospy.loginfo("Waiting for topic: " + POINTCLOUD_INPUT_TOPIC_NAME)
    pointcloud_input_topic = nepi_ros.wait_for_topic(POINTCLOUD_INPUT_TOPIC_NAME)
    rospy.loginfo("Found topic: " + pointcloud_input_topic)
    pointcloud_output_topic = NEPI_BASE_NAMESPACE + "pointcloud_mod"
    if pointcloud_input_topic.find("idx") != -1:
      # Start IDX range subscribers
      idx_status_topic = pointcloud_input_topic.replace("pointcloud","status")
      rospy.loginfo("Subscribing to idx status topic: " + idx_status_topic)
      rospy.Subscriber(idx_status_topic, IDXStatus, self.update_range_window_callback)
      sensor_name = idx_status_topic.split('/idx/')[0].split('/')[-1]
      self.save_source_name = (sensor_name + self.save_source_name)
    ## Define Class Services Calls
    ## Create Class Sevices    
    ## Create Class Publishers
    self.pointcloud_mod_pub = rospy.Publisher(pointcloud_output_topic, PointCloud2, queue_size=10)
    ## Start Class Subscribers
    # Start pointcloud process and pubslisher
    rospy.Subscriber(pointcloud_input_topic, PointCloud2, self.pointcloud_custom_callback)
    ## Start Node Processes
    ## Initiation Complete
    rospy.loginfo("Initialization Complete")

  #######################
  ### Node Methods


### Callback for IDX status message if present
  def update_range_window_callback(self,status_msg):
    total_range_m = status_msg.max_range_m - status_msg.min_range_m
    self.range_clip_min_range_m = status_msg.min_range_m  +  total_range_m * status_msg.range_window.start_range
    self.range_clip_max_range_m = status_msg.min_range_m  +  total_range_m * status_msg.range_window.stop_range
    rospy.loginfo("Updated range window meters: " + '%.2f' % self.range_clip_min_range_m + " : " + '%.2f' % self.range_clip_max_range_m)
     


  ### Callback to get,process, and publish modified pointcloud and image
  def pointcloud_custom_callback(self,ros_pc_in_msg):
    print("")
    print(ros_pc_in_msg.header)
    ros_pc_in_frame_id = ros_pc_in_msg.header.frame_id
    print(ros_pc_in_frame_id)
    ros_pc_in_stamp = ros_pc_in_msg.header.stamp
    print(ros_pc_in_stamp.secs)
    print(ros_pc_in_stamp.nsecs)
    time_start = time.time()
   
    ############################################################    
    #Convert pointcloud from ros to open3D
    o3d_pc = nepi_pc.rospc_to_o3dpc(ros_pc_in_msg, remove_nans=True)

    #*************
    print('')
    print("*************")
    print("ROS pc 2 O3D Time")
    process_time = time.time() - time_start
    print("%.2f" % process_time)
    time_start = time.time()
    print("*************")
    print('')
    #*************
   
    ###########################################################
    ### ADD POINTCLOUD PROCESS CODE HERE
    ###########################################################

    ############################################################ 
    ## Apply Range Clip Process to Input Point Cloud Here
    if self.range_clip_enable:
      rospy.loginfo("Clipping to range window meters: " + '%.2f' % self.range_clip_min_range_m + " : " + '%.2f' % self.range_clip_max_range_m)
      o3d_pc = nepi_pc.range_clip(o3d_pc, self.range_clip_min_range_m, self.range_clip_max_range_m)
      #*************
      print("O3D Range Clip Time:")
      process_time = time.time() - time_start
      print("%.2f" % process_time)
      time_start = time.time()
      #*************
    
    if self.create_bounding_box_enable:
      bounding_box = nepi_pc.create_bounding_box(self.create_bounding_box_center, self.create_bounding_box_extent_m, self.create_bounding_box_rotation_deg, 	self.create_bounding_box_color)
      #*************
      print("O3D Bounding Box Time:")
      process_time = time.time() - time_start
      print("%.2f" % process_time)
      time_start = time.time()
      #*************
    
    if self.clip_bounding_box_enable:
      o3d_pc = nepi_pc.clip_bounding_box(o3d_pc, self.clip_bounding_box_center, self.clip_bounding_box_extent_m, self.clip_bounding_box_rotation_deg)
      #*************
      print("O3D Clip Bounding Box Time:")
      process_time = time.time() - time_start
      print("%.2f" % process_time)
      time_start = time.time()
      #*************
        
    if self.voxel_down_sample_enable:
      o3d_pc = nepi_pc.voxel_down_sampling(o3d_pc, self.voxel_size_m)
      #*************
      print("O3D Voxel Downsample Time:")
      process_time = time.time() - time_start
      print("%.2f" % process_time)
      time_start = time.time()
      #*************

    if self.uniform_down_sample_enable:
      o3d_pc = nepi_pc.uniform_down_sampling(o3d_pc, self.uniform_down_sample_every_k_points)
      #*************
      print("O3D Uniform Downsample Time:")
      process_time = time.time() - time_start
      print("%.2f" % process_time)
      time_start = time.time()
      #*************

    if self.statistical_outlier_removal_enable:
      [o3d_pc, ind] = nepi_pc.statistical_outlier_removal(o3d_pc, self.statistical_outlier_removal_nb_neighbors, self.statistical_outlier_removal_std_ratio)
      #*************
      print("O3D Statistical Outlier Removal Time:")
      process_time = time.time() - time_start
      print("%.2f" % process_time)
      time_start = time.time()
      #*************

    if self.radius_outlier_removal_enable:
      [o3d_pc, ind] = nepi_pc.radius_outlier_removal(o3d_pc, self.radius_outlier_removal_nb_points, self.radius_outlier_removal_search_radius_m)
      #*************
      print("O3D Radius Outlier Removal:")
      process_time = time.time() - time_start
      print("%.2f" % process_time)
      time_start = time.time()
      #*************

    if self.rotate_pointcloud_enable:
      o3d_pc = nepi_pc.rotate_pc(o3d_pc, self.rotate_pointcloud_angles_deg)
      #*************
      print("O3D Rotate Pointcloud Time:")
      process_time = time.time() - time_start
      print("%.2f" % process_time)
      time_start = time.time()
      #*************

    if self.translate_pointcloud_enable:
      o3d_pc = nepi_pc.translate_pc(o3d_pc, self.translate_pointcloud_translation_vector_m)
      #*************
      print("O3D Translate Pointcloud Time:")
      process_time = time.time() - time_start
      print("%.2f" % process_time)
      time_start = time.time()
      #*************
    
      
      
    ###########################################################
    ### END OF POINTCLOUD PROCESS CODE
    ###########################################################
      
    ############################################################    
    ##Convert pointcloud from open3D to ros
    #ros_pc_out_msg = pc_convert.convertCloudFromOpen3dToRos(o3d_pc, frame_id="odom")
    ros_pc_out_msg = nepi_pc.o3dpc_to_rospc(o3d_pc, stamp=ros_pc_in_stamp, frame_id=ros_pc_in_frame_id)
    if not rospy.is_shutdown():
      self.pointcloud_mod_pub.publish(ros_pc_out_msg)
      
    #*************
    print('')
    print("*************")
    print("O3D PCD to ROS Time:")
    process_time = time.time() - time_start
    print("%.2f" % process_time)
    time_start = time.time()
    print("*************")
    print('')
    #*************

    # Save Pointcloud
    ############################################################
    print((time.time()-self.last_save_time))
    print(self.save_delay_s)
    ## Save pointcloud and image to file, do this after all publishing so not to add delays to outputs    
    if self.save_enable and (time.time()-self.last_save_time) > self.save_delay_s :
      self.last_save_time = time.time()
      print(ros_pc_in_stamp)
      dt_string = nepi_ros.get_datetime_str_from_stamp(ros_pc_in_stamp)
      print(dt_string)
      pc_filename = (self.save_path + self.save_prefix + "_" + dt_string + "_" + self.save_source_name + ".pcd")
      print(pc_filename)
      nepi_pc.save_pc(o3d_pc, pc_filename)

      #*************
      print('')
      print("*************")
      print("O3D Save PCD Time:")
      process_time = time.time() - time_start
      print("%.2f" % process_time)
      time_start = time.time()
      print("*************")
      print('')
      #*************

   #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    rospy.loginfo("Shutting down: Executing script cleanup actions")


#########################################
# Main
#########################################
if __name__ == '__main__':
  current_filename = sys.argv[0].split('/')[-1]
  current_filename = current_filename.split('.')[0]
  rospy.loginfo(("Starting " + current_filename), disable_signals=True) # Disable signals so we can force a shutdown
  rospy.init_node(name=current_filename)
  #Launch the node
  node_name = current_filename.rpartition("_")[0]
  rospy.loginfo("Launching node named: " + node_name)
  node_class = eval(node_name)
  node = node_class()
  #Set up node shutdown
  rospy.on_shutdown(node.cleanup_actions)
  # Spin forever (until object is detected)
  rospy.spin()






