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

# Render Image from pointcloud
Render_Enable = True
Render_Img_Width = 1280
Render_Img_Height = 720
Render_Background = [0, 0, 0, 0] # background color rgba
Render_FOV = 90 # camera field of view in degrees
Render_Center = [3, 0, 0]  # look_at target
Render_Eye = [-2, -1, 0]  # camera position
Render_Up = [0, 0, 1]  # camera orientation


Resolution_Mode = 2 # Int 0-3

#########################################
# Node Class
#########################################

class pointcloud_render_process(object):
  render_enable = Render_Enable 
  render_img_width = Render_Img_Width 
  render_img_height = Render_Img_Height
  render_background = Render_Background
  render_fov = Render_FOV
  render_center = Render_Center
  render_eye = Render_Eye
  render_up = Render_Up

  resolution_mode = Resolution_Mode

  img_renderer = None
  img_renderer_mtl = None


  
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
    pointcloud_image_output_topic = NEPI_BASE_NAMESPACE + "pointcloud_image"
    if pointcloud_input_topic.find("idx") != -1:
      # Start IDX range subscribers
      idx_status_topic = pointcloud_input_topic.replace("pointcloud","status")
      rospy.loginfo("Subscribing to idx status topic: " + idx_status_topic)
      rospy.Subscriber(idx_status_topic, IDXStatus, self.update_range_window_callback)
      sensor_name = pointcloud_image_output_topic.split('/idx/')[0].split('/')[-1]
    ## Define Class Services Calls
    ## Create Class Sevices    
    ## Create Class Publishers
    self.pointcloud_image_pub = rospy.Publisher(pointcloud_image_output_topic, Image, queue_size=10)
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
    ros_pc_in_frame_id = ros_pc_in_msg.header.frame_id
    ros_pc_in_stamp = ros_pc_in_msg.header.stamp
    time_start = time.time()
   
    ############################################################    
    #Convert pointcloud from ros to open3D
    o3d_pc = nepi_pc.rospc_to_o3dpc(ros_pc_in_msg, remove_nans=True)
##
##    #*************
##    print('')
##    print("*************")
##    print("ROS pc 2 O3D Time")
##    process_time = time.time() - time_start
##    print("%.2f" % process_time)
##    time_start = time.time()
##    print("*************")
##    #*************
         
    ############################################################
    ## Render and publish pc image, do this after pc publish so not to add delays to pc output

    res_scaler = res_scale = 0.3 + 0.7 * self.resolution_mode
    self.render_img_width = int(res_scaler * Render_Img_Width)
    self.render_img_height = int(res_scaler * Render_Img_Height)
    if self.render_enable:
      if self.img_renderer != None and self.img_renderer_mtl != None:
        self.img_renderer = nepi_pc.remove_img_renderer_geometry(self.img_renderer)
        self.img_renderer = nepi_pc.add_img_renderer_geometry(o3d_pc,self.img_renderer,self.img_renderer_mtl)
        o3d_img = nepi_pc.render_img(self.img_renderer,render_center,render_eye,render_up)
      	ros_img_out_msg = nepi_pc.o3dimg_to_rosimg(o3d_img, stamp=ros_pc_in_stamp, frame_id=ros_pc_in_frame_id)
      	if not rospy.is_shutdown():
        	self.pointcloud_image_pub.publish(ros_img_out_msg)
        # You can view the enhanced_2D_pointcloud topic at 
        # //192.168.179.103:9091/ in a connected web browser
      else:
        self.img_renderer = nepi_pc.create_img_renderer(img_width=self.render_img_width,
                           fov=self.render_fov,img_height=self.render_img_height, 
                           background = self.render_background)
        self.img_renderer_mtl = nepi_pc.create_img_renderer_mtl()

##      #*************
##      print('')
##      print("*************")
##      print("O3D Render Image Time:")
##      process_time = time.time() - time_start
##      print("%.2f" % process_time)
##      time_start = time.time()
##      print("*************")
##      #*************

    

        
##      #*************
##      print('')
##      print("*************")
##      print("O3D Image to ROS Time:")
##      process_time = time.time() - time_start
##      print("%.2f" % process_time)
##      time_start = time.time()
##      print("*************")
##      #*************

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






