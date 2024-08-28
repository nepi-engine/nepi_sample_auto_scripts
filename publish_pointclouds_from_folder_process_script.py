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
# 1. Read in and publish pointclouds as ros pointcloud2 messages from folder at specified rate
# 2. Run until Stopped

import os
import time
import sys
import rospy
import numpy as np
import open3d as o3d

from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
from std_msgs.msg import UInt8, Empty, String, Bool, Float32, Int32

from std_msgs.msg import Header
from sensor_msgs.msg import PointField
from sensor_msgs import point_cloud2

from geometry_msgs.msg import Vector3, Transform
from nepi_ros_interfaces.msg import Frame3DTransform, Frame3DTransformUpdate

from nepi_edge_sdk_base import nepi_ros
from nepi_edge_sdk_base import nepi_pc 

#########################################
# USER SETTINGS - Edit as Necessary 
#########################################
Current_Path = (os.path.dirname(os.path.realpath(__file__)) )

### Open and publish image
Pointcloud_Path = Current_Path + "/sample_data/"

Publish_Image_Topic_Name = "test_pointcloud"
Publish_Image_Rate_Hz = 5

Zero_Transform = '0 0 0 0 0 0 0'

#########################################
# ROS NAMESPACE SETUP
#########################################

# ROS namespace setup
NEPI_BASE_NAMESPACE = nepi_ros.get_base_namespace()
TRANSFORM_PUB_NAMESPACE = NEPI_BASE_NAMESPACE + "pointcloud_app/add_transform"

#########################################
# Node Class
#########################################



class publish_pointclouds_from_file_process(object):
  seq_num = 0

  #######################
  ### Node Initialization
  def __init__(self):
    rospy.loginfo("Starting Initialization Processes")
    # Get list of image files in the folder
    print("Looking for pcd files in folder: " + Pointcloud_Path)
    if os.path.exists(Pointcloud_Path):
      [pcd_file_list, num_pcd_files] = nepi_ros.get_file_list(Pointcloud_Path,'pcd')
      print(num_pcd_files)
      [transforms_file_list, num_tf_files] = nepi_ros.get_file_list(Pointcloud_Path,'txt')
      if num_pcd_files > 0:
        print("Found pointcloud files")
        for pcd_file in pcd_file_list:
          pcd_filename = os.path.basename(pcd_file)
          print(pcd_filename)
          transform_file = pcd_file.replace('.pcd','_transform.txt')
          if transform_file not in transforms_file_list:
            print("No transform file found, so creating one")
            new_file = open(transform_file,'w')
            new_file.writelines(Zero_Transform)
            new_file.close()
      else:
        print("No pcd files found in folder " + Pointcloud_Path)
    else:
      print("Folder " + Pointcloud_Path + " not found")

    # Create a transform publisher
    transform_pub = rospy.Publisher(TRANSFORM_PUB_NAMESPACE,Frame3DTransformUpdate,queue_size=10)
    # Read Pointclouds in from found files in folder
    self.pc_pubs_list = []
    self.pc_msgs_list = []
    [transforms_file_list, num_tf_files] = nepi_ros.get_file_list(Pointcloud_Path,'txt')
    print("Loading tranfer files")
    for transform_file in transforms_file_list:
          transform_filename = os.path.basename(transform_file).split('.')[0]
          print(transform_filename)
    for pcd_file in pcd_file_list:
      o3d_pc = o3d.io.read_point_cloud(pcd_file)
      self.pc_msgs_list.append(nepi_pc.o3dpc_to_rospc(o3d_pc))
      pc_topic_name = os.path.basename(pcd_file).split('.')[0]
      pc_topic_name = pc_topic_name.replace('-','_')
      pc_topic_name = pc_topic_name.replace('.','')
      pc_namespace = NEPI_BASE_NAMESPACE + 'pointclouds/' + pc_topic_name
      # Publish pointcloud transforms to pointcloud app
      transform_file = pcd_file.split('.')[0] + ".txt"
      if transform_file in transforms_file_list:
        file = open(transform_file,'r')
        transform_str = file.readline()
        file.close()
        transform_str_list = transform_str.split(' ')
        transform_val_list = []
        for str_val in transform_str_list:
          transform_val_list.append(float(str_val))
        transform_msg = Frame3DTransform()
        transform_msg.translate_vector.x = transform_val_list[0]
        transform_msg.translate_vector.y  = transform_val_list[1]
        transform_msg.translate_vector.z  = transform_val_list[2]
        transform_msg.rotate_vector.x = transform_val_list[3]
        transform_msg.rotate_vector.y = transform_val_list[4]
        transform_msg.rotate_vector.z = transform_val_list[5]
        transform_msg.heading_offset = transform_val_list[6]

        transform_update_msg = Frame3DTransformUpdate()
        transform_update_msg.topic_namespace = pc_namespace
        transform_update_msg.transform = transform_msg
        print("Publishing transform update msg")
        print(transform_update_msg)
        transform_pub.publish(transform_update_msg)
      print("Creating pointcloud publisher: " + pc_namespace)
      self.pc_pubs_list.append(rospy.Publisher(pc_namespace, PointCloud2, queue_size=1))
    nepi_ros.sleep(1,10)
    # Start Pointclouds publisher
    pub_interval_sec = float(1)/Publish_Image_Rate_Hz
    print("Starting pointcloud publishers")
    rospy.Timer(rospy.Duration(pub_interval_sec), self.publish_pcs_callback)
  


    ## Initiation Complete
    rospy.loginfo("Initialization Complete")

  #######################
  ### Node Methods

  ### Callback to publish pointcloud_test
  def publish_pcs_callback(self,timer):
    while(len(self.pc_pubs_list) != len(self.pc_msgs_list)):
      nepi_ros.sleep(1,10)
    seq_num = self.seq_num + 1
    for ind in range(len(self.pc_msgs_list)):  
      if not rospy.is_shutdown():
        self.pc_msgs_list[ind].header.stamp = rospy.Time.now()
        self.pc_pubs_list[ind].publish(self.pc_msgs_list[ind])

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






