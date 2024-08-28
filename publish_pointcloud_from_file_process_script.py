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
# 1. Open a pointcloud file and publish as ros pointcloud2 message at specified rate
# 2. Run until Stopped

import os
import time
import sys
import rospy
import numpy as np
import open3d as o3d

from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
from std_msgs.msg import UInt8, Empty, String, Bool

from std_msgs.msg import Header
from sensor_msgs.msg import PointField
from sensor_msgs import point_cloud2

from nepi_edge_sdk_base import nepi_ros
from nepi_edge_sdk_base import nepi_pc 

#########################################
# USER SETTINGS - Edit as Necessary 
#########################################
### Open and publish pointcloud
Current_Path = (os.path.dirname(os.path.realpath(__file__)) )

### Open and publish image
Pointcloud_Path = Current_Path + "/sample_data"
Pointcloud_File="test_pointcloud.pcd"

Publish_Image_Topic_Name = "test_pointcloud"
Publish_Image_Rate_Hz = 2

#########################################
# ROS NAMESPACE SETUP
#########################################

# ROS namespace setup
NEPI_BASE_NAMESPACE = nepi_ros.get_base_namespace()

#########################################
# Node Class
#########################################

class publish_pointcloud_from_file_process(object):
  seq_num = 0
  #######################
  ### Node Initialization
  def __init__(self):
    rospy.loginfo("Starting Initialization Processes")
    ## Initialize Class Variables
    ## Define Class Namespaces
    POINTCLOUD_OUTPUT_TOPIC = NEPI_BASE_NAMESPACE + Publish_Image_Topic_Name
    ## Define Class Services Calls
    ## Create Class Sevices    
    ## Create Class Publishers
    # A custom pointcloud topic
    self.custom_pointcloud_pub = rospy.Publisher(POINTCLOUD_OUTPUT_TOPIC, PointCloud2, queue_size=10)
    ## Start Class Subscribers

    ## Start Node Processes
    # Read PointCloud fron File
    pc_in = (Pointcloud_Path + Pointcloud_File)
    time_start = time.time()
    self.open3d_pc = o3d.io.read_point_cloud(pc_in)
    print(type(self.open3d_pc))	
    print(isinstance(self.open3d_pc,PointCloud))
    self.pc2_out_msg = nepi_pc.o3dpc_to_rospc(self.open3d_pc)
    print("Number of points: " + str(self.pc2_out_msg.width))
    print("PointCloud2 header")
    print(self.pc2_out_msg.header)
    print("PointCloud2 fields")
    print(self.pc2_out_msg.fields)
    print("Number of points: " + str(self.pc2_out_msg.width))
    #*************
    print("O3D to ROS Time:")
    process_time = time.time() - time_start
    print("%.2f" % process_time)
    time_start = time.time()
    #*************

    
    # Start publisher
    pub_interval_sec = float(1)/Publish_Image_Rate_Hz
    rospy.Timer(rospy.Duration(pub_interval_sec), self.publish_test_pc2_callback)
    ## Initiation Complete
    rospy.loginfo("Initialization Complete")

  #######################
  ### Node Methods

  ### Callback to publish pointcloud_test
  def publish_test_pc2_callback(self,timer):
    self.seq_num = self.seq_num + 1
    self.pc2_out_msg.header.seq = self.seq_num
    # Publish new pointcloud to ros
    if not rospy.is_shutdown():
      self.custom_pointcloud_pub.header.stamp = rospy.Time.now()
      self.custom_pointcloud_pub.publish(self.pc2_out_msg)
      # You can view the enhanced_2D_pointcloud topic at 
      # //192.168.179.103:9091/ in a connected web browser

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






