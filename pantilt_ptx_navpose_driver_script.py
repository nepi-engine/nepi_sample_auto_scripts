#!/usr/bin/env python
#
# NEPI Dual-Use License
# Project: nepi_sample_auto_scripts
#
# This license applies to any user of NEPI Engine software
#
# Copyright (C) 2023 Numurus, LLC <https://www.numurus.com>
# see https://github.com/numurus-nepi/nepi_edge_sdk_base
#
# This software is dual-licensed under the terms of either a NEPI software developer license
# or a NEPI software commercial license.
#
# The terms of both the NEPI software developer and commercial licenses
# can be found at: www.numurus.com/licensing-nepi-engine
#
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - https://www.numurus.com/licensing-nepi-engine
# - mailto:nepi@numurus.com
#
#


# Sample NEPI Automation Script.
# Uses onboard ROS python library to
# 1. Subscribes to NEPI NEPI_PTX supported pantilt status message
# 2. Creates an Orientation Publishers and Sets NEPI NavPose to connect to them

###################################################
# NEPI NavPose Axis Info
# x+ axis is forward
# y+ axis is right
# z+ axis is down
# roll: RHR about x axis
# pitch: RHR about y axis
# yaw: RHR about z axis
#####################################################


import rospy
import time
import numpy as np
import math
import tf


from std_msgs.msg import String, Float64, Float64MultiArray, Empty
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped, QuaternionStamped
from nepi_ros_interfaces.msg import PanTiltStatus, StringArray

#####################################################################################
# SETUP - Edit as Necessary ##################################
##########################################


NAVPOSE_UPDATE_RATE_HZ = 10
# Pan and Tilt setup parameters
PT_REVERSE_PAN = True # Flip Sensor Feedback Values for NavPose Body Frame
PT_REVERSE_TILT = True # Flip Sensor Feedback Values for NavPose Body Frame
# Set Start roll pitch yaw body frame values
START_RPY_DEGS =  [0.0,0.0,0.0]# Roll, Pitch, Yaw in body frame degs


### ROS namespace setup
NEPI_BASE_NAMESPACE = "/nepi/s2x/"
NEPI_PTX_NAMESPACE = NEPI_BASE_NAMESPACE + "iqr_pan_tilt/ptx/"

### PanTilt Subscribe Topics
NEPI_PTX_GET_STATUS_TOPIC = NEPI_PTX_NAMESPACE + "status"
### PanTilt NavPose Publish Topic
NEPI_PTX_NAVPOSE_ODOM_TOPIC = NEPI_PTX_NAMESPACE + "odom"
### NEPI NavPose Setting Publish Topic
NEPI_SET_NAVPOSE_SET_ORIENTATION_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose_mgr/set_orientation_topic"


#####################################################################################
# Globals
#####################################################################################
navpose_pt_orientation_pub = rospy.Publisher(NEPI_PTX_NAVPOSE_ODOM_TOPIC, Odometry , queue_size=1)
navpose_update_interval_sec = float(1.0)/NAVPOSE_UPDATE_RATE_HZ
current_rpy_pt_degs = START_RPY_DEGS

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
  global current_rpy_pt_degs
  global navpose_update_interval_sec
  print("")
  print("Starting Initialization")
  # Start PT Status Callback
  wait_for_topic(NEPI_PTX_GET_STATUS_TOPIC, 'nepi_ros_interfaces/PanTiltStatus')
  print("Starting Pan Tilt Stutus callback")
  rospy.Subscriber(NEPI_PTX_GET_STATUS_TOPIC, PanTiltStatus, pt_status_callback)  
  ##############################
  # Start our Update NavPose Publisher Topic
  print("Starting NavPose Update Publisher at: " + str(NAVPOSE_UPDATE_RATE_HZ) + " Hz")
  rospy.Timer(rospy.Duration(navpose_update_interval_sec), ptx_navpose_odom_publish_callback)
  time.sleep(2) # Wait for publiser to start
  print("Initialization Complete")

### Setup a regular background navpose update timer callback
def ptx_navpose_odom_publish_callback(timer):
  global current_rpy_pt_degs
  global navpose_pt_orientation_pub
  # Create Position Data
  new_pos = Point()
  new_pos.x = 0
  new_pos.y = 0
  new_pos.z = 0
  # Create Orientation Data
  current_orientation_quat = convert_rpy2quat(current_rpy_pt_degs)
  new_quat = Quaternion()
  new_quat.x = current_orientation_quat[0]
  new_quat.y = current_orientation_quat[1]
  new_quat.z = current_orientation_quat[2]
  new_quat.w = current_orientation_quat[3]
  # Combine for Pose Data
  new_pose=Pose()
  new_pose.position = new_pos
  new_pose.orientation = new_quat
  # Create NavPose Odom Message
  new_odometry = Odometry()
  new_odometry.header.stamp = rospy.Time.now()
  new_odometry.header.frame_id = 'map'
  new_odometry.child_frame_id = 'nepi_center_frame'
  new_odometry.pose.pose = new_pose
  if not rospy.is_shutdown():
    navpose_pt_orientation_pub.publish(new_odometry)

### Simple callback to get pt status info
def pt_status_callback(PanTiltStatus):
  global current_rpy_pt_degs
  # This is just to get the current pt positions
  pt_yaw_now_deg=PanTiltStatus.yaw_now_deg
  pt_pitch_now_deg=PanTiltStatus.pitch_now_deg
  if PT_REVERSE_PAN:
    pt_yaw_now_deg = -pt_yaw_now_deg
  if PT_REVERSE_TILT:
    pt_pitch_now_deg = -pt_pitch_now_deg
  current_rpy_pt_degs=[START_RPY_DEGS[0],pt_pitch_now_deg,pt_yaw_now_deg]


### Function to Convert Roll, Pitch, Yaw Degrees to Quaternion Attitude
def convert_rpy2quat(rpy_attitude_deg):
  roll_rad = math.radians(rpy_attitude_deg[0])
  pitch_rad = math.radians(rpy_attitude_deg[1]) 
  yaw_rad = math.radians(rpy_attitude_deg[2])
  #xyzw_attitude = tf.transformations.quaternion_from_euler(roll_rad,pitch_rad,yaw_rad,axes="sxyz")
  xyzw_attitude = tf.transformations.quaternion_from_euler(pitch_rad, yaw_rad, roll_rad, axes="ryzx")
  return xyzw_attitude


### Function to wait for topic to exist
def wait_for_topic(topic_name,message_name):
  topic_in_list = False
  while topic_in_list is False and not rospy.is_shutdown():
    topic_list=rospy.get_published_topics(namespace='/')
    topic_to_connect=[topic_name, message_name]
    if topic_to_connect not in topic_list:
      time.sleep(.1)
    else:
      topic_in_list = True

### Cleanup processes on node shutdown
def cleanup_actions():
  print("Shutting down: Executing script cleanup actions")

### Script Entrypoint
def startNode():
  rospy.loginfo("Starting Pantilt PTX NavPose Orienation Driver Script")
  rospy.init_node("pantilt_ptx_navpose_driver_script")
  # Run initialization processes
  initialize_actions()
  # run cleanup actions on shutdown
  rospy.on_shutdown(cleanup_actions)
  # Spin forever
  rospy.spin()
  
#####################################################################################
# Main
#####################################################################################

if __name__ == '__main__':
  startNode()

