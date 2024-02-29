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


# Sample NEPI Config Script.
# If your NEPI system does not have an attached GPS/IMU/Compass or other
# NavPose source, this script can be set to run at startup setting fixed
# NavPose values on your system.
# 1. Sets a fixed NavPose Solution (Lat,Long,Alt,Heading,Roll,Pitch,Yaw)


import rospy
import math
import tf
import time
import sys
from resources import nepi
from resources import nepi_navpose

from std_msgs.msg import Float64, Empty
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Quaternion, QuaternionStamped

#########################################
# USER SETTINGS - Edit as Necessary 
#########################################

# Set Start Fixed NavPose Values
#Numurus Office
START_GEOPOINT = [47.6540828,-122.3187578,0.0] # [Lat, Long, Altitude_AMSL_M]
START_HEADING_DEG = 88.0 # Global True North, or 0 for Body Relative
START_ORIENTATION_DEGS = [10.0,20.0,30.0]

#########################################
# ROS NAMESPACE SETUP
#########################################

# ROS namespace setup
NEPI_BASE_NAMESPACE = "/nepi/s2x/"

#########################################
# Node Class
#########################################

class navpose_set_fixed_config(object):

  #######################
  ### Node Initialization
  def __init__(self):
    rospy.loginfo("Starting Initialization Processes")
    ## Initialize Class Variables
    ## Define Class Namespaces
    SET_NAVPOSE_FIXED_GPS_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose_mgr/set_init_gps_fix"
    SET_NAVPOSE_FIXED_HEADING_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose_mgr/set_init_heading"
    SET_NAVPOSE_FIXED_ORIENTATION_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose_mgr/set_init_orientation"
    REINIT_NAVPOSE_SOLUTION_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose_mgr/reinit_solution"
    ## Define Class Services Calls
    ## Create Class Sevices    
    ## Create Class Publishers
    ## Start Class Subscribers
    ## Start Node Processes
    # Make sure to use the correct message type: "rostopic info" can help identify it. In this case it is a sensor_msgs/NavSatFix message type
    gps_pub = rospy.Publisher(SET_NAVPOSE_FIXED_GPS_TOPIC, NavSatFix, queue_size=1)
    rospy.sleep(1) # VERY IMPORTANT - Sleep a bit between declaring a publisher and using it subscribers have time to subscribe
    lat = START_GEOPOINT[0]
    long = START_GEOPOINT[1]
    alt = START_GEOPOINT[2]
    gps_pub.publish(latitude=lat, longitude=long, altitude=alt) # Keyword args are a nice way to construct messages right in the publish() function
    heading_pub = rospy.Publisher(SET_NAVPOSE_FIXED_HEADING_TOPIC, Float64, queue_size=1)
    rospy.sleep(1) # VERY IMPORTANT - Sleep a bit between declaring a publisher and using it subscribers have time to subscribe
    heading = START_HEADING_DEG
    heading_pub.publish(data=heading)
    orientation_pub = rospy.Publisher(SET_NAVPOSE_FIXED_ORIENTATION_TOPIC, QuaternionStamped, queue_size=1)
    rospy.sleep(1) # VERY IMPORTANT - Sleep a bit between declaring a publisher and using it subscribers have time to subscribe
    current_orientation_quat = nepi_navpose.convert_rpy2quat(START_ORIENTATION_DEGS)
    new_quat = Quaternion()
    new_quat.x = current_orientation_quat[0]
    new_quat.y = current_orientation_quat[1]
    new_quat.z = current_orientation_quat[2]
    new_quat.w = current_orientation_quat[3]
    orientation_pub.publish(quaternion=new_quat)
    # At this point, the "init" fields have been updated, but they haven't yet been applied as the current values for GPS and HEADING, 
    # so we do that here via the "reinit_solution" topic.
    rospy.sleep(1) # Give new navpose values time to get captured
    reinit_pub = rospy.Publisher(REINIT_NAVPOSE_SOLUTION_TOPIC, Empty, queue_size=10)
    rospy.sleep(1) # VERY IMPORTANT - Sleep a bit between declaring a publisher and using it subscribers have time to subscribe
    reinit_pub.publish() # "Empty" message types don't require a payload
    ## Initiation Complete
    rospy.loginfo("Initialization Complete")

  #######################
  ### Node Methods
  

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

















