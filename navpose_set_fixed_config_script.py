#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
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
from nepi_edge_sdk_base import nepi_ros 
from nepi_edge_sdk_base import nepi_msg
from nepi_edge_sdk_base import nepi_nav

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
# Node Class
#########################################

class navpose_set_fixed_config(object):

  #######################
  ### Node Initialization
  DEFAULT_NODE_NAME = "navpose_set_fixed_config" # Can be overwitten by luanch command
  def __init__(self):
    #### APP NODE INIT SETUP ####
    nepi_ros.init_node(name= self.DEFAULT_NODE_NAME)
    self.node_name = nepi_ros.get_node_name()
    self.base_namespace = nepi_ros.get_base_namespace()
    nepi_msg.createMsgPublishers(self)
    nepi_msg.publishMsgInfo(self,"Starting Initialization Processes")
    ##############################
    ## Initialize Class Variables
    ## Define Class Namespaces
    SET_NAVPOSE_FIXED_GPS_TOPIC = self.base_namespace + "nav_pose_mgr/set_init_gps_fix"
    SET_NAVPOSE_FIXED_HEADING_TOPIC = self.base_namespace + "nav_pose_mgr/set_init_heading"
    SET_NAVPOSE_FIXED_ORIENTATION_TOPIC = self.base_namespace + "nav_pose_mgr/set_init_orientation"
    REINIT_NAVPOSE_SOLUTION_TOPIC = self.base_namespace + "nav_pose_mgr/reinit_solution"
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
    current_orientation_quat = nepi_nav.convert_rpy2quat(START_ORIENTATION_DEGS)
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

    ##############################
    ## Initiation Complete
    nepi_msg.publishMsgInfo(self," Initialization Complete")
    # Spin forever (until object is detected)
    rospy.spin()
    ##############################

  #######################
  ### Node Methods
  

  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    nepi_msg.publishMsgInfo(self,"Shutting down: Executing script cleanup actions")


#########################################
# Main
#########################################
if __name__ == '__main__':
  navpose_set_fixed_config()

















