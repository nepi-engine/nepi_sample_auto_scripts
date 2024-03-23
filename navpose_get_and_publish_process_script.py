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
# 1. Calls the NEPI ROS's nav_pose_mgr/query_data_products service
# 2. Gets and publishes current navpose solution data at set rate to new topics:
#  navpose = NEPI NavPose Message
#  heading_deg = Float64 (heading in degrees)
#  orientation_ned_degs = [Float64, Float64, Float64] (roll, pitch, yaw in +-180 degrees NED frame)  
#  orientation_enu_degs = [Float64, Float64, Float64] (roll, pitch, yaw in +-180 degrees ENU frame)  
#  position_ned_m = [Float64, Float64, Float64] (x, y, z in meters NED frame)  
#  position_enu_m = [Float64, Float64, Float64] (x, y, z in meters ENU frame)  
#  location_amsl_geo = [Float64, Float64, Float64] (lat, long, altitude in meters AMSL height)
#  location_wgs84_geo = [Float64, Float64, Float64] (lat, long, altitude in meters WGS-84 Ellipoid height)
#  geoid_height_m = Float64 (meters geoid height added to AMSL height to convert to WGS84 height)
#
# Orientation and Position data are published in both ROS standard ENU reference frame
# and a robot standard NED reference frame for convenience. Learn more about these frames and converting them at:
# https://github.com/mavlink/mavros/issues/216
#
# Location Geo Altitudes are published in both meters above mean sea level (AMSL)and meters above the WGS-84 Ellipsoid (WGS84)

# Requires the following python modules installed for real-time geoid_height calculations.
# Not available for NEPI 2.0.# versions that use python 2.7, so will defualt to CURRENT_GEOID_HEIGHT_M user set value
# Pre installed on NEPI 2.1.2+
# a) geographiclib  [link text](https://pypi.org/project/geographiclib/)
# b) PyGeodesy  [link text](https://pypi.org/project/PyGeodesy/)
# Plus at least one geoid file data base from [link text](https://sourceforge.net/projects/geographiclib/files/geoids-distrib/)
# The following geoid files are Pre installed on NEPI 2.1.1+ in
# '/mnt/nepi_storage/databases/geoids/egm96-15.pgm'  - Small memory footrospy.loginfo, but less accurate
# '/mnt/nepi_storage/databases/geoids/egm2008-2_5.pgm'  - Large memory footrospy.loginfo, but less accurate
# Set the database to use below
# For NEPI 2.0.# versions, the system will default to the following user entered value for the current Lat Long working location
# Find the geoid_height value for a Lat Long location at: [link text](https://geodesy.noaa.gov/GEOID/GEOID18/computation.html)


import rospy
import numpy as np
import math
import time
import sys
import tf
from resources import nepi
from resources import nepi_navpose

from std_msgs.msg import Bool, String, Float64, Float64MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped
from nepi_ros_interfaces.msg import NavPose
from nepi_ros_interfaces.srv import NavPoseQuery, NavPoseQueryRequest

#########################################
# USER SETTINGS - Edit as Necessary 
#########################################

NAVPOSE_PUB_RATE_HZ = 10


#########################################
# ROS NAMESPACE SETUP
#########################################

# ROS namespace setup
NEPI_BASE_NAMESPACE = nepi.get_base_namespace()

#########################################
# Node Class
#########################################

class navpose_get_and_publish_process(object):

  #######################
  ### Node Initialization
  def __init__(self):
    rospy.loginfo("Starting Initialization Processes")
    ## Initialize Class Variables
    self.navpose_pub_rate_hz = NAVPOSE_PUB_RATE_HZ
    ## Define Class Namespaces
    # NavPose Heading, Orientation, Location, and Position Publish Topics
    NAVPOSE_PUBLISH_NAVPOSE_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose/navpose"
    NAVPOSE_PUBLISH_HEADING_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose/heading_deg"
    NAVPOSE_PUBLISH_ORIENTATION_NED_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose/orientation_ned_degs"
    NAVPOSE_PUBLISH_ORIENTATION_ENU_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose/orientation_enu_degs"
    NAVPOSE_PUBLISH_POSITION_NED_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose/position_ned_m"
    NAVPOSE_PUBLISH_POSITION_ENU_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose/position_enu_m"
    NAVPOSE_PUBLISH_LOCATION_AMSL_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose/location_amsl_geo"
    NAVPOSE_PUBLISH_LOCATION_WGS84_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose/location_wgs84_geo"
    NAVPOSE_PUBLISH_GEOID_HEIGHT_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose/geoid_height_m"
    ## Define Class Services Calls
    self.NAVPOSE_SERVICE_NAME = NEPI_BASE_NAMESPACE + "nav_pose_query"
    ## Create Class Sevices    
    ## Create Class Publishers
    self.navpose_navpose_pub = rospy.Publisher(NAVPOSE_PUBLISH_NAVPOSE_TOPIC, NavPose, queue_size=1)
    self.navpose_heading_pub = rospy.Publisher(NAVPOSE_PUBLISH_HEADING_TOPIC, Float64, queue_size=1)
    self.navpose_orientation_ned_pub = rospy.Publisher(NAVPOSE_PUBLISH_ORIENTATION_NED_TOPIC, Float64MultiArray , queue_size=1)
    self.navpose_orientation_enu_pub = rospy.Publisher(NAVPOSE_PUBLISH_ORIENTATION_ENU_TOPIC, Float64MultiArray , queue_size=1)
    self.navpose_position_ned_pub = rospy.Publisher(NAVPOSE_PUBLISH_POSITION_NED_TOPIC, Float64MultiArray , queue_size=1)
    self.navpose_position_enu_pub = rospy.Publisher(NAVPOSE_PUBLISH_POSITION_ENU_TOPIC, Float64MultiArray , queue_size=1)
    self.navpose_location_amsl_pub = rospy.Publisher(NAVPOSE_PUBLISH_LOCATION_AMSL_TOPIC, Float64MultiArray , queue_size=1)
    self.navpose_location_wgs84_pub = rospy.Publisher(NAVPOSE_PUBLISH_LOCATION_WGS84_TOPIC, Float64MultiArray , queue_size=1)
    self.navpose_geoid_height_pub = rospy.Publisher(NAVPOSE_PUBLISH_GEOID_HEIGHT_TOPIC, Float64, queue_size=1)
    self.navpose_pub_interval_sec = float(1.0)/self.navpose_pub_rate_hz
    ## Start Class Subscribers
    ## Start Node Processes
    rospy.wait_for_service(self.NAVPOSE_SERVICE_NAME)
    # Start navpose data publishers
    rospy.Timer(rospy.Duration(self.navpose_pub_interval_sec), self.navpose_get_publish_callback)
    ## Initiation Complete
    rospy.loginfo("Initialization Complete")

  #######################
  ### Node Methods

  ### Setup a regular background navpose get and publish timer callback
  def navpose_get_publish_callback(self,timer):
    # Get current NEPI NavPose data from NEPI ROS nav_pose_query service call
    try:
      get_navpose_service = rospy.ServiceProxy(self.NAVPOSE_SERVICE_NAME, NavPoseQuery)
      nav_pose_response = get_navpose_service(NavPoseQueryRequest())
      #rospy.loginfo(nav_pose_response)
      # Get current navpose
      current_navpose = nav_pose_response.nav_pose
      # Get current heading in degrees
      current_heading_deg = nepi_navpose.get_navpose_heading_deg(nav_pose_response)
      # Get current orientation vector (roll, pitch, yaw) in degrees enu frame
      current_orientation_enu_degs = Float64MultiArray()
      current_orientation_enu_degs.data = nepi_navpose.get_navpose_orientation_enu_degs(nav_pose_response)
      # Get current orientation vector (roll, pitch, yaw) in degrees ned frame +-180
      current_orientation_ned_degs = Float64MultiArray()
      current_orientation_ned_degs.data = nepi_navpose.get_navpose_orientation_ned_degs(nav_pose_response)
      # Get current position vector (x, y, z) in meters enu frame
      current_position_enu_m = Float64MultiArray()
      current_position_enu_m.data = nepi_navpose.get_navpose_position_enu_m(nav_pose_response)
      # Get current position vector (x, y, z) in meters ned frame
      current_position_ned_m = Float64MultiArray()
      current_position_ned_m.data = nepi_navpose.get_navpose_position_ned_m(nav_pose_response)
      # Get current location vector (lat, long, alt) in geopoint data with WGS84 height
      current_location_wgs84_geo = Float64MultiArray()
      current_location_wgs84_geo.data =  nepi_navpose.get_navpose_location_wgs84_geo(nav_pose_response)  
      # Get current location vector (lat, long, alt) in geopoint data with AMSL height
      current_location_amsl_geo = Float64MultiArray()
      current_location_amsl_geo.data =  nepi_navpose.get_navpose_location_amsl_geo(nav_pose_response)
      # Get current geoid heihgt
      current_geoid_height =  nepi_navpose.get_navpose_geoid_height(nav_pose_response)
      # Publish new current navpose data
      self.navpose_navpose_pub.publish(current_navpose)
      self.navpose_heading_pub.publish(current_heading_deg)
      self.navpose_orientation_ned_pub.publish(current_orientation_ned_degs)
      self.navpose_orientation_enu_pub.publish(current_orientation_enu_degs)
      self.navpose_position_ned_pub.publish(current_position_ned_m)
      self.navpose_position_enu_pub.publish(current_position_enu_m)
      self.navpose_location_amsl_pub.publish(current_location_amsl_geo)
      self.navpose_location_wgs84_pub.publish(current_location_wgs84_geo)
      self.navpose_geoid_height_pub.publish(current_geoid_height)
    except rospy.ServiceException as e:
      rospy.loginfo("Service call failed: %s"%e)
      time.sleep(1)
      rospy.signal_shutdown("Service call failed")

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



