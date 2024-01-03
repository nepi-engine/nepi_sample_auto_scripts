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
# Uses onboard ROS python libraries to
# 1. call the NEPI ROS's nav_pose_mgr/query_data_products service
# 2. gets and publishes current navpose solution data at set rate to new topics:
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
# '/mnt/nepi_storage/databases/geoids/egm96-15.pgm'  - Small memory footprint, but less accurate
# '/mnt/nepi_storage/databases/geoids/egm2008-2_5.pgm'  - Large memory footprint, but less accurate
# Set the database to use below
GEOID_DATABASE_FILE='/mnt/nepi_storage/databases/geoids/egm2008-2_5.pgm' # Ignored if PyGeodesy module or Geoids Database is not available
# For NEPI 2.0.# versions, the system will default to the following user entered value for the current Lat Long working location
# Find the geoid_height value for a Lat Long location at: [link text](https://geodesy.noaa.gov/GEOID/GEOID18/computation.html)
FALLBACK_GEOID_HEIGHT_M = 22.0 # Ignored if if PyGeodesy module or Geoids Database are available

import rospy
import numpy as np
import math
import time
import sys
import tf
# try and import geoid height calculation module and databases
try:
  print('Importing PyGeodesy module')
  import pygeodesy
  from pygeodesy.ellipsoidalKarney import LatLon
  try:
    print(['Loading Geoids Database from: ' + GEOID_DATABASE_FILE])
    ginterpolator = pygeodesy.GeoidKarney(GEOID_DATABASE_FILE)
    USE_FALLBACK_GEOID_HEIGHT = False
  except:
    print('Geoids database failed to import')
    print(['Using FALLBACK_GEOID_HEIGHT_M value: ' + str(FALLBACK_GEOID_HEIGHT_M)])
    USE_FALLBACK_GEOID_HEIGHT = True
except:
  print('PyGeodesy module not available')
  print(['Using FALLBACK_GEOID_HEIGHT_M value: ' + str(FALLBACK_GEOID_HEIGHT_M)])
  USE_FALLBACK_GEOID_HEIGHT = True

from std_msgs.msg import Bool, String, Float64, Float64MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped
from nepi_ros_interfaces.msg import NavPose
from nepi_ros_interfaces.srv import NavPoseQuery, NavPoseQueryRequest

#####################################################################################
# SETUP - Edit as Necessary ##################################
##########################################

NAVPOSE_PUB_RATE_HZ = 10


# ROS namespace setup
NEPI_BASE_NAMESPACE = "/nepi/s2x/"

# NEPI Get NAVPOSE Solution Service Name
NAVPOSE_SERVICE_NAME = NEPI_BASE_NAMESPACE + "nav_pose_query"

### NavPose Heading, Orientation, Location, and Position Publish Topics
NAVPOSE_PUBLISH_NAVPOSE_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose/navpose"
NAVPOSE_PUBLISH_HEADING_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose/heading_deg"
NAVPOSE_PUBLISH_ORIENTATION_NED_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose/orientation_ned_degs"
NAVPOSE_PUBLISH_ORIENTATION_ENU_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose/orientation_enu_degs"
NAVPOSE_PUBLISH_POSITION_NED_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose/position_ned_m"
NAVPOSE_PUBLISH_POSITION_ENU_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose/position_enu_m"
NAVPOSE_PUBLISH_LOCATION_AMSL_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose/location_amsl_geo"
NAVPOSE_PUBLISH_LOCATION_WGS84_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose/location_wgs84_geo"
NAVPOSE_PUBLISH_GEOID_HEIGHT_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose/geoid_height_m"


#####################################################################################
# Globals
#####################################################################################
navpose_navpose_pub = rospy.Publisher(NAVPOSE_PUBLISH_NAVPOSE_TOPIC, NavPose, queue_size=1)
navpose_heading_pub = rospy.Publisher(NAVPOSE_PUBLISH_HEADING_TOPIC, Float64, queue_size=1)
navpose_orientation_ned_pub = rospy.Publisher(NAVPOSE_PUBLISH_ORIENTATION_NED_TOPIC, Float64MultiArray , queue_size=1)
navpose_orientation_enu_pub = rospy.Publisher(NAVPOSE_PUBLISH_ORIENTATION_ENU_TOPIC, Float64MultiArray , queue_size=1)
navpose_position_ned_pub = rospy.Publisher(NAVPOSE_PUBLISH_POSITION_NED_TOPIC, Float64MultiArray , queue_size=1)
navpose_position_enu_pub = rospy.Publisher(NAVPOSE_PUBLISH_POSITION_ENU_TOPIC, Float64MultiArray , queue_size=1)
navpose_location_amsl_pub = rospy.Publisher(NAVPOSE_PUBLISH_LOCATION_AMSL_TOPIC, Float64MultiArray , queue_size=1)
navpose_location_wgs84_pub = rospy.Publisher(NAVPOSE_PUBLISH_LOCATION_WGS84_TOPIC, Float64MultiArray , queue_size=1)
navpose_geoid_height_pub = rospy.Publisher(NAVPOSE_PUBLISH_GEOID_HEIGHT_TOPIC, Float64, queue_size=1)
navpose_pub_interval_sec = float(1.0)/NAVPOSE_PUB_RATE_HZ


#####################################################################################
# Methods
#####################################################################################

### System Initialization processes
def initialize_actions():
  print("")
  print("Starting Initialization")
  print("Waiting for NEPI NavPose service") 
  rospy.wait_for_service(NAVPOSE_SERVICE_NAME)
  print("Initialization Complete")


### Setup a regular background navpose get and publish timer callback
def navpose_get_publish_callback(timer):
  global navpose_heading_pub 
  global navpose_heading_pub 
  global navpose_orientation_ned_pub 
  global navpose_orientation_enu_pub 
  global navpose_position_ned_pub 
  global navpose_position_enu_pub 
  global navpose_location_amsl_pub
  global navpose_location_wgs84_pub
  global navpose_geoid_height_pub
  # Get current NEPI NavPose data from NEPI ROS nav_pose_query service call
  try:
    get_navpose_service = rospy.ServiceProxy(NAVPOSE_SERVICE_NAME, NavPoseQuery)
    nav_pose_response = get_navpose_service(NavPoseQueryRequest())
    #print(nav_pose_response)
    # Set current navpose
    current_navpose = nav_pose_response.nav_pose
    
    # Set current heading in degrees
    current_heading_deg = nav_pose_response.nav_pose.heading.heading

    # Set current orientation vector (roll, pitch, yaw) in degrees enu frame
    pose_enu_o = nav_pose_response.nav_pose.odom.pose.pose.orientation
    xyzw_enu_o = list([pose_enu_o.x,pose_enu_o.y,pose_enu_o.z,pose_enu_o.w])
    rpy_enu_d = convert_quat2rpy(xyzw_enu_o)
    current_orientation_enu_degs = Float64MultiArray()
    current_orientation_enu_degs.data = [rpy_enu_d[0],rpy_enu_d[1],rpy_enu_d[2]]

    # Set current orientation vector (roll, pitch, yaw) in degrees ned frame +-180
    pose_enu_o = nav_pose_response.nav_pose.odom.pose.pose.orientation
    xyzw_enu_o = list([pose_enu_o.x,pose_enu_o.y,pose_enu_o.z,pose_enu_o.w])
    rpy_enu_d = convert_quat2rpy(xyzw_enu_o)
    yaw_ned_d = convert_yaw_enu2ned(rpy_enu_d[2])
    rpy_ned_d = [rpy_enu_d[0],rpy_enu_d[1],yaw_ned_d]
    current_orientation_ned_degs = Float64MultiArray()
    current_orientation_ned_degs.data = [rpy_ned_d[0],rpy_ned_d[1],rpy_ned_d[2]]

    # Set current position vector (x, y, z) in meters enu frame
    pose_enu_p = nav_pose_response.nav_pose.odom.pose.pose.position
    current_position_enu_m = Float64MultiArray()
    current_position_enu_m.data = [pose_enu_p.x, pose_enu_p.y, pose_enu_p.z]

    # Set current position vector (x, y, z) in meters ned frame
    pose_enu_p = nav_pose_response.nav_pose.odom.pose.pose.position
    current_position_ned_m = Float64MultiArray()
    current_position_ned_m.data = [pose_enu_p.y, pose_enu_p.x, -pose_enu_p.z]

    # Set current location vector (lat, long, alt) in geopoint data with AMSL height
    fix_amsl = nav_pose_response.nav_pose.fix
    current_location_amsl_geo = Float64MultiArray()
    current_location_amsl_geo.data =  [fix_amsl.latitude,fix_amsl.longitude,fix_amsl.altitude]

    # Set current geoid heihgt
    if USE_FALLBACK_GEOID_HEIGHT:
      geoid_height=FALLBACK_GEOID_HEIGHT_M #### user hard coaded geoid height at current lat long location
    else:
      single_position=LatLon(fix_amsl.latitude,fix_amsl.longitude)
      geoid_height = ginterpolator(single_position)
    current_geoid_height =  geoid_height

    # Set current location vector (lat, long, alt) in geopoint data with WGS84 height
    fix_amsl = nav_pose_response.nav_pose.fix
    current_location_wgs84_geo = Float64MultiArray()
    current_location_wgs84_geo.data =  [fix_amsl.latitude,fix_amsl.longitude,(fix_amsl.altitude + geoid_height)]

    # Publish new current navpose data
    navpose_navpose_pub.publish(current_navpose)
    navpose_heading_pub.publish(current_heading_deg)
    navpose_orientation_ned_pub.publish(current_orientation_ned_degs)
    navpose_orientation_enu_pub.publish(current_orientation_enu_degs)
    navpose_position_ned_pub.publish(current_position_ned_m)
    navpose_position_enu_pub.publish(current_position_enu_m)
    navpose_location_amsl_pub.publish(current_location_amsl_geo)
    navpose_location_wgs84_pub.publish(current_location_wgs84_geo)
    navpose_geoid_height_pub.publish(current_geoid_height)
  except rospy.ServiceException as e:
    print("Service call failed: %s"%e)
    time.sleep(1)
    rospy.signal_shutdown("Service call failed")

### Function to Convert Quaternion Attitude to Roll, Pitch, Yaw Degrees
def convert_quat2rpy(xyzw_attitude):
  rpy_attitude_rad = tf.transformations.euler_from_quaternion(xyzw_attitude)
  rpy_attitude_deg = np.array(rpy_attitude_rad) * 180/math.pi
  roll_deg = rpy_attitude_deg[0] 
  pitch_deg = rpy_attitude_deg[1] 
  yaw_deg = rpy_attitude_deg[2]
  return rpy_attitude_deg

### Function to Convert Roll, Pitch, Yaw Degrees to Quaternion Attitude
def convert_rpy2quat(rpy_attitude_deg):
  roll_deg = rpy_attitude_deg[0] 
  pitch_deg = rpy_attitude_deg[1] 
  yaw_deg = rpy_attitude_deg[2]
  xyzw_attitude = tf.transformations.quaternion_from_euler(math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg))
  return xyzw_attitude

### Function to Convert Yaw NED to Yaw ENU
def convert_yaw_ned2enu(yaw_ned_deg):
  yaw_enu_deg = 90-yaw_ned_deg
  if yaw_enu_deg < -180:
    yaw_enu_deg = 360 + yaw_enu_deg
  elif yaw_enu_deg > 180:
    yaw_enu_deg = yaw_enu_deg - 360
  return yaw_enu_deg

### Function to Convert Yaw ENU to Yaw NED
def convert_yaw_enu2ned(yaw_enu_deg):
  yaw_ned_deg =  90-yaw_enu_deg
  if yaw_ned_deg < -180:
    yaw_ned_deg = 360 + yaw_ned_deg
  elif yaw_ned_deg > 180:
    yaw_ned_deg = yaw_ned_deg - 360
  return yaw_ned_deg

### Cleanup processes on node shutdown
def cleanup_actions():
  print("Shutting down: Executing script cleanup actions")


### Script Entrypoint
def startNode():
  rospy.init_node("set_mavlink_navpose_auto_script")
  rospy.loginfo("Starting Set MAVLink NavPose automation script")
  # Run initialization processes
  initialize_actions()
  # Start navpose data publishers
  rospy.Timer(rospy.Duration(navpose_pub_interval_sec), navpose_get_publish_callback)
  # run cleanup actions on shutdown
  rospy.on_shutdown(cleanup_actions)
  # Spin forever
  rospy.spin()


#####################################################################################
# Main
#####################################################################################

if __name__ == '__main__':
  startNode()

