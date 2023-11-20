#!/usr/bin/env python

# Sample NEPI Automation Script.
# Uses onboard ROS python and mavros libraries to
# 1. Connect NEPI NavPose topics to appropriate mavros topics
# 2. call the NEPI ROS's nav_pose_mgr/query_data_products service
# 3. gets and publishes current navpose data at set rate to topics
#  heading_deg = Float64 (heading in degrees)
#  orientation_ned_degs = [Float64, Float64, Float64] (roll, pitch, yaw in degrees NED frame)  
#  orientation_enu_degs = [Float64, Float64, Float64] (roll, pitch, yaw in degrees ENU frame)  
#  position_ned_m = [Float64, Float64, Float64] (x, y, z in meters NED frame)  
#  position_enu_m = [Float64, Float64, Float64] (x, y, z in meters ENU frame)  
#  location_amsl_geo = [Float64, Float64, Float64] (lat, long, altitude in meters AMSL height)
#  location_wgs84_geo = [Float64, Float64, Float64] (lat, long, altitude in meters WGS-84 Ellipoid height)
#  geoid_height_m = Float64 (meters geoid height added to AMSL height to convert to WGS84 height)
#
# Orientation and Position data are published in both ROS standard ENU reference frame
# and a robot centric NED reference frame for convenience. Learn more about these frames and converting them at:
# https://github.com/mavlink/mavros/issues/216
#
# Location Geo Altitudes are published in both meters above mean sea level (AMSL)and meters above the WGS-84 Ellipsoid (WGS84)

# Requires the following additional scripts are running
# a) Optional) MAVROS_fake_gps_sim_auto_script.py if a real GPS fix is not available
# These scripts are available for download at:
# [link text](https://github.com/numurus-nepi/nepi_sample_auto_scripts)

import rospy
import mavros
import numpy as np
import math
import time
import sys
import tf

from std_msgs.msg import Bool, Float64, Float64MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped
from nepi_ros_interfaces.srv import NavPoseQuery, NavPoseQueryRequest

# Requires the following additional scripts are running
# a) (Optional) mavros_fake_gps_sim_auto_script.py if a real GPS fix is not available
# These scripts are available for download at:
# [link text](https://github.com/numurus-nepi/nepi_sample_auto_scripts)

import rospy
import time

from std_msgs.msg import String, Empty
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

#####################################################################################
# SETUP - Edit as Necessary ##################################
##########################################


#  GEOID HEIGHT INPUT
### Currently, the geoid_height is hard coded value entered for your current location.
### Plan is to automate this in future updates. For now, you can use this link to get your geoid height value
### for the current Lat Long location: [link text](https://geodesy.noaa.gov/GEOID/GEOID18/computation.html)
CURRENT_GEOID_HEIGHT_M = 22.0 

# ROS namespace setup
BASE_NAMESPACE = "/nepi/s2x/"
MAVROS_NAMESPACE = BASE_NAMESPACE + "pixhawk_mavlink/"

# NEPI Get NAVPOSE Solution Service Name
NAVPOSE_SERVICE_NAME = BASE_NAMESPACE + "nav_pose_query"

### Setup NEPI NavPose Topic Namespaces
NEPI_SET_NAVPOSE_GPS_TOPIC = BASE_NAMESPACE + "nav_pose_mgr/set_gps_fix_topic"
NEPI_SET_NAVPOSE_HEADING_TOPIC = BASE_NAMESPACE + "nav_pose_mgr/set_heading_topic"
NEPI_SET_NAVPOSE_ORIENTATION_TOPIC = BASE_NAMESPACE + "nav_pose_mgr/set_orientation_topic"
NEPI_ENABLE_NAVPOSE_GPS_CLOCK_SYNC_TOPIC = BASE_NAMESPACE + "nav_pose_mgr/enable_gps_clock_sync"

# MAVROS Subscriber Topics
MAVROS_STATE_TOPIC = MAVROS_NAMESPACE + "state"
MAVROS_HEADING_TOPIC = MAVROS_NAMESPACE + "global_position/compass_hdg"
MAVROS_ORIENTATION_TOPIC = MAVROS_NAMESPACE + "global_position/local"
MAVROS_POSITION_LOCAL_TOPIC = MAVROS_NAMESPACE + "global_position/local"
MAVROS_POSITION_GLOBAL_TOPIC = MAVROS_NAMESPACE + "global_position/global"

### NavPose Heading, Oreanation, Location, and Position Publish Topics
NAVPOSE_CURRENT_HEADING_TOPIC = BASE_NAMESPACE + "nav_pose_current/heading_deg"
NAVPOSE_CURRENT_ORIENTATION_NED_TOPIC = BASE_NAMESPACE + "nav_pose_current/orientation_ned_degs"
NAVPOSE_CURRENT_ORIENTATION_ENU_TOPIC = BASE_NAMESPACE + "nav_pose_current/orientation_enu_degs"
NAVPOSE_CURRENT_POSITION_NED_TOPIC = BASE_NAMESPACE + "nav_pose_current/position_ned_m"
NAVPOSE_CURRENT_POSITION_ENU_TOPIC = BASE_NAMESPACE + "nav_pose_current/position_enu_m"
NAVPOSE_CURRENT_LOCATION_AMSL_TOPIC = BASE_NAMESPACE + "nav_pose_current/location_amsl_geo"
NAVPOSE_CURRENT_LOCATION_WGS84_TOPIC = BASE_NAMESPACE + "nav_pose_current/location_wgs84_geo"
NAVPOSE_CURRENT_GEOID_HEIGHT_TOPIC = BASE_NAMESPACE + "nav_pose_current/geoid_height_m"
NAVPOSE_PUB_RATE_HZ = 10

#####################################################################################
# Globals
#####################################################################################
navpose_heading_pub = rospy.Publisher(NAVPOSE_CURRENT_HEADING_TOPIC, Float64, queue_size=1)
navpose_oreantation_ned_pub = rospy.Publisher(NAVPOSE_CURRENT_ORIENTATION_NED_TOPIC, Float64MultiArray , queue_size=1)
navpose_oreantation_enu_pub = rospy.Publisher(NAVPOSE_CURRENT_ORIENTATION_ENU_TOPIC, Float64MultiArray , queue_size=1)
navpose_position_ned_pub = rospy.Publisher(NAVPOSE_CURRENT_POSITION_NED_TOPIC, Float64MultiArray , queue_size=1)
navpose_position_enu_pub = rospy.Publisher(NAVPOSE_CURRENT_POSITION_ENU_TOPIC, Float64MultiArray , queue_size=1)
navpose_location_amsl_pub = rospy.Publisher(NAVPOSE_CURRENT_LOCATION_AMSL_TOPIC, Float64MultiArray , queue_size=1)
navpose_location_wgs84_pub = rospy.Publisher(NAVPOSE_CURRENT_LOCATION_WGS84_TOPIC, Float64MultiArray , queue_size=1)
navpose_geoid_height_pub = rospy.Publisher(NAVPOSE_CURRENT_GEOID_HEIGHT_TOPIC, Float64, queue_size=1)
navpose_pub_interval_sec = float(1.0)/NAVPOSE_PUB_RATE_HZ

mavros_global_check=None
mavros_heading_check=None
mavros_orientation_check=None

#####################################################################################
# Methods
#####################################################################################

### System Initialization processes
def initialize_actions():
  global mavros_global_check
  global mavros_heading_check
  global mavros_orientation_check

  ## Start Check Heading Callback
  print("Starting heading check subscriber callback")
  print(MAVROS_HEADING_TOPIC)
  rospy.Subscriber(MAVROS_HEADING_TOPIC, Float64, check_heading_callback)
  while mavros_heading_check is None:
    print("Waiting for mavros heading data to publish")
    time.sleep(0.5)
  # Update Heading source
  heading_pub = rospy.Publisher(NEPI_SET_NAVPOSE_HEADING_TOPIC, String, queue_size=10)
  rospy.sleep(1) # VERY IMPORTANT - Sleep a bit between declaring a publisher and using it subscribers have time to subscribe
  heading_pub.publish(MAVROS_HEADING_TOPIC)
  
  ## Start Check Global Callback
  print("Starting global check subscriber callback")
  print(MAVROS_POSITION_GLOBAL_TOPIC)
  rospy.Subscriber(MAVROS_POSITION_GLOBAL_TOPIC, NavSatFix, check_global_callback)
  while mavros_global_check is None:
    print("Waiting for mavros global data to publish")
    time.sleep(0.5)  
  # Update GPS source
  gps_pub = rospy.Publisher(NEPI_SET_NAVPOSE_GPS_TOPIC, String, queue_size=10)
  rospy.sleep(1) # VERY IMPORTANT - Sleep a bit between declaring a publisher and using it subscribers have time to subscribe
  gps_pub.publish(MAVROS_POSITION_GLOBAL_TOPIC)

  ## Start Check Orientation Callback
  print("Starting orientation check subscriber callback")
  print(MAVROS_ORIENTATION_TOPIC)
  rospy.Subscriber(MAVROS_ORIENTATION_TOPIC, Odometry, check_orientation_callback)
  while mavros_orientation_check is None:
    print("Waiting for mavros orientation data to publish")
    time.sleep(0.5)  
  # Update Orientation source
  orientation_pub = rospy.Publisher(NEPI_SET_NAVPOSE_ORIENTATION_TOPIC, String, queue_size=10)
  rospy.sleep(1) # VERY IMPORTANT - Sleep a bit between declaring a publisher
  orientation_pub.publish(MAVROS_ORIENTATION_TOPIC)

  # Sync NEPI clock to GPS timestamp
  gps_timesync_pub = rospy.Publisher(NEPI_ENABLE_NAVPOSE_GPS_CLOCK_SYNC_TOPIC, Bool, queue_size=10)
  rospy.sleep(1) # VERY IMPORTANT - Sleep a bit between declaring a publisher
  gps_timesync_pub.publish(data=True)
  
  print("Initialization Complete")


### Callback to check mavros global data
def check_global_callback(global_msg):
  global mavros_global_check
  mavros_global_check = global_msg

### Callback to check mavros heading data
def check_heading_callback(heading_msg):
  global mavros_heading_check
  mavros_heading_check = heading_msg

### Callback to check mavros orientation data
def check_orientation_callback(orientation_msg):
  global mavros_orientation_check
  mavros_orientation_check = orientation_msg
 
### Setup a regular background navpose get and publish timer callback
def navpose_get_publish_callback(timer):
  global navpose_heading_pub 
  global navpose_oreantation_ned_pub 
  global navpose_oreantation_enu_pub 
  global navpose_position_ned_pub 
  global navpose_position_enu_pub 
  global navpose_location_amsl_pub
  global navpose_location_wgs84_pub
  global navpose_geoid_height_pub
  if not rospy.is_shutdown():
    # Get current NEPI NavPose data from NEPI ROS nav_pose_query service call
    rospy.wait_for_service(NAVPOSE_SERVICE_NAME)
    try:
      get_navpose_service = rospy.ServiceProxy(NAVPOSE_SERVICE_NAME, NavPoseQuery)
      nav_pose_response = get_navpose_service(NavPoseQueryRequest())
      #print(nav_pose_response)
      
      # Set current heading in degrees
      current_heading_deg = nav_pose_response.nav_pose.heading.heading

      # Set current orientation vector (roll, pitch, yaw) in degrees enu frame
      pose_enu_o = nav_pose_response.nav_pose.odom.pose.pose.orientation
      xyzw_enu_o = list([pose_enu_o.x,pose_enu_o.y,pose_enu_o.z,pose_enu_o.w])
      rpy_enu_d = convert_quat2rpy(xyzw_enu_o)
      current_orientation_enu_degs = Float64MultiArray()
      current_orientation_enu_degs.data = [rpy_enu_d[0], rpy_enu_d[1], rpy_enu_d[2]]

      # Set current orientation vector (roll, pitch, yaw) in degrees ned frame
      pose_ned_o = nav_pose_response.nav_pose.odom.pose.pose.orientation
      xyzw_ned_o = list([pose_ned_o.x,pose_ned_o.y,pose_ned_o.z,pose_ned_o.w])
      rpy_ned_d = convert_quat2rpy(xyzw_ned_o)
      yaw_ned_deg = -rpy_ned_d[2] + 90
      if yaw_ned_deg < 0:
        yaw_ned_deg = 360 + yaw_ned_deg
      elif yaw_ned_deg > 360:
        yaw_ned_deg = yaw_ned_deg - 360
      rpy_ned_d[2] = yaw_ned_deg
      current_orientation_ned_degs = Float64MultiArray()
      current_orientation_ned_degs.data = [rpy_ned_d[0], rpy_ned_d[1], rpy_ned_d[2]]

      # Set current position vector (x, y, z) in meters enu frame
      pose_enu_p = nav_pose_response.nav_pose.odom.pose.pose.position
      current_position_enu_m = Float64MultiArray()
      current_position_enu_m.data = [pose_enu_p.x, pose_enu_p.y, pose_enu_p.z]

      # Set current position vector (x, y, z) in meters ned frame
      pose_ned_p = nav_pose_response.nav_pose.odom.pose.pose.position
      current_position_ned_m = Float64MultiArray()
      current_position_ned_m.data = [pose_ned_p.x, pose_ned_p.y, -pose_ned_p.z]

      # Set current location vector (lat, long, alt) in geopoint data with AMSL height
      pos_l = nav_pose_response.nav_pose.fix
      current_location_amsl_geo = Float64MultiArray()
      current_location_amsl_geo.data =  [pos_l.latitude,pos_l.longitude,pos_l.altitude]

      # Set current geoid heihgt
      geoid_height=CURRENT_GEOID_HEIGHT_M
      current_geoid_height =  geoid_height

      # Set current location vector (lat, long, alt) in geopoint data with WGS84 height
      pos_l = nav_pose_response.nav_pose.fix
      current_location_wgs84_geo = Float64MultiArray()
      current_location_wgs84_geo.data =  [pos_l.latitude,pos_l.longitude,pos_l.altitude + geoid_height]

      # Publish new current navpose data
      navpose_heading_pub.publish(current_heading_deg)
      navpose_oreantation_ned_pub.publish(current_orientation_ned_degs)
      navpose_oreantation_enu_pub.publish(current_orientation_enu_degs)
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

  ### Cleanup processes on node shutdown
def cleanup_actions():
  print("Shutting down: Executing script cleanup actions")
  time.sleep(2)

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

