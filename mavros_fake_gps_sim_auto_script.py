#!/usr/bin/env python

__author__ = "Jason Seawall"
__copyright__ = "Copyright 2023, Numurus LLC"
__email__ = "nepi@numurus.com"
__credits__ = ["Jason Seawall", "Josh Maximoff"]

__license__ = "GPL"
__version__ = "2.0.4.0"


# Sample NEPI Automation Script.
# Uses onboard ROS python and mavros libraries to
# 1) Publishes a fake GPS MAVLink Message 
# Provides two ROS control topics
# a) goto_geopoint_wgs84 - Simulates move to new geopoint
# b) reset_geopoint_wgs84 - Resets GPS and global x,y NED home position at new geopoint
# c) reset_current - Resets GPS and global x,y NED home position at current geopoint
# Fake GPS control messages take a GeoPoint with WGS84 Height for Atlitude

# Requires the following additional scripts are running
# a) navpose_config_and_publish_auto_script.py
# These scripts are available for download at:
# [link text](https://github.com/numurus-nepi/nepi_sample_auto_scripts)

###################################################
### These Parameters Must Be Configured First to allow MAVLINK GPS source
### There maybe better configuration options, lots of knobs to play with
#GPS_TYPE = 14
#GPS_DELAY_MS = 1
#EK3_POS_I_GATE = 300
#EK3_POSNE_M_NSE = 5
#EK3_SRC_OPTIONS = 0
#EK3_SRC1_POSXY = 3
#EK3_SRC1_POSZ = 3
#EK3_SRC1_VELXY = 3
#EK3_SRC1_VELZ = 3
#EK3_SRC1_YAW = 1
#BARO_OPTION = 1  (This was required for proper barometer reading on Pixhawk)
#####################################################


import rospy
import time
import numpy as np
import math

from std_msgs.msg import Empty, Float64, Float64MultiArray, Header
from mavros_msgs.msg import HilGPS, State
from geometry_msgs.msg import Point
from geographic_msgs.msg import GeoPoint
from mavros_msgs.srv import CommandHome

###################################################
# SETUP - Edit as Necessary 
##########################################

#Startup Location
LAT = 47.65412711862056
LONG = -122.31894922885307
ALT_WSG84_METERS = 0.0

#GPS Setup
SAT_COUNT = 20
GPS_PUB_RATE_HZ = 20

#GPS Simulation Position Update Controls
#Adjust these settings for smoother xyz movements
MOVE_UPDATE_TIME_SEC=10
MOVE_UPDATE_STEPS = 100 # Number of moves to reach new position

PRINT_STATE_POSITION_1HZ = False # Print State and Position data at 1Hz

# ROS namespace setup
NEPI_BASE_NAMESPACE = "/nepi/s2x/"
MAVROS_NAMESPACE = NEPI_BASE_NAMESPACE + "pixhawk_mavlink/"

###NEPI NavPose Heading, Oreanation, Location, and Position Subscribe Topics
NAVPOSE_CURRENT_HEADING_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose_current/heading_deg"
NAVPOSE_CURRENT_ORIENTATION_NED_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose_current/orientation_ned_degs"
NAVPOSE_CURRENT_ORIENTATION_ENU_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose_current/orientation_enu_degs"
NAVPOSE_CURRENT_POSITION_NED_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose_current/position_ned_m"
NAVPOSE_CURRENT_POSITION_ENU_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose_current/position_enu_m"
NAVPOSE_CURRENT_LOCATION_AMSL_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose_current/location_amsl_geo"
NAVPOSE_CURRENT_LOCATION_WGS84_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose_current/location_wgs84_geo"
NAVPOSE_CURRENT_GEOID_HEIGHT_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose_current/geoid_height_m"


# MAVROS Fake GPS Publish Topic
MAVROS_HILGPS_TOPIC = MAVROS_NAMESPACE + "hil/gps"

# MAVROS Fake GPS Control Topic
MAVROS_FAKE_GPS_GOTO_GEOPOINT_TOPIC = MAVROS_NAMESPACE + "fake_gps/goto_geopoint_wgs84"
MAVROS_FAKE_GPS_RESET_GEOPOINT_TOPIC = MAVROS_NAMESPACE + "fake_gps/reset_geopoint_wgs84"
MAVROS_FAKE_GPS_RESET_CURRENT_TOPIC = MAVROS_NAMESPACE + "fake_gps/reset_current"



#####################################################################################
# Globals
#####################################################################################
mavros_fake_gps_mavlink_pub = rospy.Publisher(MAVROS_HILGPS_TOPIC, HilGPS, queue_size=1)
current_location_wgs84_geo = None
fake_gps_enabled = True
gps_publish_interval_sec=1.0/GPS_PUB_RATE_HZ
#####################################################################################
# Methods
##########f###########################################################################

### System Initialization processes
def initialize_actions():
  global current_location_wgs84_geo
  # Initialize Current Location
  current_location_wgs84_geo=GeoPoint()
  current_location_wgs84_geo.latitude = LAT
  current_location_wgs84_geo.longitude = LONG
  current_location_wgs84_geo.altitude = ALT_WSG84_METERS
  #Start fake gps subscribers
  rospy.Subscriber(MAVROS_FAKE_GPS_GOTO_GEOPOINT_TOPIC, GeoPoint, mavros_fake_gps_goto_geopoint_callback)
  rospy.Subscriber(MAVROS_FAKE_GPS_RESET_GEOPOINT_TOPIC, GeoPoint, mavros_fake_gps_reset_geopoint_callback)
  rospy.Subscriber(MAVROS_FAKE_GPS_RESET_CURRENT_TOPIC, Empty, mavros_fake_gps_reset_current_callback)
  # Start Print Callback if Enabled
  if PRINT_STATE_POSITION_1HZ:
    rospy.Timer(rospy.Duration(1.0), mavros_fake_gps_print_callback)
  print("Initialization Complete")

### Setup a regular Send Fake GPS callback using current geo point value
def mavros_fake_gps_pub_callback(timer):
  global mavros_fake_gps_mavlink_pub
  global current_location_wgs84_geo
  global fake_gps_enabled
  if fake_gps_enabled:
    hilgps=HilGPS()
    hilgps.header = Header(stamp=rospy.Time.now(), frame_id="mavros_fake_gps")
    hilgps.fix_type=3
    hilgps.geo.latitude=current_location_wgs84_geo.latitude
    hilgps.geo.longitude=current_location_wgs84_geo.longitude
    hilgps.geo.altitude=current_location_wgs84_geo.altitude
    hilgps.satellites_visible=SAT_COUNT
    ##print("Created new HilGPS message")
    ##print(hilgps)
    # Create and publish Fake GPS Publisher
    hilgps.header = Header(stamp=rospy.Time.now(), frame_id="mavros_fake_gps")
    if not rospy.is_shutdown():
      mavros_fake_gps_mavlink_pub.publish(hilgps)


### Callback to simulate move to new global geo position
def mavros_fake_gps_goto_geopoint_callback(geopoint_msg):
  global current_location_wgs84_geo
  print('')
  print('***********************')
  print("Recieved Fake GPS Goto Message")
  print('***********************')
  print('')
  print('***********************')
  print("Moving FROM, TO, DELTA")
  print('***********************')
  print(current_location_wgs84_geo)
  print('')
  print(geopoint_msg)
  org_geo=np.array([current_location_wgs84_geo.latitude, current_location_wgs84_geo.longitude, current_location_wgs84_geo.altitude])
  new_geo=np.array([geopoint_msg.latitude, geopoint_msg.longitude, geopoint_msg.altitude])
  for ind, val in enumerate(new_geo):
    if new_geo[ind] == -999.0: # Use current
      new_geo[ind]=org_geo[ind]
  delta_geo = new_geo - org_geo
  print('')
  print(delta_geo)
  ramp=np.hanning(MOVE_UPDATE_STEPS)
  ramp=ramp**2
  ramp_norm=ramp/np.sum(ramp)
  step_norm=np.zeros(len(ramp_norm))
  for ind, val in enumerate(ramp_norm):
    step_norm[ind]=np.sum(ramp_norm[0:ind])
  stp_interval_sec = float(MOVE_UPDATE_TIME_SEC)/float(MOVE_UPDATE_STEPS)
  for ind, val in enumerate(step_norm):
    time.sleep(stp_interval_sec)
    cur_geo_step = delta_geo * val
    cur_geo = org_geo + cur_geo_step
    current_location_wgs84_geo.latitude = cur_geo[0]
    current_location_wgs84_geo.longitude = cur_geo[1]
    current_location_wgs84_geo.altitude = cur_geo[2]
    print("")
    print("Updated to")
    print(current_location_wgs84_geo)
  print('')
  print('***********************')
  print("Moving Complete")
  print('***********************')





### Callback to Reset GPS position and global x,y NED home position
def mavros_fake_gps_reset_geopoint_callback(geopoint_msg):
  global current_location_wgs84_geo
  print('')
  print('***********************')
  print("Recieved Fake GPS Reset GeoPoint Message")
  print('***********************')
  print(geopoint_msg)
  org_geo=np.array([current_location_wgs84_geo.latitude, current_location_wgs84_geo.longitude, current_location_wgs84_geo.altitude])
  new_geo=np.array([geopoint_msg.latitude, geopoint_msg.longitude, geopoint_msg.altitude])
  for ind, val in enumerate(new_geo):
    if new_geo[ind] == -999.0: # Use current
      new_geo[ind]=org_geo[ind]
  current_location_wgs84_geo.latitude=new_geo[0]
  current_location_wgs84_geo.longitude=new_geo[1]
  current_location_wgs84_geo.altitude=new_geo[2]
  time.sleep(1)
  print("Reseting GPS with new location")
  reset_gps()
  print("Reset Complete")


  ### Callback to Reset GPS position and global x,y NED home position
def mavros_fake_gps_reset_current_callback(empty_msg):
  print('')
  print('***********************')
  print("Recieved Fake GPS Reset Current Message")
  print('***********************')
  print("Reseting GPS with current location")
  reset_gps()
  print("Reset Complete")

### Function to reset gps and wait for position ned x,y to reset
def reset_gps():
  global current_position_ned_m
  global fake_gps_enabled
  fake_gps_enabled = False
  print("Waiting for GPS to reset")  
  fake_gps_enabled = False
  time.sleep(MOVE_UPDATE_TIME_SEC)
  fake_gps_enabled = True


### Callback to print the current fake geopoint position at slower rate
def mavros_fake_gps_print_callback(timer):
  global current_location_wgs84_geo
  print('')
  print("Current WGS84 Geo Location ")
  print(current_location_wgs84_geo)

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
  global mavros_fake_gps_mavlink_pub
  print("Shutting down: Executing script cleanup actions")
  # Stop geopoint publisher
  time.sleep(2)


### Script Entrypoint
def startNode():
  global gps_publish_interval_sec
  rospy.loginfo("Starting Fake GPS Simulation automation script")
  rospy.init_node("mavros_fake_gps_sim_auto_script")
  #initialize system including pan scan process
  initialize_actions()
  #Start fake gps publish and print callbacks
  print("Starting fake gps publishing to " + MAVROS_HILGPS_TOPIC)
  rospy.Timer(rospy.Duration(gps_publish_interval_sec), mavros_fake_gps_pub_callback)
  # run cleanup actions on shutdown
  rospy.on_shutdown(cleanup_actions)
  # Spin forever
  rospy.spin()

#####################################################################################
# Main
#####################################################################################

if __name__ == '__main__':
  startNode()
