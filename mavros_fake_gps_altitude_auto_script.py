#!/usr/bin/env python

# Sample NEPI Automation Script.
# Uses onboard ROS python and mavros libraries to
# 1) Publishes a fake GPS and Altitude MAVLink Message
# 2) Accepts fake GPS updates from update topic

###################################################
### These Parameters Must Be Configured First
#GPS_TYPE = 14
#EK3_SRC1_POSZ = 3
#Optional
#BARO_OPTION = 1  (This was required for proper barometer reading on Pixhawk)
#####################################################

import rospy
import time

from std_msgs.msg import Float64, Header 
from mavros_msgs.msg import HilGPS
from geographic_msgs.msg import GeoPoint

###################################################
# SETUP - Edit as Necessary 
##########################################

#Initial GPS Setup
LAT = 47.65412711862056
LONG = -122.31894922885307
ALT_METERS = 0.0
SAT_COUNT = 9
GPS_PUB_RATE_HZ = 10

# ROS namespace setup
BASE_NAMESPACE = "/nepi/s2x/"
MAVROS_NAMESPACE = BASE_NAMESPACE + "pixhawk_mavlink/"
MAVROS_HILGPS_TOPIC = MAVROS_NAMESPACE + "hil/gps"
FAKE_GPS_UPDATE_TOPIC = BASE_NAMESPACE + "fake_gps_update"

#####################################################################################
# Globals
#####################################################################################
fake_gps_update_pub = rospy.Publisher(FAKE_GPS_UPDATE_TOPIC, GeoPoint, queue_size=10)
fake_gps_mavlink_pub = rospy.Publisher(MAVROS_HILGPS_TOPIC, HilGPS, queue_size=1)
latest_lat = LAT
latest_long = LONG
latest_alt_m = ALT_METERS
gps_publish_interval_sec=1.0/GPS_PUB_RATE_HZ
#####################################################################################
# Methods
##########f###########################################################################

### System Initialization processes
def initialize_actions():
  global fake_gps_update_pub 
  # Publish Initial Geopoint data to Fake GPS Update Topic to create it
  geopoint_data_msg=GeoPoint()
  geopoint_data_msg.latitude=LAT
  geopoint_data_msg.longitude=LONG
  geopoint_data_msg.altitude=ALT_METERS
  for i in range(10):
    fake_gps_update_pub.publish(geopoint_data_msg)
    time.sleep(0.1)
  print("Completed Initialization")

### Callback to get fake gps updates
def fake_gps_update_callback(geopoint_update_msg):
  global latest_lat
  global latest_long
  global latest_alt_m
  print("Recieved Fake GPS Update")
  print(geopoint_update_msg)
  latest_lat=geopoint_update_msg.latitude
  latest_long=geopoint_update_msg.longitude
  latest_alt_m=geopoint_update_msg.altitude

### Setup a regular Send Fake GPS callback
def fake_gps_pub_callback(timer):
  global fake_gps_mavlink_pub
  global latest_lat
  global latest_long
  global latest_alt
  hilgps=HilGPS()
  hilgps.header = Header(stamp=rospy.Time.now(), frame_id="fake_gps")
  hilgps.fix_type=3
  hilgps.geo.latitude=latest_lat
  hilgps.geo.longitude=latest_long
  hilgps.geo.altitude=latest_alt_m
  hilgps.satellites_visible=SAT_COUNT
  ##  print("Created new HilGPS message")
  ##  print(hilgps)
  # Create and publish Fake GPS Publisher
  if not rospy.is_shutdown():
    hilgps.header = Header(stamp=rospy.Time.now(), frame_id="fake_gps")
    fake_gps_mavlink_pub.publish(hilgps)

  ### Cleanup processes on node shutdown
def cleanup_actions():
  global fake_gps_mavlink_pub
  print("Shutting down: Executing script cleanup actions")
  # Stop geopoint publisher
  fake_gps_mavlink_pub.unregister()
  time.sleep(2)


### Script Entrypoint
def startNode():
  global gps_publish_interval_sec
  rospy.loginfo("Starting Fake GPS automation script")
  rospy.init_node("mavros_fake_gps_auto_script")
  #initialize system including pan scan process
  initialize_actions()
  #Start fake gps update callback
  rospy.Subscriber(FAKE_GPS_UPDATE_TOPIC, GeoPoint, fake_gps_update_callback)
  #Start fake gps publish callback
  print("Starting fake gps publishing to " + MAVROS_HILGPS_TOPIC)
  rospy.Timer(rospy.Duration(gps_publish_interval_sec), fake_gps_pub_callback)
  # run cleanup actions on shutdown
  rospy.on_shutdown(cleanup_actions)
  # Spin forever
  rospy.spin()

#####################################################################################
# Main
#####################################################################################

if __name__ == '__main__':
  startNode()
