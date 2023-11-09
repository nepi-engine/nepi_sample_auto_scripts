#!/usr/bin/env python

# Sample NEPI Automation Script.
# Uses onboard ROS python and mavros libraries to
# 1. Publishes a fake GPS and Altitude MAVLink Message

import rospy
from std_msgs.msg import Float64, Header 
from mavros_msgs.msg import HilGPS
from geographic_msgs.msg import GeoPoint

#####################################################################################
# SETUP - Edit as Necessary ##################################
##########################################

#Numurus Office
LAT = 47.65412711862056
LONG = -122.31894922885307
ALT = 0.005
SAT_COUNT = 9
PUB_RATE_HZ = 1

# ROS namespace setup
BASE_NAMESPACE = "/nepi/s2x/"
MAVROS_NAMESPACE = BASE_NAMESPACE + "pixhawk_mavlink/"
MAVROS_HILGPS_TOPIC = MAVROS_NAMESPACE + "hil/gps"

#####################################################################################
# Methods
#####################################################################################

### Script Entrypoint
def startNode():
  rospy.init_node("mavros_fake_gps_altitude_auto_script")
  rospy.loginfo("Starting MAVROS Fake GPS automation script")
  # Build our fake GPS Message
  hilgps=HilGPS()
  hilgps.header = Header(stamp=rospy.Time.now(), frame_id="fake_gps")
  hilgps.fix_type=3
  hilgps.geo.latitude=LAT
  hilgps.geo.longitude=LONG
  hilgps.geo.altitude=ALT
  hilgps.satellites_visible=SAT_COUNT
  pub_interval=1.0/PUB_RATE_HZ
  print("Created new HilGPS message")
  print(hilgps)
  # Create Fake GPS Publisher
  print("Starting hil/gps ROS publisher")
  fake_gps_pub = rospy.Publisher(MAVROS_HILGPS_TOPIC, HilGPS, queue_size=10)
  rospy.sleep(1) # VERY IMPORTANT - Sleep a bit between declaring a publisher and using it subscribers have time to subscribe
  while not rospy.is_shutdown():
    hilgps.header = Header(stamp=rospy.Time.now(), frame_id="fake_gps")
    fake_gps_pub.publish(hilgps)
    rospy.sleep(pub_interval)


#####################################################################################
# Main
#####################################################################################

if __name__ == '__main__':
  startNode()
