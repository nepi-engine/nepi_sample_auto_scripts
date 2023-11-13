#!/usr/bin/env python

# Sample NEPI Automation Script.
# Uses onboard ROS python and mavros libraries to
# 1) Publishes a fake GPS MAVLink Message
# 2) Accepts fake GPS updates from update topic
# 3) Accepts fake Pos movement from goto topic
# 4) Accepts fake Home movement from gohome topic

###################################################
### These Parameters Must Be Configured First
#GPS_TYPE = 14
#EK3_SRC1_POSZ = 3
#EK3_SRC1_VELZ = 0
#Optional
#BARO_OPTION = 1  (This was required for proper barometer reading on Pixhawk)
#####################################################

###################################################
### Local Position Updates X,Y,Z use these vehicle relative conventions
# x+ axis is forward
# y+ axis is right
# z+ axis is down
#####################################################

import rospy
import time
import numpy as np
import math

from std_msgs.msg import Empty, Float64, Header 
from mavros_msgs.msg import HilGPS
from geometry_msgs.msg import Point
from geographic_msgs.msg import GeoPoint

###################################################
# SETUP - Edit as Necessary 
##########################################

#Initial GPS Setup
LAT = 47.65412711862056
LONG = -122.31894922885307
ALT_METERS = 0.0
SAT_COUNT = 20
GPS_PUB_RATE_HZ = 50

#Vehicle Geo Position Update Controls
POS_UPDATE_STEPS = 100 # Number of moves to reach new position
POS_UPDATE_RATE_HZ = 10 # Rate that position updates are made

# ROS namespace setup
BASE_NAMESPACE = "/nepi/s2x/"
MAVROS_NAMESPACE = BASE_NAMESPACE + "pixhawk_mavlink/"
MAVROS_GLOBAL_HEADING_TOPIC = MAVROS_NAMESPACE + "global_position/compass_hdg"
MAVROS_TAKEOFF_SERVICE = MAVROS_NAMESPACE + "cmd/takeoff"
MAVROS_HILGPS_TOPIC = MAVROS_NAMESPACE + "hil/gps"
FAKE_GPS_UPDATE_GLOBAL_TOPIC = BASE_NAMESPACE + "fake_gps_update_global"
FAKE_GPS_GOTO_GLOBAL_TOPIC = BASE_NAMESPACE + "fake_gps_goto_global"
FAKE_GPS_GOTO_LOCAL_TOPIC = BASE_NAMESPACE + "fake_gps_goto_local"
FAKE_GPS_GOHOME_TOPIC = BASE_NAMESPACE + "fake_gps_gohome"

#####################################################################################
# Globals
#####################################################################################
fake_gps_mavlink_pub = rospy.Publisher(MAVROS_HILGPS_TOPIC, HilGPS, queue_size=1)
geopoint_home = None
geopoint_current = None
heading_deg_current = None

gps_publish_interval_sec=1.0/GPS_PUB_RATE_HZ
#####################################################################################
# Methods
##########f###########################################################################

### System Initialization processes
def initialize_actions():
  global geopoint_home
  global geopoint_current
  # Initialize Home Poistion
  geopoint_home=GeoPoint()
  geopoint_home.latitude = LAT
  geopoint_home.longitude = LONG
  geopoint_home.altitude = ALT_METERS
  # Initialize Current Poistion
  geopoint_current=GeoPoint()
  geopoint_current.latitude = LAT
  geopoint_current.longitude = LONG
  geopoint_current.altitude = ALT_METERS
  print("Initialization Complete")

### Callback to get update global heading
def get_update_heading_callback(heading_msg):
  global heading_deg_current
  heading_deg_current=heading_msg.data

### Callback to get and apply GPS global update
def fake_gps_update_global_callback(geopoint_msg):
  global geopoint_current
  # Replace Keep Current Flagged Values
  if geopoint_msg.latitude == -999.0:
    geopoint_msg.latitude=geopoint_current.latitude
  if geopoint_msg.longitude == -999.0:
    geopoint_msg.longitude=geopoint_current.longitude
  if geopoint_msg.altitude == -999.0:
    geopoint_msg.altitude=geopoint_current.altitude
  print("Recieved Fake GPS Update Global Message")
  print(geopoint_msg)
  geopoint_current=geopoint_msg

### Callback to get and move to new global geo position
def fake_gps_goto_global_callback(geopoint_msg):
  global geopoint_current
  # Replace Keep Current Flagged Values
  if geopoint_msg.latitude == -999.0:
    geopoint_msg.latitude=geopoint_current.latitude
  if geopoint_msg.longitude == -999.0:
    geopoint_msg.longitude=geopoint_current.longitude
  if geopoint_msg.altitude == -999.0:
    geopoint_msg.altitude=geopoint_current.altitude
  print("Recieved Fake GPS Goto Global Message")
  print(geopoint_msg)
  fake_gps_sim_move(geopoint_msg)

### Callback to get and move to new local geo position
def fake_gps_goto_local_callback(point_msg):
  global geopoint_current
  global heading_deg_current
  print("Recieved Fake Local X,Y,Z Message")
  print(point_msg)
  # Calculate new global lat long position from realtive xy
  cur_lat=geopoint_current.latitude
  cur_long=geopoint_current.longitude
  local_dist_km=math.sqrt(point_msg.x**2+point_msg.y**2)/1000
  local_angle_deg=90-(math.atan2(point_msg.x,point_msg.y)/math.pi*180)
  global_bearing_deg=heading_deg_current+local_angle_deg
  latlong_new = get_point_at_distance(cur_lat,cur_long,local_dist_km,global_bearing_deg)
  # Create updated global GeoPoint
  geopoint_new=GeoPoint()
  geopoint_new.latitude = latlong_new[0]
  geopoint_new.longitude = latlong_new[1]
  geopoint_new.altitude = geopoint_current.altitude - point_msg.z
  print("Calculated NEW Fake GPS Global Position")
  print(geopoint_new)
  # Initiate Move to new point
  fake_gps_sim_move(geopoint_new)

### Callback to move to new geo position goto
def fake_gps_gohome_callback(Empty):
  global geopoint_home
  print("Recieved Fake GPS GoHome Message")
  print(geopoint_home)
  fake_gps_sim_move(geopoint_home)

### Function to simulate movement
def fake_gps_sim_move(geopoint_move):
  global geopoint_current
  print("  Moving from, to, delta")
  print(geopoint_current)
  print(geopoint_move)
  org_geo=np.array([geopoint_current.latitude, geopoint_current.longitude, geopoint_current.altitude])
  new_geo=np.array([geopoint_move.latitude, geopoint_move.longitude, geopoint_move.altitude])
  delta_geo = new_geo - org_geo
  print(delta_geo)
  ramp=np.hanning(POS_UPDATE_STEPS)
  ramp=ramp**2
  ramp_norm=ramp/np.sum(ramp)
  step_norm=np.zeros(len(ramp_norm))
  for ind, val in enumerate(ramp_norm):
    step_norm[ind]=np.sum(ramp_norm[0:ind])
  stp_interval_sec = 1.0/ float(POS_UPDATE_RATE_HZ)
  for ind, val in enumerate(step_norm):
    time.sleep(stp_interval_sec)
    cur_geo = org_geo + delta_geo * val
    geopoint_current.latitude = cur_geo[0]
    geopoint_current.longitude = cur_geo[1]
    geopoint_current.altitude = cur_geo[2]
    print("")
    print("Updated to")
    print(geopoint_current)
  return True

def get_point_at_distance(lat1, lon1, d_km, bearing_deg, R=6371):
  """
  lat: initial latitude, in degrees
  lon: initial longitude, in degrees
  d: target distance from initial in km
  bearing: (true) heading in degrees
  R: optional radius of sphere, defaults to mean radius of earth
  Returns new lat/lon coordinate {d}km from initial, in degrees
  """
  lat1 = math.radians(lat1)
  lon1 = math.radians(lon1)
  a = math.radians(bearing_deg)
  lat2 = math.asin(math.sin(lat1) * math.cos(d_km/R) + math.cos(lat1) * math.sin(d_km/R) * math.cos(a))
  lon2 = lon1 + math.atan2(
      math.sin(a) * math.sin(d_km/R) * math.cos(lat1),
      math.cos(d_km/R) - math.sin(lat1) * math.sin(lat2)
  )
  return (math.degrees(lat2), math.degrees(lon2),)    

### Setup a regular Send Fake GPS callback using current geo point value
def fake_gps_pub_callback(timer):
  global fake_gps_mavlink_pub
  global geopoint_current
  hilgps=HilGPS()
  hilgps.header = Header(stamp=rospy.Time.now(), frame_id="fake_gps")
  hilgps.fix_type=3
  hilgps.geo.latitude=geopoint_current.latitude
  hilgps.geo.longitude=geopoint_current.longitude
  hilgps.geo.altitude=geopoint_current.altitude
  hilgps.satellites_visible=SAT_COUNT
  ##  print("Created new HilGPS message")
  ##  print(hilgps)
  # Create and publish Fake GPS Publisher
  if not rospy.is_shutdown():
    hilgps.header = Header(stamp=rospy.Time.now(), frame_id="fake_gps")
    fake_gps_mavlink_pub.publish(hilgps)

### Print the current fake geopoint position at slower rate
def fake_gps_print_callback(timer):
  global geopoint_current
  print("")
  print("Current Geo Position")
  print(geopoint_current)

  ### Cleanup processes on node shutdown
def cleanup_actions():
  global fake_gps_mavlink_pub
  print("Shutting down: Executing script cleanup actions")
  # Stop geopoint publisher
  time.sleep(2)


### Script Entrypoint
def startNode():
  global gps_publish_interval_sec
  global geopoint_current
  global heading_deg_current
  rospy.loginfo("Starting Fake GPS Simulation automation script")
  rospy.init_node("mavros_fake_gps_sim_auto_script")
  #initialize system including pan scan process
  initialize_actions()
  #Start fake gps publish callback
  print("Starting fake gps publishing to " + MAVROS_HILGPS_TOPIC)
  rospy.Timer(rospy.Duration(gps_publish_interval_sec), fake_gps_pub_callback)
  #rospy.Timer(rospy.Duration(1.0), fake_gps_print_callback)
  #Start get update global heading callback
  rospy.Subscriber(MAVROS_GLOBAL_HEADING_TOPIC, Float64, get_update_heading_callback)
  #Start fake gps update global callback
  rospy.Subscriber(FAKE_GPS_UPDATE_GLOBAL_TOPIC, GeoPoint, fake_gps_update_global_callback)
  #Start fake gps goto global callback
  rospy.Subscriber(FAKE_GPS_GOTO_GLOBAL_TOPIC, GeoPoint, fake_gps_goto_global_callback)
  #Start fake gps goto global callback
  rospy.Subscriber(FAKE_GPS_GOTO_LOCAL_TOPIC, Point, fake_gps_goto_local_callback)
  #Start fake gps gohome callback
  rospy.Subscriber(FAKE_GPS_GOHOME_TOPIC, Empty, fake_gps_gohome_callback)
  # run cleanup actions on shutdown
  rospy.on_shutdown(cleanup_actions)
  # Spin forever
  rospy.spin()

#####################################################################################
# Main
#####################################################################################

if __name__ == '__main__':
  startNode()
