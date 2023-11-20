#!/usr/bin/env python

# Sample NEPI Automation Script.
# Uses onboard ROS python and mavros libraries to
# 1) Publishes a fake GPS MAVLink Message
# 2) Accepts fake GPS updates from update topic
# 3) Accepts fake Pos movement from goto topic
# 4) Accepts fake Home movement from gohome topic

# Expects "navpose_get_and_publish_auto_script.py" to be running

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

###################################################
### Local Position (NED Frame) Updates X,Y,Z use these vehicle relative conventions
# x+ axis is forward
# y+ axis is right
# z+ axis is down
#####################################################

import rospy
import time
import numpy as np
import math

from std_msgs.msg import Empty, Float64, Header 
from mavros_msgs.msg import HilGPS, State
from geometry_msgs.msg import Point
from geographic_msgs.msg import GeoPoint

###################################################
# SETUP - Edit as Necessary 
##########################################

#Startup Home Position
LAT = 47.65412711862056
LONG = -122.31894922885307
ALT_METERS = 0.0

#GPS Setup
SAT_COUNT = 20
GPS_PUB_RATE_HZ = 20

#GPS Simulation Position Update Controls
#Adjust these settings for smoother xyz movements
POS_UPDATE_TIME_SEC=10
POS_UPDATE_STEPS = 100 # Number of moves to reach new position

PRINT_STATE_POSITION_1HZ = False # Print State and Position data at 1Hz

# ROS namespace setup
BASE_NAMESPACE = "/nepi/s2x/"
MAVROS_NAMESPACE = BASE_NAMESPACE + "pixhawk_mavlink/"
MAVROS_FAKE_GPS_NAMESPACE = MAVROS_NAMESPACE + "fake_gps/"

# MAVROS Topics
MAVROS_HILGPS_TOPIC = MAVROS_NAMESPACE + "hil/gps"
MAVROS_FAKE_GPS_SETHOME_TOPIC = MAVROS_FAKE_GPS_NAMESPACE + "sethome"
MAVROS_FAKE_GPS_TAKEOFF_TOPIC = MAVROS_FAKE_GPS_NAMESPACE + "takeoff"
MAVROS_FAKE_GPS_UPDATE_GLOBAL_TOPIC = MAVROS_FAKE_GPS_NAMESPACE + "update_global"
MAVROS_FAKE_GPS_GOTO_LOCAL_TOPIC = MAVROS_FAKE_GPS_NAMESPACE + "goto_local"
MAVROS_FAKE_GPS_GOTO_GLOBAL_TOPIC = MAVROS_FAKE_GPS_NAMESPACE + "goto_global"
MAVROS_FAKE_GPS_GOHOME_TOPIC = MAVROS_FAKE_GPS_NAMESPACE + "gohome"
MAVROS_FAKE_GPS_LAND_TOPIC = MAVROS_FAKE_GPS_NAMESPACE + "land"

### Mavros Heading Subscriber Topics
MAVROS_HEADING_TOPIC = MAVROS_NAMESPACE + "global_position/compass_hdg"

#####################################################################################
# Globals
#####################################################################################
mavros_fake_gps_mavlink_pub = rospy.Publisher(MAVROS_HILGPS_TOPIC, HilGPS, queue_size=1)
current_home_geo = None
current_location_geo = None
current_heading_deg = None
gps_publish_interval_sec=1.0/GPS_PUB_RATE_HZ
#####################################################################################
# Methods
##########f###########################################################################

### System Initialization processes
def initialize_actions():
  global current_home_geo
  global current_location_geo
  global current_heading_deg
  #Start get and update current navpose data 
  rospy.Subscriber(MAVROS_HEADING_TOPIC, Float64, update_heading_callback)
  print("Waiting for heading topic to publish on")
  print(MAVROS_HEADING_TOPIC)
  while current_heading_deg is None:
    print("Waiting for heading topic to publish on")
    time.sleep(1)
  #Start fake gps update global subscriber
  rospy.Subscriber(MAVROS_FAKE_GPS_UPDATE_GLOBAL_TOPIC, GeoPoint, mavros_fake_gps_update_global_callback)
  #Start fake gps goto global subscriber
  rospy.Subscriber(MAVROS_FAKE_GPS_GOTO_GLOBAL_TOPIC, GeoPoint, mavros_fake_gps_goto_global_callback)
  #Start fake gps goto global subscriber
  rospy.Subscriber(MAVROS_FAKE_GPS_GOTO_LOCAL_TOPIC, Point, mavros_fake_gps_goto_local_callback)
  #Start fake gps takeoff subscriber
  rospy.Subscriber(MAVROS_FAKE_GPS_TAKEOFF_TOPIC, Float64, mavros_fake_gps_takeoff_callback)
  #Start fake gps gohome subscriber
  rospy.Subscriber(MAVROS_FAKE_GPS_GOHOME_TOPIC, Empty, mavros_fake_gps_gohome_callback)
  #Start fake gps land subscriber
  rospy.Subscriber(MAVROS_FAKE_GPS_LAND_TOPIC, Empty, mavros_fake_gps_land_callback)
  #Start fake gps sethome subscriber
  rospy.Subscriber(MAVROS_FAKE_GPS_SETHOME_TOPIC, GeoPoint, mavros_fake_gps_sethome_callback)
  # Initialize Home Poistion
  current_home_geo=GeoPoint()
  current_home_geo.latitude = LAT
  current_home_geo.longitude = LONG
  current_home_geo.altitude = ALT_METERS
  # Initialize Current Poistion
  current_location_geo=GeoPoint()
  current_location_geo.latitude = LAT
  current_location_geo.longitude = LONG
  current_location_geo.altitude = ALT_METERS
  print("Initialization Complete")


### Setup a regular Send Fake GPS callback using current geo point value
def mavros_fake_gps_pub_callback(timer):
  global mavros_fake_gps_mavlink_pub
  global current_location_geo
  hilgps=HilGPS()
  hilgps.header = Header(stamp=rospy.Time.now(), frame_id="mavros_fake_gps")
  hilgps.fix_type=3
  hilgps.geo.latitude=current_location_geo.latitude
  hilgps.geo.longitude=current_location_geo.longitude
  hilgps.geo.altitude=current_location_geo.altitude
  hilgps.satellites_visible=SAT_COUNT
  ##print("Created new HilGPS message")
  ##print(hilgps)
  # Create and publish Fake GPS Publisher
  hilgps.header = Header(stamp=rospy.Time.now(), frame_id="mavros_fake_gps")
  if not rospy.is_shutdown():
    mavros_fake_gps_mavlink_pub.publish(hilgps)

### Callback to get update global heading
def update_heading_callback(heading_msg):
  global current_heading_deg
  current_heading_deg=heading_msg.data

### Callback to get and apply GPS global update
def mavros_fake_gps_update_global_callback(geopoint_msg):
  global current_location_geo
  print('')
  print("Recieved Fake GPS Update Global Message")
  print(geopoint_msg)
  current_location_geo=geopoint_msg

### Callback to get and move to new global geo position
def mavros_fake_gps_goto_global_callback(geopoint_msg):
  global current_location_geo
  print('')
  print("Recieved Fake GPS Goto Global Message")
  print(geopoint_msg)
  mavros_fake_gps_sim_move(geopoint_msg)

### Callback to get and move to new local geo position
def mavros_fake_gps_goto_local_callback(point_msg):
  global current_location_geo
  global current_heading_deg
  print('')
  print("Recieved Fake Local X,Y,Z Message")
  print(point_msg)
  # Calculate new global lat long position from realtive xy
  cur_lat=current_location_geo.latitude
  cur_long=current_location_geo.longitude
  local_dist_km=math.sqrt(point_msg.x**2+point_msg.y**2)/1000
  local_angle_deg=90-(math.atan2(point_msg.x,point_msg.y)/math.pi*180)
  global_bearing_deg= current_heading_deg + local_angle_deg
  latlong_new = get_point_at_distance(cur_lat,cur_long,local_dist_km,global_bearing_deg)
  # Create updated global GeoPoint
  geopoint_new=GeoPoint()
  geopoint_new.latitude = latlong_new[0]
  geopoint_new.longitude = latlong_new[1]
  geopoint_new.altitude = current_location_geo.altitude + point_msg.z
  print("Calculated NEW Fake GPS Global Position")
  print(geopoint_new)
  # Initiate Move to new point
  mavros_fake_gps_sim_move(geopoint_new)

### Callback to takeoff from current lat and long
def mavros_fake_gps_takeoff_callback(alt_m):
  print('takeoff alt')
  global current_location_geo
  geopoint_takeoff = GeoPoint()
  geopoint_takeoff.latitude = current_location_geo.latitude
  geopoint_takeoff.longitude = current_location_geo.longitude
  geopoint_takeoff.altitude = alt_m.data
  print('')
  print("Recieved Fake GPS Takeoff Message")
  print(geopoint_takeoff)
  mavros_fake_gps_sim_move(geopoint_takeoff)

### Callback to move to home position
def mavros_fake_gps_gohome_callback(Empty):
  global current_home_geo
  print('')
  print("Recieved Fake GPS GoHome Message")
  print(current_home_geo)
  mavros_fake_gps_sim_move(current_home_geo)

### Callback to land at current lat and long
def mavros_fake_gps_land_callback(Empty):
  global current_location_geo
  geopoint_land = GeoPoint()
  geopoint_land.latitude = current_location_geo.latitude
  geopoint_land.longitude = current_location_geo.longitude
  geopoint_land.altitude = 0.0
  print('')
  print("Recieved Fake GPS Land Message")
  print(geopoint_land)
  mavros_fake_gps_sim_move(geopoint_land)

### Callback to set new home position
def mavros_fake_gps_sethome_callback(geopoint_msg):
  global current_home_geo
  print('')
  print("Recieved Fake GPS Set Home Message")
  print(geopoint_msg)
  current_home_geo=geopoint_msg

### Function to simulate movement
def mavros_fake_gps_sim_move(geopoint_move):
  global current_location_geo
  print('')
  print('')
  print("Moving FROM, TO, DELTA")
  print(current_location_geo)
  print('')
  print(geopoint_move)
  org_geo=np.array([current_location_geo.latitude, current_location_geo.longitude, current_location_geo.altitude])
  new_geo=np.array([geopoint_move.latitude, geopoint_move.longitude, geopoint_move.altitude])
  delta_geo = new_geo - org_geo
  print('')
  print(delta_geo)
  ramp=np.hanning(POS_UPDATE_STEPS)
  ramp=ramp**2
  ramp_norm=ramp/np.sum(ramp)
  step_norm=np.zeros(len(ramp_norm))
  for ind, val in enumerate(ramp_norm):
    step_norm[ind]=np.sum(ramp_norm[0:ind])
  stp_interval_sec = float(POS_UPDATE_TIME_SEC)/float(POS_UPDATE_STEPS)
  for ind, val in enumerate(step_norm):
    time.sleep(stp_interval_sec)
    cur_geo_step = delta_geo * val
    cur_geo = org_geo + cur_geo_step
    current_location_geo.latitude = cur_geo[0]
    current_location_geo.longitude = cur_geo[1]
    current_location_geo.altitude = cur_geo[2]
    print("")
    print("Updated to")
    print(current_location_geo)
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




 

### Print the current fake geopoint position at slower rate
def mavros_fake_gps_print_callback(timer):
  global current_location_geo
  print('')
  print("Current Geo Position")
  print(current_location_geo)
  

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
  rospy.init_node("mavros_mavros_fake_gps_sim_auto_script")
  #initialize system including pan scan process
  initialize_actions()
  #Start fake gps publish and print callbacks
  print("Starting fake gps publishing to " + MAVROS_HILGPS_TOPIC)
  rospy.Timer(rospy.Duration(gps_publish_interval_sec), mavros_fake_gps_pub_callback)
  if PRINT_STATE_POSITION_1HZ:
    rospy.Timer(rospy.Duration(1.0), mavros_fake_gps_print_callback)
  # run cleanup actions on shutdown
  rospy.on_shutdown(cleanup_actions)
  # Spin forever
  rospy.spin()

#####################################################################################
# Main
#####################################################################################

if __name__ == '__main__':
  startNode()
