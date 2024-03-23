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
# 1) Publishes a fake GPS MAVLink Message 
# Provides two ROS control topics
# a) goto_geopoint_wgs84 - Simulates move to new geopoint
# b) reset_geopoint_wgs84 - Resets GPS and global x,y NED home position at new geopoint
# c) reset_current - Resets GPS and global x,y NED home position at current geopoint
# d) subscribe to RBX command topics and apply position moves
# Fake GPS control messages take a GeoPoint with WGS84 Height for Atlitude

# Requires the following additional scripts are running
# a) None
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
import rosnode
import time
import sys
import numpy as np
import math
import random
from resources import nepi
from resources import nepi_navpose

from std_msgs.msg import Empty, UInt8, Int8, Float32, Float64, Float64MultiArray, Header
from mavros_msgs.msg import HilGPS, State
from geometry_msgs.msg import Point
from geographic_msgs.msg import GeoPoint
from mavros_msgs.srv import CommandHome
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped
from nepi_ros_interfaces.srv import NavPoseQuery, NavPoseQueryRequest

#########################################
# USER SETTINGS - Edit as Necessary 
#########################################

#Startup Location
# [Lat, Long, Altitude_WGS84]
FAKE_GPS_START_GEOPOINT_WGS84 = [47.6540828,-122.3187578,0.0]
TAKEOFF_M = 10 # updated by RBX driver

#GPS Setup
SAT_COUNT = 20
GPS_PUB_RATE_HZ = 50

#GPS Simulation Position Update Controls
#Adjust these settings for smoother xyz movements
MOVE_UPDATE_TIME_SEC_PER_METER=1

PRINT_LOCATION_1HZ = False # rospy.loginfo current location at 1Hz


#########################################
# ROS NAMESPACE SETUP
#########################################

# ROS namespace setup
NEPI_BASE_NAMESPACE = nepi.get_base_namespace()

#########################################
# Node Class
#########################################

class ardupilot_rbx_fake_gps_process(object):

  #######################
  ### Node Initialization
  def __init__(self):
    rospy.loginfo("Starting Initialization Processes")
    ## Initialize Class Variables
    # RBX State and Mode Dictionaries
    self.RBX_MODE_FUNCTIONS = ["stabilize","land","rtl","loiter","guided","resume"]
    self.RBX_ACTION_FUNCTIONS = ["takeoff"]
    self.current_location_wgs84_geo = None
    self.current_heading_deg = None
    self.current_home_wgs84_geo = None
    self.navpose_update_interval = 0.1
    self.fake_gps_enabled = True
    self.gps_publish_interval_sec=1.0/GPS_PUB_RATE_HZ
    self.takeoff_m = TAKEOFF_M
    # Initialize Current Location
    self.current_location_wgs84_geo=GeoPoint()
    self.current_location_wgs84_geo.latitude = FAKE_GPS_START_GEOPOINT_WGS84[0]
    self.current_location_wgs84_geo.longitude =FAKE_GPS_START_GEOPOINT_WGS84[1]
    self.current_location_wgs84_geo.altitude = FAKE_GPS_START_GEOPOINT_WGS84[2]
    self.current_home_wgs84_geo=GeoPoint()
    self.current_home_wgs84_geo.latitude = FAKE_GPS_START_GEOPOINT_WGS84[0]
    self.current_home_wgs84_geo.longitude =FAKE_GPS_START_GEOPOINT_WGS84[1]
    self.current_home_wgs84_geo.altitude = FAKE_GPS_START_GEOPOINT_WGS84[2]
    ## Define Class Namespaces
    self.NEPI_NAVPOSE_SERVICE_NAME = NEPI_BASE_NAMESPACE + "nav_pose_query"
    # MAVLINK Namespace
    # Find Mavlink Topic Name
    node_string = "mavlink_tty"
    rospy.loginfo("Waiting for node that includes string: " + node_string)
    node_name = nepi.wait_for_node(node_string)
    MAVLINK_NAMESPACE = (node_name + '/')
    rospy.loginfo("Found mavlink namespace: " + MAVLINK_NAMESPACE)
    # MAVLINK Fake GPS Publish Topic
    MAVLINK_HILGPS_TOPIC = MAVLINK_NAMESPACE + "hil/gps"
    rospy.loginfo("Will publish fake gps on mavlink topic: " + MAVLINK_HILGPS_TOPIC)
    # MAVLINK Fake GPS Control Topic
    MAVLINK_FAKE_GPS_GOTO_GEOPOINT_TOPIC = MAVLINK_NAMESPACE + "fake_gps/goto_geopoint_wgs84"
    MAVLINK_FAKE_GPS_RESET_GEOPOINT_TOPIC = MAVLINK_NAMESPACE + "fake_gps/reset_geopoint_wgs84"
    MAVLINK_FAKE_GPS_RESET_CURRENT_TOPIC = MAVLINK_NAMESPACE + "fake_gps/reset_current"
    ## Define Class Services Calls
    ## Create Class Sevices    
    ## Create Class Publishers
    self.mavlink_fake_gps_mavlink_pub = rospy.Publisher(MAVLINK_HILGPS_TOPIC, HilGPS, queue_size=1)
    ## Start Class Subscribers
    rospy.Subscriber(MAVLINK_FAKE_GPS_GOTO_GEOPOINT_TOPIC, GeoPoint,self.mavlink_fake_gps_goto_geopoint_callback)
    rospy.Subscriber(MAVLINK_FAKE_GPS_RESET_GEOPOINT_TOPIC, GeoPoint, self.mavlink_fake_gps_reset_geopoint_callback)
    rospy.Subscriber(MAVLINK_FAKE_GPS_RESET_CURRENT_TOPIC, Empty, self.mavlink_fake_gps_reset_current_callback)
    ## Start Node Processes
    # Start print Callback if Enabled
    if PRINT_LOCATION_1HZ:
      rospy.Timer(rospy.Duration(1), self.mavlink_fake_gps_print_callback)
    #####
    rospy.Timer(rospy.Duration(self.navpose_update_interval), self.update_current_heading_callback)
    while self.current_heading_deg is None and not rospy.is_shutdown():
      rospy.loginfo("Waiting for current heading from navpose service call")
      nepi.sleep(1,10)
      #Start fake gps publish and print callbacks
      rospy.loginfo("Starting fake gps publishing to " + MAVLINK_HILGPS_TOPIC)
      rospy.Timer(rospy.Duration(self.gps_publish_interval_sec), self.mavlink_fake_gps_pub_callback)
    ## Subscribe to NEPI RBX driver if and when avaiable
    # Wait for rbx status topic to publish and get rbx namespace
    rbx_status_topic = "/rbx/status"
    rospy.loginfo("Waiting for rbx status message on topic: " + rbx_status_topic)
    rbx_topic=nepi.wait_for_topic(rbx_status_topic)
    NEPI_RBX_NAMESPACE = (rbx_topic.rpartition("rbx")[0] + "rbx/")
    rospy.loginfo("Found rbx namespace: " + NEPI_RBX_NAMESPACE)
    rospy.loginfo("Found rbx status topic: " + rbx_topic)
    # NEPI RBX Fake GPS RBX Subscriber Topic
    NEPI_RBX_SET_MODE_TOPIC = NEPI_RBX_NAMESPACE + "set_mode"  # Int to Defined Dictionary RBX_MODES
    NEPI_RBX_GOTO_POSITION_TOPIC = NEPI_RBX_NAMESPACE + "goto_position"
    NEPI_RBX_GOTO_LOCATION_TOPIC = NEPI_RBX_NAMESPACE + "goto_location"
    NEPI_RBX_SET_HOME_CURRENT_TOPIC = NEPI_RBX_NAMESPACE + "set_home_current"
    NEPI_RBX_GO_ACTION_TOPIC = NEPI_RBX_NAMESPACE + "go_action"  # Int to Defined Dictionary RBX_MODES
    NEPI_RBX_GET_TAKEOFF_M_TOPIC = NEPI_RBX_NAMESPACE + "get_takeoff_m"  # Float meters takeoff height
    #Start fake gps rbx control subscribers
    rospy.Subscriber(NEPI_RBX_GOTO_POSITION_TOPIC, Float64MultiArray, self.rbx_goto_position_callback)
    rospy.Subscriber(NEPI_RBX_GOTO_LOCATION_TOPIC, Float64MultiArray, self.rbx_goto_location_callback)
    rospy.Subscriber(NEPI_RBX_SET_HOME_CURRENT_TOPIC, Empty, self.rbx_set_home_current_callback)
    rospy.Subscriber(NEPI_RBX_SET_MODE_TOPIC, UInt8, self.rbx_set_mode_callback)
    rospy.Subscriber(NEPI_RBX_GO_ACTION_TOPIC, UInt8, self.rbx_go_action_callback)
    rospy.Subscriber(NEPI_RBX_GET_TAKEOFF_M_TOPIC, Float32, self.rbx_update_takeoff_m_callback)   
    ## Initiation Complete
    rospy.loginfo("Initialization Complete")

  #######################
  ### Node Methods
  
  ### Setup a regular Send Fake GPS callback using current geo point value
  def mavlink_fake_gps_pub_callback(self,timer):
    if self.fake_gps_enabled:
      hilgps=HilGPS()
      hilgps.header = Header(stamp=rospy.Time.now(), frame_id="mavlink_fake_gps")
      hilgps.fix_type=3
      hilgps.geo.latitude=self.current_location_wgs84_geo.latitude
      hilgps.geo.longitude=self.current_location_wgs84_geo.longitude
      hilgps.geo.altitude=self.current_location_wgs84_geo.altitude
      hilgps.satellites_visible=SAT_COUNT
      #rospy.loginfo("Created new HilGPS message")
      #rospy.loginfo(hilgps)
      # Create and publish Fake GPS Publisher
      hilgps.header = Header(stamp=rospy.Time.now(), frame_id="mavlink_fake_gps")
      if not rospy.is_shutdown():
        self.mavlink_fake_gps_mavlink_pub.publish(hilgps)


  ### Callback to simulate move to new global geo position
  def mavlink_fake_gps_goto_geopoint_callback(self,geopoint_msg):
    rospy.loginfo('')
    rospy.loginfo('***********************')
    rospy.loginfo("Recieved Fake GPS Goto Message")
    rospy.loginfo('***********************')
    rospy.loginfo('')
    rospy.loginfo(geopoint_msg)
    self.mavlink_fake_gps_move(geopoint_msg)


  ### function to simulate move to new global geo position
  def mavlink_fake_gps_move(self,geopoint_msg):
    rospy.loginfo('***********************')
    rospy.loginfo("Moving FROM, TO, DELTA")
    rospy.loginfo('***********************')
    rospy.loginfo(self.current_location_wgs84_geo)
    rospy.loginfo('')
    rospy.loginfo(geopoint_msg)
    org_geo=np.array([self.current_location_wgs84_geo.latitude, \
                      self.current_location_wgs84_geo.longitude, self.current_location_wgs84_geo.altitude])
    new_geo=np.array([geopoint_msg.latitude, geopoint_msg.longitude, geopoint_msg.altitude])
    for ind, val in enumerate(new_geo):
      if new_geo[ind] == -999.0: # Use current
        new_geo[ind]=org_geo[ind]
    delta_geo = new_geo - org_geo
    rospy.loginfo('')
    rospy.loginfo(delta_geo)
    
    move_dist_m = nepi_navpose.distance_geopoints(org_geo,new_geo)
    move_time = MOVE_UPDATE_TIME_SEC_PER_METER * move_dist_m
    move_steps = move_time * GPS_PUB_RATE_HZ
    stp_interval_sec = float(move_time)/float(move_steps)
    rospy.loginfo("Moving " + "%.2f" % move_dist_m + " meters in " + "%.2f" % move_time + " seconds")
    rospy.loginfo("with " + "%.2f" % move_steps + " steps")

    ramp=np.hanning(move_steps)
    ramp=ramp**2
    ramp_norm=ramp/np.sum(ramp)
    step_norm=np.zeros(len(ramp_norm))
    for ind, val in enumerate(ramp_norm):
      step_norm[ind]=np.sum(ramp_norm[0:ind])
    
    rospy.loginfo_timer = 0
    for ind, val in enumerate(step_norm):
      time.sleep(stp_interval_sec)
      rospy.loginfo_timer = rospy.loginfo_timer + stp_interval_sec
      cur_geo_step = delta_geo * val
      cur_geo = org_geo + cur_geo_step
      self.current_location_wgs84_geo.latitude = cur_geo[0]
      self.current_location_wgs84_geo.longitude = cur_geo[1]
      self.current_location_wgs84_geo.altitude = cur_geo[2]
      if rospy.loginfo_timer > 0.5:
        rospy.loginfo("")
        rospy.loginfo("Updated to")
        rospy.loginfo(self.current_location_wgs84_geo)
        current_error_m = nepi_navpose.distance_geopoints(cur_geo,new_geo)
        rospy.loginfo("Current move error : " + "%.2f" % (current_error_m) + " meters")
        rospy.loginfo_timer=0
    rospy.loginfo('')
    rospy.loginfo('***********************')
    rospy.loginfo("Moving Complete")
    rospy.loginfo('***********************')



  ### Callback to Reset GPS position and global x,y NED home position
  def mavlink_fake_gps_reset_geopoint_callback(self,geopoint_msg):
    rospy.loginfo('')
    rospy.loginfo('***********************')
    rospy.loginfo("Recieved Fake GPS Reset GeoPoint Message")
    rospy.loginfo('***********************')
    rospy.loginfo(geopoint_msg)
    org_geo=np.array([self.current_location_wgs84_geo.latitude, \
                      self.current_location_wgs84_geo.longitude, self.current_location_wgs84_geo.altitude])
    new_geo=np.array([geopoint_msg.latitude, geopoint_msg.longitude, geopoint_msg.altitude])
    for ind, val in enumerate(new_geo):
      if new_geo[ind] == -999.0: # Use current
        new_geo[ind]=org_geo[ind]
    self.current_location_wgs84_geo.latitude=new_geo[0]
    self.current_location_wgs84_geo.longitude=new_geo[1]
    self.current_location_wgs84_geo.altitude=new_geo[2]
    time.sleep(1)
    rospy.loginfo("Reseting GPS with new location")
    reset_gps()
    rospy.loginfo("Reset Complete")


    ### Callback to Reset GPS position and global x,y NED home position
  def mavlink_fake_gps_reset_current_callback(self,empty_msg):
    rospy.loginfo('')
    rospy.loginfo('***********************')
    rospy.loginfo("Recieved Fake GPS Reset Current Message")
    rospy.loginfo('***********************')
    rospy.loginfo("Reseting GPS with current location")
    self.reset_gps()
    rospy.loginfo("Reset Complete")

  ### Function to reset gps and wait for position ned x,y to reset
  def reset_gps(self):
    self.fake_gps_enabled = False
    rospy.loginfo("Waiting for GPS to reset")  
    self.fake_gps_enabled = False
    nepi.sleep(10,100)
    self.fake_gps_enabled = True


  ### Callback to rospy.loginfo the current fake geopoint position at slower rate
  def mavlink_fake_gps_print_callback(self,timer):
    rospy.loginfo('')
    rospy.loginfo("Current WGS84 Geo Location ")
    rospy.loginfo(self.current_location_wgs84_geo)


  #######################
  # NEPI NavPose Interface Methods

  ### Setup a regular background navpose get and update heading data
  def update_current_heading_callback(self,timer):
    # Get current NEPI NavPose data from NEPI ROS nav_pose_query service call
    try:
      get_navpose_service = rospy.ServiceProxy(self.NEPI_NAVPOSE_SERVICE_NAME, NavPoseQuery)
      nav_pose_response = get_navpose_service(NavPoseQueryRequest())
      self.current_heading_deg = nav_pose_response.nav_pose.heading.heading
      #rospy.loginfo('')
      #rospy.loginfo("Update current heading to: " + "%.2f" % (self.current_heading_deg))
    except Exception as e:
      rospy.loginfo("navpose service call failed: " + str(e))

  #######################
  # RBX Driver Interface Methods

  ### Function to monitor RBX GoTo Position Command Topics
  def rbx_goto_position_callback(self,position_cmd_msg):
    rospy.loginfo("*******************************")
    rospy.loginfo("Recieved GoTo Position Message")
    rospy.loginfo("")
    rospy.loginfo(position_cmd_msg)
    new_position = position_cmd_msg.data
    rospy.loginfo("Sending Fake GPS Setpoint Position Update")
    new_geopoint_wgs84=nepi_navpose.get_geopoint_at_body_point(self.current_location_wgs84_geo, \
                                                  self.current_heading_deg, new_position)    
    self.mavlink_fake_gps_move(new_geopoint_wgs84)


  ### Function to monitor RBX GoTo Location Command Topics
  def rbx_goto_location_callback(self,location_cmd_msg):
    rospy.loginfo("*******************************")
    rospy.loginfo("Recieved GoTo Location Message")
    rospy.loginfo("")
    rospy.loginfo(location_cmd_msg)
    new_location = location_cmd_msg.data
    new_geopoint_wgs84=GeoPoint()
    new_geopoint_wgs84.latitude = new_location[0]
    new_geopoint_wgs84.longitude = new_location[1]
    new_geopoint_wgs84.altitude = new_location[2]
    self.mavlink_fake_gps_move(new_geopoint_wgs84)

  ### Callback to set mode
  def rbx_set_home_current_callback(self,set_home_msg):
    rospy.loginfo("Recieved Set Home Message")
    self.current_home_wgs84_geo =  self.current_location_wgs84_geo

  ### Callback to set mode
  def rbx_set_mode_callback(self,mode_msg):
    rospy.loginfo("Recieved Set Mode Message")
    rospy.loginfo(mode_msg)
    mode_ind = mode_msg.data
    if mode_ind < 0 or mode_ind > (len(self.RBX_MODE_FUNCTIONS)-1):
      rospy.loginfo("No matching rbx mode found")
    else:
      set_mode_function = globals()[self.RBX_MODE_FUNCTIONS[mode_ind]]
      set_mode_function(self)

  ### Callback to set mode
  def rbx_go_action_callback(self,action_msg):
    rospy.loginfo("Recieved Go Action Message")
    rospy.loginfo(action_msg)
    action_ind = action_msg.data
    print(action_ind)
    print((len(self.RBX_ACTION_FUNCTIONS)-1))
    if action_ind < 0 or action_ind > (len(self.RBX_ACTION_FUNCTIONS)-1):
      rospy.loginfo("No matching rbx action found")
    else:
      set_action_function = globals()[self.RBX_ACTION_FUNCTIONS[action_ind]]
      set_action_function(self)

  ### Callback to update takeoff m height
  def rbx_update_takeoff_m_callback(self,takeoff_m_msg):
    self.takeoff_m =  takeoff_m_msg.data


  #######################
  # Mavlink Ardupilot Interface Methods
    
  ## Function for sending takeoff command
  global takeoff
  def takeoff(self):
    print(type(self.current_location_wgs84_geo.altitude))
    print(type(self.takeoff_m))
    new_alt_m = self.current_location_wgs84_geo.altitude + self.takeoff_m
    print(new_alt_m)
    new_geopoint_wgs84=GeoPoint()
    new_geopoint_wgs84.latitude = self.current_location_wgs84_geo.latitude
    new_geopoint_wgs84.longitude = self.current_location_wgs84_geo.longitude
    new_geopoint_wgs84.altitude = new_alt_m
    self.mavlink_fake_gps_move(new_geopoint_wgs84)

  ### Function for switching to STABILIZE mode
  global stabilize
  def stabilize(self):
    rospy.loginfo("")
      
  ### Function for switching to LAND mode
  global land
  def land(self):
    new_geopoint_wgs84=GeoPoint()
    new_geopoint_wgs84.latitude = self.current_location_wgs84_geo.latitude
    new_geopoint_wgs84.longitude = self.current_location_wgs84_geo.longitude
    new_geopoint_wgs84.altitude = 0
    self.mavlink_fake_gps_move(new_geopoint_wgs84)
    
  ### Function for sending go home command
  global rtl
  def rtl(self):
    self.mavlink_fake_gps_move(self.current_home_wgs84_geo)

  ### Function for switching to LOITER mode
  global loiter
  def loiter(self):
    rospy.loginfo("")

  ### Function for switching back to GUIDED mode
  global guided
  def guided(self):
    rospy.loginfo("")

  ### Function for switching back to current mission
  global resume
  def resume(self):
    rospy.loginfo("")

  ### Function for sending set home current
  global sethome_current
  def sethome_current():
    rospy.loginfo("")
    
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
