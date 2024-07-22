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


import os
# ROS namespace setup
NEPI_BASE_NAMESPACE = '/nepi/s2x/'
os.environ["ROS_NAMESPACE"] = NEPI_BASE_NAMESPACE[0:-1]
import rospy

import rosnode
import time
import sys
import numpy as np
import math
import random
from nepi_edge_sdk_base import nepi_ros 
from nepi_edge_sdk_base import nepi_nav
from nepi_edge_sdk_base import nepi_rbx

from std_msgs.msg import Empty, Bool, UInt8, Int8, Float32, Float64, String, Header
from mavros_msgs.msg import HilGPS, State
from geometry_msgs.msg import Point
from geographic_msgs.msg import GeoPoint
from mavros_msgs.srv import CommandHome
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped
from nepi_ros_interfaces.msg import RBXGotoPose, RBXGotoPosition, RBXGotoLocation
from nepi_ros_interfaces.srv import NavPoseQuery, NavPoseQueryRequest



#########################################
# Node Class
#########################################

class ArdupilotFakeGPS(object):

  # ROS namespace setup
  NEPI_BASE_NAMESPACE = nepi_ros.get_base_namespace()

  # MAVLINK Full or Partial Namespace
  MAVLINK_NAMESPACE = "mavlink_tty"

  #Homeup Location
  # [Lat, Long, Altitude_WGS84]
  FAKE_GPS_START_GEOPOINT_WGS84 = [46.6540828,-122.3187578,0.0]

  #GPS Setup
  SAT_COUNT = 20
  GPS_PUB_RATE_HZ = 50

  #GPS Simulation Position Update Controls
  #Adjust these settings for smoother xyz movements
  MOVE_UPDATE_TIME_SEC_PER_METER=1

  PRINT_LOCATION_1HZ = False
   
  #######################
  ### Node Initialization
  def __init__(self):
    self.fake_gps_enabled = False

    rospy.loginfo("RBX_FAKE_GPS: Starting Initialization Processes")
    ## Initialize Class Variables
    # RBX State and Mode Dictionaries
    self.rbx_cap_modes = []
    self.rbx_cap_actions = []
    self.current_location_wgs84_geo = None
    self.current_heading_deg = None
    self.current_home_wgs84_geo = None
    self.navpose_update_interval = 0.1
    self.fake_gps_ready = True
    self.gps_publish_interval_sec=1.0/self.GPS_PUB_RATE_HZ
    self.takeoff_m = 0
    # Initialize Current Location
    self.current_location_wgs84_geo=GeoPoint()
    self.current_location_wgs84_geo.latitude = self.FAKE_GPS_START_GEOPOINT_WGS84[0]
    self.current_location_wgs84_geo.longitude = self.FAKE_GPS_START_GEOPOINT_WGS84[1]
    self.current_location_wgs84_geo.altitude = self.FAKE_GPS_START_GEOPOINT_WGS84[2]
    self.current_home_wgs84_geo=GeoPoint()
    self.current_home_wgs84_geo.latitude = self.current_location_wgs84_geo.latitude
    self.current_home_wgs84_geo.longitude = self.current_location_wgs84_geo.longitude
    self.current_home_wgs84_geo.altitude = self.current_location_wgs84_geo.altitude

    rospy.loginfo("RBX_FAKE_GPS: Start Home GEO Location: " + str(self.current_home_wgs84_geo))
    ## Define Class Namespaces
    self.nepi_nav_SERVICE_NAME = self.NEPI_BASE_NAMESPACE + "nav_pose_query"


    # Find Mavlink NameSpace
    mavlink_namespace = rospy.get_name().replace("fake_gps","mavlink")
    rospy.loginfo("RBX_ARDU: Waiting for node that includes string: " + mavlink_namespace)
    mavlink_namespace = nepi_ros.wait_for_node(mavlink_namespace)
    self.mavlink_namespace = (mavlink_namespace + '/')
    rospy.loginfo("RBX_ARDU: Found mavlink namespace: " + self.mavlink_namespace)
    # MAVLINK Fake GPS Publish Topic
    MAVLINK_HILGPS_TOPIC = self.mavlink_namespace + "hil/gps"
    rospy.loginfo("RBX_FAKE_GPS: Will publish fake gps on mavlink topic: " + MAVLINK_HILGPS_TOPIC)
    self.mavlink_pub = rospy.Publisher(MAVLINK_HILGPS_TOPIC, HilGPS, queue_size=1)

    # Start update heading callback 
    rospy.Timer(rospy.Duration(self.navpose_update_interval), self.update_current_heading_callback)
    while self.current_heading_deg is None and not rospy.is_shutdown():
      rospy.loginfo("RBX_FAKE_GPS: Waiting for current heading from navpose service call")
      nepi_ros.sleep(1,10)
      #Home fake gps publish and print callbacks
      rospy.loginfo("RBX_FAKE_GPS: Homeing fake gps publishing to " + MAVLINK_HILGPS_TOPIC)
      rospy.Timer(rospy.Duration(self.gps_publish_interval_sec), self.fake_gps_pub_callback)

   # Setup RBX driver interfaces
    robot_namespace = rospy.get_name().replace("fake_gps","ardupilot")
    rospy.loginfo("RBX_ARDU: Waiting for node that includes string: " + robot_namespace)
    robot_namespace = nepi_ros.wait_for_node(robot_namespace)
    robot_namespace = robot_namespace + "/"
    rbx_namespace = (robot_namespace + 'rbx/')
    rospy.loginfo("RBX_ARDU: Found rbx namespace: " + rbx_namespace)

    # NEPI RBX Caps Service
    caps_topic = rbx_namespace + "capabilities_query"
    rbx_caps = nepi_rbx.get_capabilities(self,caps_topic)
    self.rbx_cap_states = eval(rbx_caps.state_options)
    self.rbx_cap_modes = eval(rbx_caps.mode_options)
    self.rbx_cap_actions = eval(rbx_caps.action_options)
    # NEPI RBX Settings Status Subscriber Topic
    rospy.Subscriber(robot_namespace + "settings_status", String, self.rbxSettingsCB)
    self.force_settings_pub = rospy.Publisher(robot_namespace + "publish_settings",Empty, queue_size=1)
    time.sleep(1)
    self.force_settings_pub.publish(Empty())
  

        
    # Create Fake GPS controls subscribers
    rospy.Subscriber("~set_enable", Bool, self.fakeGPSEnableCB)
    rospy.Subscriber("~reset",Empty, self.fakeGPSResetCb)
    rospy.Subscriber("~goto_position", RBXGotoPosition, self.fakeGPSGoPosCB)
    rospy.Subscriber("~goto_location", RBXGotoLocation, self.fakeGPSGoLocCB)
    rospy.Subscriber("~set_home", GeoPoint, self.fakeGPSSetHomeCB)
    rospy.Subscriber("~go_home", Empty, self.fakeGPSGoHomeCB)
    rospy.Subscriber("~set_home_current", Empty, self.fakeGPSSetHomeCurCB)
    rospy.Subscriber("~set_mode", UInt8, self.fakeGPSModeCB)
    rospy.Subscriber("~go_action", UInt8, self.fakeGPSActionCB)
 

    ## Initiation Complete
    rospy.loginfo("RBX_FAKE_GPS: Initialization Complete")

  #######################
  ### Node Methods


  ### Setup a regular Send Fake GPS callback using current geo point value
  def fake_gps_pub_callback(self,timer):
    if self.fake_gps_ready:
      self.publishFakeGPS()

  def publishFakeGPS(self):
    if self.fake_gps_enabled:
      hilgps=HilGPS()
      hilgps.header = Header(stamp=rospy.Time.now(), frame_id="mavlink_fake_gps")
      hilgps.fix_type=3
      hilgps.geo.latitude=self.current_location_wgs84_geo.latitude
      hilgps.geo.longitude=self.current_location_wgs84_geo.longitude
      hilgps.geo.altitude=self.current_location_wgs84_geo.altitude
      hilgps.satellites_visible=self.SAT_COUNT
      #rospy.loginfo("RBX_FAKE_GPS: Created new HilGPS message")
      #rospy.loginfo(hilgps)
      # Create and publish Fake GPS Publisher
      hilgps.header = Header(stamp=rospy.Time.now(), frame_id="mavlink_fake_gps")
      if not rospy.is_shutdown():
        self.mavlink_pub.publish(hilgps)


  ### function to simulate move to new global geo position
  def move(self,geopoint_msg):
    rospy.loginfo('***********************')
    rospy.loginfo("RBX_FAKE_GPS: Fake GPS Moving FROM, TO, DELTA")

    rospy.loginfo(self.current_location_wgs84_geo)
    rospy.loginfo('')
    rospy.loginfo(geopoint_msg)
    org_geo=np.array([self.current_location_wgs84_geo.latitude, \
                      self.current_location_wgs84_geo.longitude, self.current_location_wgs84_geo.altitude])
    cur_geo = org_geo
    new_geo=np.array([geopoint_msg.latitude, geopoint_msg.longitude, geopoint_msg.altitude])
    for ind, val in enumerate(new_geo):
      if new_geo[ind] == -999.0: # Use current
        new_geo[ind]=org_geo[ind]
    delta_geo = new_geo - org_geo
    rospy.loginfo('')
    rospy.loginfo(delta_geo)
    
    move_dist_m = nepi_nav.distance_geopoints(org_geo,new_geo)
    if move_dist_m > 0:
      move_time = self.MOVE_UPDATE_TIME_SEC_PER_METER * move_dist_m
      move_steps = move_time * self.GPS_PUB_RATE_HZ
      stp_interval_sec = float(move_time)/float(move_steps)
      rospy.loginfo("RBX_FAKE_GPS: Moving " + "%.2f" % move_dist_m + " meters in " + "%.2f" % move_time + " seconds")
      rospy.loginfo("RBX_FAKE_GPS: with " + "%.2f" % move_steps + " steps")

      ramp=np.hanning(move_steps)
      ramp=ramp**2
      ramp_norm=ramp/np.sum(ramp)
      step_norm=np.zeros(len(ramp_norm))
      for ind, val in enumerate(ramp_norm):
        step_norm[ind]=np.sum(ramp_norm[0:ind])
      
      rospy.loginfo_timer = 0
      for ind, val in enumerate(step_norm):
        time.sleep(stp_interval_sec)
        #rospy.loginfo_timer = rospy.loginfo_timer + stp_interval_sec
        cur_geo_step = delta_geo * val
        cur_geo = org_geo + cur_geo_step
        self.current_location_wgs84_geo.latitude = cur_geo[0]
        self.current_location_wgs84_geo.longitude = cur_geo[1]
        self.current_location_wgs84_geo.altitude = cur_geo[2]
        #if rospy.loginfo_timer > 0.5:
          #rospy.loginfo("RBX_FAKE_GPS: ")
          #rospy.loginfo("RBX_FAKE_GPS: Updated to")
          #rospy.loginfo(self.current_location_wgs84_geo)
          #current_error_m = nepi_nav.distance_geopoints(cur_geo,new_geo)
          #rospy.loginfo("RBX_FAKE_GPS: Current move error : " + "%.2f" % (current_error_m) + " meters")
          #rospy.loginfo_timer=0
    current_error_m = nepi_nav.distance_geopoints(cur_geo,new_geo)
    rospy.loginfo("RBX_FAKE_GPS: Move Error: " + "%.2f" % (current_error_m) + " meters")

    rospy.loginfo("RBX_FAKE_GPS: FAKE GPS Move Complete")
    rospy.loginfo('***********************')



  ### Callback to rospy.loginfo the current fake geopoint position at slower rate
  def print_callback(self,timer):
    if (self.checkEnable()):
      rospy.loginfo('')
      rospy.loginfo("RBX_FAKE_GPS: Current WGS84 Geo Location ")
      rospy.loginfo(self.current_location_wgs84_geo)


  #######################
  # NEPI NavPose Interfaces

  ### Setup a regular background navpose get and update heading data
  def update_current_heading_callback(self,timer):
    # Get current NEPI NavPose data from NEPI ROS nav_pose_query service call
    try:
      get_navpose_service = rospy.ServiceProxy(self.nepi_nav_SERVICE_NAME, NavPoseQuery)
      nav_pose_response = get_navpose_service(NavPoseQueryRequest())
      self.current_heading_deg = nav_pose_response.nav_pose.heading.heading
      #rospy.loginfo('')
      #rospy.loginfo("RBX_FAKE_GPS: Update current heading to: " + "%.2f" % (self.current_heading_deg))
    except Exception as e:
      rospy.loginfo("RBX_FAKE_GPS: navpose service call failed: " + str(e))


  #######################
  # NEPI Fake GPS Interfaces


  def fakeGPSResetCb(self,msg):
    rospy.loginfo('***********************')
    rospy.loginfo("Received Fake GPS Reset Msg")
    success = self.reset_gps()
    if success:
      rospy.loginfo("RBX_FAKE_GPS: Reset Complete")
    return success

  ### Function to reset gps and wait for position ned x,y to reset
  def reset_gps(self):
    self.fake_gps_ready = False
    time.sleep(1)
    new_location = GeoPoint()
    new_location.latitude = self.current_home_wgs84_geo.latitude
    new_location.longitude = self.current_home_wgs84_geo.longitude
    new_location.altitude = self.current_home_wgs84_geo.altitude
    self.current_location_wgs84_geo = new_location
    #rospy.loginfo("RBX_FAKE_GPS: Waiting for GPS to reset") 
    for i in range(90):
      self.publishFakeGPS()
      time.sleep(.1)
    self.fake_gps_ready = True
    return True

  ### Callback to set fake gps enable
  def fakeGPSEnableCB(self,msg):
      rospy.loginfo('***********************')
      rospy.loginfo("RBX_FAKE_GPS: Received set fake gps enable message")
      rospy.loginfo(msg)
      self.fake_gps_enabled = msg.data

  ### Callback to set home
  def fakeGPSSetHomeCB(self,geo_msg):
      rospy.loginfo('***********************')
      rospy.loginfo("RBX_FAKE_GPS: Received set home message")
      rospy.loginfo(geo_msg)
      self.current_home_wgs84_geo = geo_msg


  ### Callback to set home current
  def fakeGPSSetHomeCurCB(self,empty_msg):
      rospy.loginfo('***********************')
      rospy.loginfo("RBX_FAKE_GPS: Received set home current message")
      new_location = GeoPoint()
      new_location.latitude = self.current_location_wgs84_geo.latitude
      new_location.longitude = self.current_location_wgs84_geo.longitude
      new_location.altitude = self.current_location_wgs84_geo.altitude
      self.current_home_wgs84_geo = self.current_location_wgs84_geo

  ### Callback to go home
  def fakeGPSGoHomeCB(self,empty_msg):
      rospy.loginfo('***********************')
      rospy.loginfo("RBX_FAKE_GPS: Received go home message")
      self.move(self.current_home_wgs84_geo)

  ### Function to monitor RBX GoTo Position Command Topics
  def fakeGPSGoPosCB(self,position_cmd_msg):
    rospy.loginfo('***********************')
    rospy.loginfo("RBX_FAKE_GPS: Recieved GoTo Position Message")
    rospy.loginfo(position_cmd_msg)
    new_position = [position_cmd_msg.x_meters,position_cmd_msg.y_meters,position_cmd_msg.z_meters,position_cmd_msg.yaw_deg]
    rospy.loginfo("RBX_FAKE_GPS: Sending Fake GPS Setpoint Position Update")
    new_geopoint_wgs84=nepi_nav.get_geopoint_at_body_point(self.current_location_wgs84_geo, \
                                                  self.current_heading_deg, new_position)    
    self.move(new_geopoint_wgs84)


  ### Function to monitor RBX GoTo Location Command Topics
  def fakeGPSGoLocCB(self,location_cmd_msg):
    rospy.loginfo('***********************')
    rospy.loginfo("RBX_FAKE_GPS: Recieved GoTo Location Message")
    rospy.loginfo(location_cmd_msg)
    new_location = [location_cmd_msg.lat,location_cmd_msg.long,location_cmd_msg.altitude_meters,location_cmd_msg.yaw_deg]
    new_geopoint_wgs84=GeoPoint()
    new_geopoint_wgs84.latitude = new_location[0]
    new_geopoint_wgs84.longitude = new_location[1]
    new_geopoint_wgs84.altitude = new_location[2]
    self.move(new_geopoint_wgs84)


  ### Callback to set mode
  def fakeGPSModeCB(self,mode_msg):
    rospy.loginfo('***********************')
    rospy.loginfo("RBX_FAKE_GPS: Recieved Set Mode Message")
    rospy.loginfo(mode_msg)
    mode_ind = mode_msg.data
    if mode_ind < 0 or mode_ind > (len(self.rbx_cap_modes)-1):
      rospy.loginfo("RBX_FAKE_GPS: No matching rbx mode found")
    else:
      set_mode_function = globals()[self.rbx_cap_modes[mode_ind]]
      set_mode_function(self)

  ### Callback to set mode
  def fakeGPSActionCB(self,action_msg):
    rospy.loginfo('***********************')
    rospy.loginfo("RBX_FAKE_GPS: Recieved Go Action Message")
    rospy.loginfo(action_msg)
    action_ind = action_msg.data
    print(action_ind)
    print((len(self.rbx_cap_actions)-1))
    if action_ind < 0 or action_ind > (len(self.rbx_cap_actions)-1):
      rospy.loginfo("RBX_FAKE_GPS: No matching rbx action found")
    else:
      set_action_function = globals()[self.rbx_cap_actions[action_ind]]
      set_action_function(self)


  #######################
  # Mavlink Ardupilot Interface Methods
    
  ### update Takeoff height from RBX settings status message
  def rbxSettingsCB(self,settingsMsg):
    rospy.loginfo('***********************')
    rospy.loginfo("RBX_FAKE_GPS: Fake GPS recievied settings status message")
    rospy.loginfo(settingsMsg)
    settings = nepi_ros.parse_settings_msg_data(settingsMsg.data)
    setting = nepi_ros.get_setting_from_settings('takeoff_height_m',settings)
    #rospy.loginfo(setting)
    if setting is not None:
      [s_name,s_type, data]= nepi_ros.get_data_from_setting(setting)    
      #rospy.loginfo(data)
      #rospy.loginfo(type(data))
      if data is not None:
        self.takeoff_m = data

  ## Function for sending takeoff command

  global LAUNCH
  def LAUNCH(self):
    print(self.current_location_wgs84_geo.altitude)
    print(self.takeoff_m)
    new_alt_m = self.current_location_wgs84_geo.altitude + self.takeoff_m
    print(new_alt_m)
    new_geopoint_wgs84=GeoPoint()
    new_geopoint_wgs84.latitude = self.current_location_wgs84_geo.latitude
    new_geopoint_wgs84.longitude = self.current_location_wgs84_geo.longitude
    new_geopoint_wgs84.altitude = new_alt_m
    self.move(new_geopoint_wgs84)

  global TAKEOFF
  def TAKEOFF(self):
    print(self.current_location_wgs84_geo.altitude)
    print(self.takeoff_m)
    new_alt_m = self.current_location_wgs84_geo.altitude + self.takeoff_m
    print(new_alt_m)
    new_geopoint_wgs84=GeoPoint()
    new_geopoint_wgs84.latitude = self.current_location_wgs84_geo.latitude
    new_geopoint_wgs84.longitude = self.current_location_wgs84_geo.longitude
    new_geopoint_wgs84.altitude = new_alt_m
    self.move(new_geopoint_wgs84)

  ### Function for switching to STABILIZE mode
  global STABILIZE
  def STABILIZE(self):
    rospy.loginfo("RBX_FAKE_GPS: ")
      
  ### Function for switching to LAND mode
  global LAND
  def LAND(self):
    new_geopoint_wgs84=GeoPoint()
    new_geopoint_wgs84.latitude = self.current_location_wgs84_geo.latitude
    new_geopoint_wgs84.longitude = self.current_location_wgs84_geo.longitude
    new_geopoint_wgs84.altitude = 0
    self.move(new_geopoint_wgs84)
    
  ### Function for sending go home command
  global RTL
  def RTL(self):
    self.move(self.current_home_wgs84_geo)

  ### Function for switching to LOITER mode
  global LOITER
  def LOITER(self):
    rospy.loginfo("RBX_FAKE_GPS: ")

  ### Function for switching back to GUIDED mode
  global GUIDED
  def GUIDED(self):
    rospy.loginfo("RBX_FAKE_GPS: ")

  ### Function for switching back to current mission
  global RESUME
  def RESUME(self):
    rospy.loginfo("RBX_FAKE_GPS: ")

  
    #######################
    # Node Cleanup Function
    
  def cleanup_actions(self):
    rospy.loginfo("RBX_FAKE_GPS: Shutting down: Executing script cleanup actions")


#########################################
# Main
#########################################
if __name__ == '__main__':
  search_name = "mavlink"
  replace_name = "fake_gps"

  rospy.loginfo("Waiting for namespace: " + search_name)
  mavlink_namespace = nepi_ros.wait_for_node(search_name)
  rospy.loginfo("Found namespace: " + mavlink_namespace)
  mavlink_namespace_list = mavlink_namespace.split('/')
  node_name = None
  for name in mavlink_namespace_list:
    if "mavlink" in name:
      node_name = name.replace(search_name,replace_name)
  if node_name is not None:
    rospy.init_node
    rospy.init_node(name = node_name)
    rospy.loginfo("Launching node named: " + node_name)
    node = ArdupilotFakeGPS()  
    #Set up node shutdown
    rospy.on_shutdown(node.cleanup_actions)
    # Spin forever (until object is detected)
    rospy.spin()
  
