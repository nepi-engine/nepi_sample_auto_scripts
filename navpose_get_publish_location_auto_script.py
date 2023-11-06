#!/usr/bin/env python

# Sample NEPI Automation Script.
# Uses onboard ROS python library to
# 1. call the NEPI ROS's nav_pose_mgr/query_data_products service
# 2. get current Lat, Long, Altitude data
# 3. publish as ROS geographic_msgs topic

import rospy
import time
import sys

from std_msgs.msg import Float64
from nepi_ros_interfaces.srv import NavPoseQuery, NavPoseQueryRequest
from geographic_msgs.msg import GeoPoint


#####################################################################################
# SETUP - Edit as Necessary ##################################
##########################################

# ROS namespace setup
BASE_NAMESPACE = "/nepi/s2x/"

NAVPOSE_SERVICE_NAME = BASE_NAMESPACE + "nav_pose_query"
OUTPUT_GEOPOINT_TOPIC = BASE_NAMESPACE + "current_location"
OUTPUT_INTERVAL_SEC = 0.25

#####################################################################################
# Globals
#####################################################################################
geopoint_data_pub = rospy.Publisher(OUTPUT_GEOPOINT_TOPIC, GeoPoint, queue_size=1)

navpose_timestamp = 0 # Latest Data
navpose_transform = 0 # No Tranformation
publish_enable = True

#####################################################################################
# Methods
#####################################################################################

### Setup a regular background navpose get and publish timer callback
def navpose_get_publish_callback(timer):
  # Called periodically
  global geopoint_data_pub
  global navpose_timestamp
  global navpose_transform
  global publish_enable
  # Get current NEPI NavPose data from NEPI ROS nav_pose_query service call
  rospy.wait_for_service(NAVPOSE_SERVICE_NAME)
  try:
    get_navpose_service = rospy.ServiceProxy(NAVPOSE_SERVICE_NAME, NavPoseQuery)
    nav_pose_response = get_navpose_service(NavPoseQueryRequest())
    #print(nav_pose_response)
    geopoint_data_msg=GeoPoint()
    geopoint_data_msg.latitude=nav_pose_response.nav_pose.fix.latitude
    geopoint_data_msg.longitude=nav_pose_response.nav_pose.fix.longitude
    geopoint_data_msg.altitude=nav_pose_response.nav_pose.fix.altitude
    if publish_enable:
      #print(geopoint_data_msg)
      geopoint_data_pub.publish(geopoint_data_msg)
  except rospy.ServiceException as e:
    print("Service call failed: %s"%e)
    time.sleep(1)
    rospy.signal_shutdown("Service call failed")
 


  ### Cleanup processes on node shutdown
def cleanup_actions():
  global geopoint_data_pub
  global publish_enable
  publish_enable=False
  time.sleep(2)
  print("Shutting down: Executing script cleanup actions")
  # Stop geopoint publisher
  geopoint_data_pub.unregister()
  time.sleep(2)


### Script Entrypoint
def startNode():
  rospy.loginfo("Starting Get Publish NavPose automation script")
  rospy.init_node(name="navpose_get_publish_auto_script")
  rospy.Timer(rospy.Duration(OUTPUT_INTERVAL_SEC), navpose_get_publish_callback)

  # run cleanup actions on shutdown
  rospy.on_shutdown(cleanup_actions)
  # Spin forever
  rospy.spin()


#####################################################################################
# Main
#####################################################################################

if __name__ == '__main__':
  startNode()

