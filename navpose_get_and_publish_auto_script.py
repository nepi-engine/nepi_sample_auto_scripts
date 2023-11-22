#!/usr/bin/env python

# Sample NEPI Automation Script.
# Uses onboard ROS python library to
# 1. call the NEPI ROS's nav_pose_mgr/query_data_products service
# 2. gets and publishes current navpose data at set rate to topic


import rospy
import time
import sys

from nepi_ros_interfaces.msg import NavPose
from nepi_ros_interfaces.srv import NavPoseQuery, NavPoseQueryRequest


#####################################################################################
# SETUP - Edit as Necessary ##################################
##########################################

# ROS namespace setup
NEPI_BASE_NAMESPACE = "/nepi/s2x/"

# NEPI Get NAVPOSE Solution Service Name
NAVPOSE_SERVICE_NAME = NEPI_BASE_NAMESPACE + "nav_pose_query"

### NavPose Heading, Oreanation, Location, and Position Publish Topics
NAVPOSE_CURRENT_NAVPOSE_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose_current/navpose"
NAVPOSE_PUB_RATE_HZ = 10

#####################################################################################
# Globals
#####################################################################################
navpose_navpose_pub = rospy.Publisher(NAVPOSE_CURRENT_NAVPOSE_TOPIC, NavPose, queue_size=1)
navpose_pub_interval_sec = float(1.0)/NAVPOSE_PUB_RATE_HZ
#####################################################################################
# Methods
#####################################################################################

### Setup a regular background navpose get and publish timer callback
def navpose_get_publish_callback(timer):
  global navpose_navpose_pub 
  if not rospy.is_shutdown():
    # Get current NEPI NavPose data from NEPI ROS nav_pose_query service call
    rospy.wait_for_service(NAVPOSE_SERVICE_NAME)
    try:
      get_navpose_service = rospy.ServiceProxy(NAVPOSE_SERVICE_NAME, NavPoseQuery)
      nav_pose_response = get_navpose_service(NavPoseQueryRequest())
      current_navpose = nav_pose_response.nav_pose
      #publish(nav_pose_response)
      navpose_navpose_pub.publish(current_navpose)
    except rospy.ServiceException as e:
      print("Service call failed: %s"%e)
      time.sleep(1)
      rospy.signal_shutdown("Service call failed")

  ### Cleanup processes on node shutdown
def cleanup_actions():
  print("Shutting down: Executing script cleanup actions")
  time.sleep(2)


### Script Entrypoint
def startNode():
  rospy.loginfo("Starting Get Publish NavPose automation script")
  rospy.init_node(name="navpose_get_publish_auto_script")
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

