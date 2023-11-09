#!/usr/bin/env python

# Sample NEPI Automation Script.
# Uses onboard ROS python and mavros libraries to
# 1. Connect NEPI NavPose topics to appropriate mavros topics

import rospy
from std_msgs.msg import String, Empty
from sensor_msgs.msg import NavSatFix

#####################################################################################
# SETUP - Edit as Necessary ##################################
##########################################

# ROS namespace setup
BASE_NAMESPACE = "/nepi/s2x/"
MAVROS_NAMESPACE = BASE_NAMESPACE + "pixhawk_mavlink/"

### Uncomment This Section for Ardupilot Firmware
MAVROS_GPS_TOPIC = MAVROS_NAMESPACE + "global_position/global"
MAVROS_HEADING_TOPIC = MAVROS_NAMESPACE + "global_position/compass_hdg"
MAVROS_ORIENTATION_TOPIC = MAVROS_NAMESPACE + "global_position/local"

### Uncomment This Section for Pixhawk Firmware
##MAVROS_GPS_TOPIC = MAVROS_NAMESPACE + "global_position/global"
##MAVROS_HEADING_TOPIC = MAVROS_NAMESPACE + "global_position/compass_hdg"
##MAVROS_ORIENTATION_TOPIC = MAVROS_NAMESPACE + "local_position/odom"

### Setup NEPI NavPose Topic Namespaces
SET_NAVPOSE_GPS_TOPIC = BASE_NAMESPACE + "nav_pose_mgr/set_gps_fix_topic"
SET_NAVPOSE_HEADING_TOPIC = BASE_NAMESPACE + "nav_pose_mgr/set_heading_topic"
SET_NAVPOSE_ORIENTATION_TOPIC = BASE_NAMESPACE + "nav_pose_mgr/set_orientation_topic"

#####################################################################################
# Methods
#####################################################################################

### Script Entrypoint
def startNode():
  rospy.init_node("set_mavlink_navpose_auto_script")
  rospy.loginfo("Starting Set MAVLink NavPose automation script")

  # Update GPS source
  gps_pub = rospy.Publisher(SET_NAVPOSE_GPS_TOPIC, String, queue_size=10)
  rospy.sleep(1) # VERY IMPORTANT - Sleep a bit between declaring a publisher and using it subscribers have time to subscribe
  gps_pub.publish(MAVROS_GPS_TOPIC)
  # Update Heading source
  heading_pub = rospy.Publisher(SET_NAVPOSE_HEADING_TOPIC, String, queue_size=10)
  rospy.sleep(1) # VERY IMPORTANT - Sleep a bit between declaring a publisher and using it subscribers have time to subscribe
  heading_pub.publish(MAVROS_HEADING_TOPIC)
  # Update Orientation source
  orientation_pub = rospy.Publisher(SET_NAVPOSE_ORIENTATION_TOPIC, String, queue_size=10)
  rospy.sleep(1) # VERY IMPORTANT - Sleep a bit between declaring a publisher and usin
  orientation_pub.publish(MAVROS_ORIENTATION_TOPIC)

  # Sleep a bit so that the publisher threads above have time to do their work before this script exits
  rospy.loginfo("Sleeping to let publishers finish")
  rospy.sleep(1)

#####################################################################################
# Main
#####################################################################################

if __name__ == '__main__':
  startNode()

