#!/usr/bin/env python

# Sample NEPI Automation Script.
# If your NEPI system does not have an attached GPS/IMU/Compass or other
# NavPose source, this script can be set to run at startup setting fixed
# NavPose values on your system.
# Uses onboard ROS python library to
# 1. Set a fixed NavPose Solution (Lat,Long,Alt,Heading)
# 2. Exit after set

import rospy
from std_msgs.msg import Float64, Empty
from sensor_msgs.msg import NavSatFix

#####################################################################################
# SETUP - Edit as Necessary ##################################
##########################################

# ROS namespace setup
BASE_NAMESPACE = "/nepi/s2x/"

###!!!!!!!! Set NavPose initialization values !!!!!!!!
SET_NAVPOSE_FIXED_GPS_TOPIC = BASE_NAMESPACE + "nav_pose_mgr/set_init_gps_fix"
SET_NAVPOSE_FIXED_HEADING_TOPIC = BASE_NAMESPACE + "nav_pose_mgr/set_init_heading"
REINIT_NAVPOSE_SOLUTION_TOPIC = BASE_NAMESPACE + "nav_pose_mgr/reinit_solution"

LAT = 47.654327404913275
LONG = -122.3213194962342
ALT = 0.0
HEADING = 45

#####################################################################################
# Methods
#####################################################################################

### Script Entrypoint
def startNode():
  rospy.init_node("set_fixed_navpose_auto_script")
  rospy.loginfo("Starting Set Fixed NavPose automation script")

  # Make sure to use the correct message type: "rostopic info" can help identify it. In this case it is a sensor_msgs/NavSatFix message type
  gps_pub = rospy.Publisher(SET_NAVPOSE_FIXED_GPS_TOPIC, NavSatFix, queue_size=10)
  rospy.sleep(1) # VERY IMPORTANT - Sleep a bit between declaring a publisher and using it subscribers have time to subscribe
  gps_pub.publish(latitude=LAT, longitude=LONG, altitude=ALT) # Keyword args are a nice way to construct messages right in the publish() function

  heading_pub = rospy.Publisher(SET_NAVPOSE_FIXED_HEADING_TOPIC, Float64, queue_size=10)
  rospy.sleep(1) # VERY IMPORTANT - Sleep a bit between declaring a publisher and using it subscribers have time to subscribe
  heading_pub.publish(data=HEADING)

  # At this point, the "init" fields have been updated, but they haven't yet been applied as the current values for GPS and HEADING, 
  # so we do that here via the "reinit_solution" topic.
  reinit_pub = rospy.Publisher(REINIT_NAVPOSE_SOLUTION_TOPIC, Empty, queue_size=10)
  rospy.sleep(1) # VERY IMPORTANT - Sleep a bit between declaring a publisher and using it subscribers have time to subscribe
  reinit_pub.publish() # "Empty" message types don't require a payload

  # Sleep a bit so that the publisher threads above have time to do their work before this script exits
  rospy.loginfo("Sleeping to let publishers finish")
  rospy.sleep(1)

#####################################################################################
# Main
#####################################################################################

if __name__ == '__main__':
  startNode()

