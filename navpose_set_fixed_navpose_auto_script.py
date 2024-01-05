#!/usr/bin/env python


__author__ = "Jason Seawall"
__copyright__ = "Copyright 2023, Numurus LLC"
__email__ = "nepi@numurus.com"
__credits__ = ["Jason Seawall", "Josh Maximoff"]

__license__ = "GPL"
__version__ = "2.0.4.0"


# Sample NEPI Automation Script.
# If your NEPI system does not have an attached GPS/IMU/Compass or other
# NavPose source, this script can be set to run at startup setting fixed
# NavPose values on your system.
# Uses onboard ROS python library to
# 1. Set a fixed NavPose Solution (Lat,Long,Alt,Heading)
# 2. Exit after setting

import rospy
from std_msgs.msg import Float64, Empty
from sensor_msgs.msg import NavSatFix

#####################################################################################
# SETUP - Edit as Necessary ##################################
##########################################

# ROS namespace setup
NEPI_BASE_NAMESPACE = "/nepi/s2x/"

###!!!!!!!! Set NavPose initialization values !!!!!!!!
SET_NAVPOSE_FIXED_GPS_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose_mgr/set_init_gps_fix"
SET_NAVPOSE_FIXED_HEADING_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose_mgr/set_init_heading"
REINIT_NAVPOSE_SOLUTION_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose_mgr/reinit_solution"

#Numurus Office
LAT = 47.65412711862056
LONG = -122.31894922885307
ALT = 0.0
HEADING = 45

#####################################################################################
# Methods
#####################################################################################


### System Initialization processes
def initialize_actions():
  print("")
  print("Starting Initialization")
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
  print("Initialization Complete")


### Cleanup processes on node shutdown
def cleanup_actions():
  print("Shutting down: Executing script cleanup actions")
  time.sleep(2)

### Script Entrypoint
def startNode():
  rospy.init_node("set_fixed_navpose_auto_script")
  rospy.loginfo("Starting Set Fixed NavPose automation script")
  # Run initialization processes
  initialize_actions()
  # run cleanup actions on shutdown
  rospy.on_shutdown(cleanup_actions)
  # Spin forever
  rospy.spin()
  
#####################################################################################
# Main
#####################################################################################

if __name__ == '__main__':
  startNode()

