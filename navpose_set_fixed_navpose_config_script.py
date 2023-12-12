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
# 1. Set a fixed NavPose Solution (Lat,Long,Alt,Heading,Roll,Pitch,Yaw)
# 2. Exit after setting

import rospy
import math
import tf
import time

from std_msgs.msg import Float64, Empty
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Quaternion, QuaternionStamped

#####################################################################################
# SETUP - Edit as Necessary ##################################
##########################################

# Set Start Fixed NavPose Values
#Numurus Office
START_GEOPOINT = [47.6540828,-122.3187578,0.0] # [Lat, Long, Altitude_AMSL_M]
START_HEADING_DEG = 88.0 # Global True North, or 0 for Body Relative
START_ORIENTATION_DEGS = [10.0,20.0,30.0]
# ROS namespace setup
NEPI_BASE_NAMESPACE = "/nepi/s2x/"

###!!!!!!!! Set NavPose initialization values !!!!!!!!
SET_NAVPOSE_FIXED_GPS_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose_mgr/set_init_gps_fix"
SET_NAVPOSE_FIXED_HEADING_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose_mgr/set_init_heading"
SET_NAVPOSE_FIXED_ORIENTATION_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose_mgr/set_init_orientation"
REINIT_NAVPOSE_SOLUTION_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose_mgr/reinit_solution"


#####################################################################################
# Methods
#####################################################################################


### System Initialization processes
def initialize_actions():
  print("")
  print("Starting Initialization")
  # Make sure to use the correct message type: "rostopic info" can help identify it. In this case it is a sensor_msgs/NavSatFix message type
  gps_pub = rospy.Publisher(SET_NAVPOSE_FIXED_GPS_TOPIC, NavSatFix, queue_size=1)
  rospy.sleep(1) # VERY IMPORTANT - Sleep a bit between declaring a publisher and using it subscribers have time to subscribe
  lat = START_GEOPOINT[0]
  long = START_GEOPOINT[1]
  alt = START_GEOPOINT[2]
  gps_pub.publish(latitude=lat, longitude=long, altitude=alt) # Keyword args are a nice way to construct messages right in the publish() function

  heading_pub = rospy.Publisher(SET_NAVPOSE_FIXED_HEADING_TOPIC, Float64, queue_size=1)
  rospy.sleep(1) # VERY IMPORTANT - Sleep a bit between declaring a publisher and using it subscribers have time to subscribe
  heading = START_HEADING_DEG
  heading_pub.publish(data=heading)

  orientation_pub = rospy.Publisher(SET_NAVPOSE_FIXED_ORIENTATION_TOPIC, QuaternionStamped, queue_size=1)
  rospy.sleep(1) # VERY IMPORTANT - Sleep a bit between declaring a publisher and using it subscribers have time to subscribe
  current_orientation_quat = convert_rpy2quat(START_ORIENTATION_DEGS)
  new_quat = Quaternion()
  new_quat.x = current_orientation_quat[0]
  new_quat.y = current_orientation_quat[1]
  new_quat.z = current_orientation_quat[2]
  new_quat.w = current_orientation_quat[3]
  orientation_pub.publish(quaternion=new_quat)

  # At this point, the "init" fields have been updated, but they haven't yet been applied as the current values for GPS and HEADING, 
  # so we do that here via the "reinit_solution" topic.
  rospy.sleep(1) # Give new navpose values time to get captured
  reinit_pub = rospy.Publisher(REINIT_NAVPOSE_SOLUTION_TOPIC, Empty, queue_size=10)
  rospy.sleep(1) # VERY IMPORTANT - Sleep a bit between declaring a publisher and using it subscribers have time to subscribe
  reinit_pub.publish() # "Empty" message types don't require a payload
  print("Initialization Complete")

### Function to Convert Quaternion Attitude to Roll, Pitch, Yaw Degrees
def convert_quat2rpy(xyzw_attitude):
  rpy_attitude_rad = tf.transformations.euler_from_quaternion(xyzw_attitude)
  rpy_attitude_deg = np.array(rpy_attitude_rad) * 180/math.pi
  roll_deg = rpy_attitude_deg[0] 
  pitch_deg = rpy_attitude_deg[1] 
  yaw_deg = rpy_attitude_deg[2]
  return rpy_attitude_deg

### Function to Convert Roll, Pitch, Yaw Degrees to Quaternion Attitude
def convert_rpy2quat(rpy_attitude_deg):
  roll_deg = rpy_attitude_deg[0] 
  pitch_deg = rpy_attitude_deg[1] 
  yaw_deg = rpy_attitude_deg[2]
  xyzw_attitude = tf.transformations.quaternion_from_euler(math.radians(pitch_deg), math.radians(yaw_deg), math.radians(roll_deg),axes="ryzx")
  xyzw_norm = (xyzw_attitude[0]*xyzw_attitude[0]) + (xyzw_attitude[1]*xyzw_attitude[1]) + (xyzw_attitude[2]*xyzw_attitude[2]) + (xyzw_attitude[3]*xyzw_attitude[3])
  print("Norm is " + str(xyzw_norm))
  return xyzw_attitude

### Cleanup processes on node shutdown
def cleanup_actions():
  print("Shutting down: Executing script cleanup actions")

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
