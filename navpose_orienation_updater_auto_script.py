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
# 1. Creates an Orientation Publishers and Sets NEPI NavPose to connect to them
# 3. Creates Body Orientation Subscriber that applies to Start
# 2. Exit after setting

import rospy
import time
import numpy as np
import math
import tf


from std_msgs.msg import String, Float64, Float64MultiArray, Empty
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped, QuaternionStamped

#####################################################################################
# SETUP - Edit as Necessary ##################################
##########################################


NAVPOSE_UPDATE_RATE_HZ = 10
# Set Start roll pitch yaw body frame values
START_RPY_DEGS =  [20.0,30.0,40.0]# Roll, Pitch, Yaw


# ROS namespace setup
NEPI_BASE_NAMESPACE = "/nepi/s2x/"


NEPI_SET_NAVPOSE_ORIENTATION_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose_mgr/set_orientation_topic"

### Setup NEPI NavPose Orientation Update Publisher Namespace
NEPI_ORIENTATION_BODY_ODOMETRY_TOPIC = NEPI_BASE_NAMESPACE + "odom_body_odometry"

### Setup NEPI NavPose Orientation Update Relative Subscriber Namespace
NEPI_RPY_BODY_DEGS_TOPIC = NEPI_BASE_NAMESPACE + "rpy_body_degs"


#####################################################################################
# Globals
#####################################################################################
navpose_update_orientation_pub = rospy.Publisher(NEPI_ORIENTATION_BODY_ODOMETRY_TOPIC, Odometry , queue_size=1)
navpose_update_interval_sec = float(1.0)/NAVPOSE_UPDATE_RATE_HZ
current_rpy_degs = START_RPY_DEGS

#####################################################################################
# Methods
#####################################################################################


### System Initialization processes
def initialize_actions():
  global current_rpy_degs
  global navpose_update_interval_sec
  print("")
  print("Starting Initialization") 
  ##############################
  # Start our Update NavPose Publisher Topic
  print("Starting NavPose Update Publisher at: " + str(NAVPOSE_UPDATE_RATE_HZ) + " Hz")
  rospy.Timer(rospy.Duration(navpose_update_interval_sec), orienation_update_publish_callback)
  time.sleep(2) # Wait for publiser to start
  ##############################
  # Update Orientation source to our new update orientation publisher
  wait_for_topic(NEPI_ORIENTATION_BODY_ODOMETRY_TOPIC, 'nav_msgs/Odometry')
  set_orientation_pub = rospy.Publisher(NEPI_SET_NAVPOSE_ORIENTATION_TOPIC, String, queue_size=10)
  time.sleep(1) # Wait between creating and using publisher
  set_orientation_pub.publish(NEPI_ORIENTATION_BODY_ODOMETRY_TOPIC)
  print("Orientation Topic Set to: " + NEPI_ORIENTATION_BODY_ODOMETRY_TOPIC)
  ##############################
  ## Start Get RPY Body Subscriber Callback
  rospy.Subscriber(NEPI_RPY_BODY_DEGS_TOPIC, Float64MultiArray, get_rpy_body_callback)
  print("Initialization Complete")

### Setup a regular background navpose update timer callback
def orienation_update_publish_callback(timer):
  global current_rpy_degs
  global navpose_update_orientation_pub
  new_pos = Point()
  new_pos.x = 0
  new_pos.y = 0
  new_pos.z = 0

  current_orientation_quat = convert_rpy2quat(current_rpy_degs)
  new_quat = Quaternion()
  new_quat.x = current_orientation_quat[0]
  new_quat.y = current_orientation_quat[1]
  new_quat.z = current_orientation_quat[2]
  new_quat.w = current_orientation_quat[3]

  new_pose=Pose()
  new_pose.position = new_pos
  new_pose.orientation = new_quat

  new_odometry = Odometry()
  new_odometry.header.stamp = rospy.Time.now()
  new_odometry.header.frame_id = 'map'
  new_odometry.child_frame_id = 'nepi_center_frame'
  new_odometry.pose.pose = new_pose
  if not rospy.is_shutdown():
    navpose_update_orientation_pub.publish(new_odometry)

### Callback to get relative orientations applied to start
def get_rpy_body_callback(rpy_body_degs_msg):
  global current_rpy_degs
  new_rpy_body_degs = rpy_body_degs_msg.data
  print("Received roll pitch yaw body degs update message")
  print(new_rpy_body_degs)
  new_orien_body_degs = list(START_RPY_DEGS) # Initialize to current
  new_orien_body_degs[0] = new_rpy_body_degs[0] 
  new_orien_body_degs[1] = new_rpy_body_degs[1] 
  new_orien_body_degs[2] = new_rpy_body_degs[2] 
  print("Setting new current orienation body degs")
  print(new_orien_body_degs)
  current_rpy_degs = new_orien_body_degs

### Function to Convert Roll, Pitch, Yaw Degrees to Quaternion Attitude
def convert_rpy2quat(rpy_attitude_deg):
  roll_rad = math.radians(rpy_attitude_deg[0])
  pitch_rad = math.radians(rpy_attitude_deg[1]) 
  yaw_rad = math.radians(rpy_attitude_deg[2])
  #xyzw_attitude = tf.transformations.quaternion_from_euler(roll_rad,pitch_rad,yaw_rad,axes="sxyz")
  xyzw_attitude = tf.transformations.quaternion_from_euler(pitch_rad, yaw_rad, roll_rad, axes="ryzx")
  return xyzw_attitude


### Function to wait for topic to exist
def wait_for_topic(topic_name,message_name):
  topic_in_list = False
  while topic_in_list is False and not rospy.is_shutdown():
    topic_list=rospy.get_published_topics(namespace='/')
    topic_to_connect=[topic_name, message_name]
    if topic_to_connect not in topic_list:
      time.sleep(.1)
    else:
      topic_in_list = True

### Cleanup processes on node shutdown
def cleanup_actions():
  print("Shutting down: Executing script cleanup actions")
  time.sleep(2)

### Script Entrypoint
def startNode():
  rospy.init_node("navpose_orienatation_updater_auto_script")
  rospy.loginfo("Starting NavPose Orienation Updater automation script")
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

