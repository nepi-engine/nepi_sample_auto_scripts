#!/usr/bin/env python


__author__ = "Jason Seawall"
__copyright__ = "Copyright 2023, Numurus LLC"
__email__ = "nepi@numurus.com"
__credits__ = ["Jason Seawall", "Josh Maximoff"]

__license__ = "GPL"
__version__ = "2.0.4.0"


# Sample NEPI Automation Script.
# Uses onboard ROS python library to
# 1. Subscribes to NEPI PTX supported pantilt status message
# 2. Creates an Orientation Publishers and Sets NEPI NavPose to connect to them

###################################################
# Local Body Position Setpoint Function use these body relative x,y,z,yaw conventions
# x+ axis is forward
# y+ axis is right
# z+ axis is down
# Only yaw orientation updated
# yaw+ clockwise, yaw- counter clockwise from x axis (0 degrees faces x+ and rotates positive using right hand rule around z+ axis down)
#####################################################


import rospy
import time
import numpy as np
import math
import tf


from std_msgs.msg import String, Float64, Float64MultiArray, Empty
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped, QuaternionStamped
from nepi_ros_interfaces.msg import PanTiltStatus, StringArray

#####################################################################################
# SETUP - Edit as Necessary ##################################
##########################################


NAVPOSE_UPDATE_RATE_HZ = 10
# Pan and Tilt setup parameters
PT_REVERSE_PAN = True # Flip Sensor Feedback Values for NavPose Body Frame
PT_REVERSE_TILT = True # Flip Sensor Feedback Values for NavPose Body Frame
# Set Start roll pitch yaw body frame values
START_RPY_DEGS =  [0.0,0.0,0.0]# Roll, Pitch, Yaw in body frame degs


### ROS namespace setup
NEPI_BASE_NAMESPACE = "/nepi/s2x/"
PT_NAMESPACE = NEPI_BASE_NAMESPACE + "iqr_pan_tilt/"

### PanTilt Subscribe Topics
PT_GET_STATUS_TOPIC = PT_NAMESPACE + "ptx/status"
### PanTilt NavPose Publish Topic
PANTILT_NAVPOSE_PUBLISH_TOPIC = PT_NAMESPACE + "odometry"
### NEPI NavPose Setting Publish Topic
NEPI_SET_NAVPOSE_ORIENTATION_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose_mgr/set_orientation_topic"


#####################################################################################
# Globals
#####################################################################################
navpose_pt_orientation_pub = rospy.Publisher(PANTILT_NAVPOSE_PUBLISH_TOPIC, Odometry , queue_size=1)
navpose_update_interval_sec = float(1.0)/NAVPOSE_UPDATE_RATE_HZ
current_rpy_pt_degs = START_RPY_DEGS

pt_yaw_now_deg=0
pt_pitch_now_deg=0
pt_yaw_now_ratio=0
pt_pitch_now_ratio=0
pt_speed_now_ratio=0
#####################################################################################
# Methods
#####################################################################################


### System Initialization processes
def initialize_actions():
  global current_rpy_pt_degs
  global navpose_update_interval_sec
  print("")
  print("Starting Initialization")
  # Start PT Status Callback
  wait_for_topic(PT_GET_STATUS_TOPIC, 'nepi_ros_interfaces/PanTiltStatus')
  print("Starting Pan Tilt Stutus callback")
  rospy.Subscriber(PT_GET_STATUS_TOPIC, PanTiltStatus, pt_status_callback)  
  ##############################
  # Start our Update NavPose Publisher Topic
  print("Starting NavPose Update Publisher at: " + str(NAVPOSE_UPDATE_RATE_HZ) + " Hz")
  rospy.Timer(rospy.Duration(navpose_update_interval_sec), orienation_update_publish_callback)
  time.sleep(2) # Wait for publiser to start
  ##############################
  # Update Orientation source to our new update orientation publisher
  wait_for_topic(PANTILT_NAVPOSE_PUBLISH_TOPIC, 'nav_msgs/Odometry')
  set_orientation_pub = rospy.Publisher(NEPI_SET_NAVPOSE_ORIENTATION_TOPIC, String, queue_size=10)
  time.sleep(1) # Wait between creating and using publisher
  set_orientation_pub.publish(PANTILT_NAVPOSE_PUBLISH_TOPIC)
  print("Orientation Topic Set to: " + PANTILT_NAVPOSE_PUBLISH_TOPIC)
  print("Initialization Complete")

### Setup a regular background navpose update timer callback
def orienation_update_publish_callback(timer):
  global current_rpy_pt_degs
  global navpose_pt_orientation_pub
  new_pos = Point()
  new_pos.x = 0
  new_pos.y = 0
  new_pos.z = 0

  current_orientation_quat = convert_rpy2quat(current_rpy_pt_degs)
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
    navpose_pt_orientation_pub.publish(new_odometry)

### Simple callback to get pt status info
def pt_status_callback(PanTiltStatus):
  global current_rpy_pt_degs
  # This is just to get the current pt positions
  pt_yaw_now_deg=PanTiltStatus.yaw_now_deg
  pt_pitch_now_deg=PanTiltStatus.pitch_now_deg
  if PT_REVERSE_PAN:
    pt_yaw_now_deg = -pt_yaw_now_deg
  if PT_REVERSE_TILT:
    pt_pitch_now_deg = -pt_pitch_now_deg
  current_rpy_pt_degs=[START_RPY_DEGS[0],pt_pitch_now_deg,pt_yaw_now_deg]


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

### Script Entrypoint
def startNode():
  rospy.init_node("pantilt_navpose_orienatation_updater_auto_script")
  rospy.loginfo("Starting Pantilt NavPose Orienation Updater automation script")
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

