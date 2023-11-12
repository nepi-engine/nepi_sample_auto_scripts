#!/usr/bin/env python

# Sample NEPI Automation Script.
# Uses onboard ROS python and mavros libraries to
# 1) Send Takeoff Command
# 2) Get the current attitude value
# 3) Sets system to Guided mode and arms system
# 4) Set attitude target goal
# 5) Monitors system to reach set attitude and take some action
# 6) Set system to original state and arm mode

import rospy
import time
import numpy as np
import math
import tf

from std_msgs.msg import String, Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from mavros_msgs.msg import State, AttitudeTarget, CommandTOL 
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest


#####################################################################################
# SETUP - Edit as Necessary ##################################
##########################################

ATTITUDE_GOAL_DEG = [-999,30,-999] # Roll, Pitch, Yaw Degrees: Enter -999 to use current value
# rostopic echo /nepi/s2x/pixhawk_mavlink/global_position/local
MAX_ERROR_DEG = 5 # Goal reached when all values within this error value
THRUST_SETTING = 0.2
SET_MODE = 'GUIDED' # Requires GPS, use mavros_fake_gps_altitude_auto_script.py to fake
SETPOINT_UPDATE_RATE_HZ = 5

# ROS namespace setup
BASE_NAMESPACE = "/nepi/s2x/"
MAVROS_NAMESPACE = BASE_NAMESPACE + "pixhawk_mavlink/"
MAVROS_STATE_TOPIC = MAVROS_NAMESPACE + "state"
MAVROS_TAKEOFF_SERVICE = MAVROS_NAMESPACE + "cmd/takeoff"
MAVROS_ARMING_SERVICE = MAVROS_NAMESPACE + "cmd/arming"
MAVROS_SET_MODE_SERVICE = MAVROS_NAMESPACE + "set_mode"
MAVROS_ORIENTATION_TOPIC = MAVROS_NAMESPACE + "global_position/local"
MAVROS_SET_ATTITUDE_TOPIC = MAVROS_NAMESPACE + "setpoint_raw/attitude"


#####################################################################################
# Globals
#####################################################################################
arming_client = rospy.ServiceProxy(MAVROS_ARMING_SERVICE, CommandBool)
set_mode_client = rospy.ServiceProxy(MAVROS_SET_MODE_SERVICE, SetMode)
attitude_control_pub = rospy.Publisher(MAVROS_SET_ATTITUDE_TOPIC, AttitudeTarget, queue_size=1)
attitude_goal_deg = None
attitude_current_deg = None
attitude_target_msg = None
state_current = None
org_mode = None
org_arm = None
set_mode = None # global control 
set_arm = False # global control
setpoint_interval_sec = 1.0/SETPOINT_UPDATE_RATE_HZ

#####################################################################################
# Methods
#####################################################################################


### System Initialization processes
def initialize_actions():
  global attitude_goal_deg
  global attitude_current_deg
  global attitude_target_msg
  global state_current
  global org_mode
  global org_arm
  global set_mode
  global set_arm
  global setpoint_interval_sec
  ## Start Set State Callback
  print("Starting get_state_callback")
  rospy.Subscriber(MAVROS_STATE_TOPIC, State, callback = get_state_callback)
  while state_current is None:
    print("Waiting for current state reading to set")
    time.sleep(1)
  print(state_current)
  print("Getting Original State")
  org_mode = state_current.mode
  print(org_mode)
  org_arm = state_current.armed
  print(org_arm)
  ## Start Attitude Subscriber Callback
  print("Starting get_attiude_callback")
  print(MAVROS_ORIENTATION_TOPIC)
  rospy.Subscriber(MAVROS_ORIENTATION_TOPIC, Odometry, check_attitude_callback)
  while attitude_current_deg is None:
    #print("Waiting for current attitude reading to set")
    time.sleep(.1)
  ## Set Attitude Goal
  new_attitude_deg=attitude_current_deg # Initialize to current
  for ind, val in enumerate(ATTITUDE_GOAL_DEG): # Overwrite if set and valid
    if ATTITUDE_GOAL_DEG[ind] != -999 and np.abs(ATTITUDE_GOAL_DEG[ind])<180:
      new_attitude_deg[ind]=ATTITUDE_GOAL_DEG[ind]
  attitude_goal_deg = new_attitude_deg
  print('')
  print("Attitude Goal Degrees")
  print(" Roll, Pitch, Yaw")
  print(["%.2f" % attitude_goal_deg[0],"%.2f" % attitude_goal_deg[1],"%.2f" % attitude_goal_deg[2]])
  ## Change Vehicle State
  print("Send new mode and arming command")
  set_mode = SET_MODE
  set_arm = True
  while(state_current.mode != set_mode or state_current.armed != set_arm):
    update_state(set_mode,set_arm)
    #print("Waiting for new mode and arming to set")
    print(state_current.mode)
    print(state_current.armed)
    time.sleep(.1)
  ## Set up Setpoint Attitude control message
  attitude_goal_quat = convert_rpy2quat(attitude_goal_deg)
  orientation = Quaternion()
  orientation.x = attitude_goal_quat[0]
  orientation.y = attitude_goal_quat[1]
  orientation.z = attitude_goal_quat[2]
  orientation.w = attitude_goal_quat[3]
  body_rate = Vector3()
  body_rate.x = 0
  body_rate.y = 0
  body_rate.z = 0
  type_mask = 1|2|4
  thrust = THRUST_SETTING
  attitude_target_msg = AttitudeTarget()
  attitude_target_msg.orientation = orientation
  attitude_target_msg.body_rate = body_rate
  attitude_target_msg.type_mask = type_mask
  attitude_target_msg.thrust = thrust
  ## Start setpoint attitude callback
  rospy.Timer(rospy.Duration(setpoint_interval_sec), setpoint_attitude_callback)
  print("Completed Initialization")

### Function to Convert Quaternion Attitude to Roll, Pitch, Yaw Degrees
def update_state(mode='LOITER',armed=False):
  global arming_client
  global set_mode_client 
  new_mode = SetModeRequest()
  new_mode.custom_mode = mode
  set_mode_client.call(new_mode)
  arm_cmd = CommandBoolRequest()
  arm_cmd.value = armed
  arming_client.call(arm_cmd)


### Callback to get current state
def get_state_callback(state_msg):
  global state_current
  state_current = state_msg


### Setup a regular setpoint attitude publisher
def setpoint_attitude_callback(timer):
  global attitude_control_pub
  global attitude_target_msg
  if attitude_target_msg is not None:
    attitude_control_pub.publish(attitude_target_msg)

### Callback to check attitude
def check_attitude_callback(odometry_msg):
  global attitude_goal_deg
  global attitude_current_deg
  pose = odometry_msg.pose.pose.orientation
  xyzw_pose=(pose.x,pose.y,pose.z,pose.w)
  attitude_current_deg = convert_quat2rpy(xyzw_pose)
  print('')
  print("Current and Goal Attitude Degrees")
  print(" Roll, Pitch, Yaw")
  print(["%.2f" % attitude_current_deg[0],"%.2f" % attitude_current_deg[1],"%.2f" % attitude_current_deg[2]])
  if attitude_goal_deg is not None:  # Wait for attitude goal to be set
    print(["%.2f" % attitude_goal_deg[0],"%.2f" % attitude_goal_deg[1],"%.2f" % attitude_goal_deg[2]])
    errors_deg = np.array(attitude_goal_deg) - np.array(attitude_current_deg)
    print("Current Attitude Errors")
    print(["%.3f" % errors_deg[0],"%.3f" % errors_deg[1],"%.3f" % errors_deg[2]])
    if np.amax(np.abs(errors_deg))< MAX_ERROR_DEG:
          print("Attitude Reached")
          rospy.signal_shutdown("Attitude Reached")
  else:
    print('Goal not set yet')



### Function to Convert Quaternion Attitude to Roll, Pitch, Yaw Degrees
def convert_quat2rpy(xyzw_attitude):
  ##  print('')
  ##  print("Input Attitude Oreantation")
  ##  print(" X, Y, Z, W")
  ##  print(["%.3f" % xyzw_attitude[0], "%.3f" % xyzw_attitude[1], "%.3f" % xyzw_attitude[2], "%.3f" % xyzw_attitude[3]])
  # Convert 
  rpy_attitude_rad = tf.transformations.euler_from_quaternion(xyzw_attitude)
  rpy_attitude_deg = np.array(rpy_attitude_rad) * 180/math.pi
  roll_deg = rpy_attitude_deg[0] 
  pitch_deg = rpy_attitude_deg[1] 
  yaw_deg = rpy_attitude_deg[2]
  ##  print(" Calculated Attitude Degrees")
  ##  print(" Roll, Pitch, Yaw")
  ##  print(["%.3f" % roll_deg,"%.3f" % pitch_deg,"%.3f" % yaw_deg])
  return rpy_attitude_deg

### Function to Convert Roll, Pitch, Yaw Degrees to Quaternion Attitude
def convert_rpy2quat(rpy_attitude_deg):
  roll_deg = rpy_attitude_deg[0] 
  pitch_deg = rpy_attitude_deg[1] 
  yaw_deg = rpy_attitude_deg[2]
  ##  print('')
  ##  print("Input Attitude Degrees")
  ##  print(" Roll, Pitch, Yaw")
  ##  print(["%.3f" % roll_deg,"%.3f" % pitch_deg,"%.3f" % yaw_deg])
  # Convert
  xyzw_attitude = tf.transformations.quaternion_from_euler(math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg))
  ##  print("Calculated Attitude Oreantation")
  ##  print(" X, Y, Z, W")
  ##  print(["%.3f" % xyzw_attitude[0], "%.3f" % xyzw_attitude[1], "%.3f" % xyzw_attitude[2], "%.3f" % xyzw_attitude[3]])
  return xyzw_attitude

### Cleanup processes on node shutdown
def cleanup_actions():
##  global state_current
##  global org_mode
##  global org_arm
##  ("Setting System End_Mode and End_Arm states")
##  while(state_current.mode != org_mode  or state_current.armed != org_arm):
##    update_state(org_mode,org_arm)
##    time.sleep(0.1)
  time.sleep(2)
  
### Script Entrypoint
def startNode():
  rospy.loginfo("Starting MAVROS Set Attitude Target automation script")
  rospy.init_node("mavros_set_attitude_target_auto_script")
  #initialize system including pan scan process
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

