#!/usr/bin/env python

# Sample NEPI Automation Script.
# Uses onboard ROS python and mavros libraries to
# Gets the current attitude value
# Sets system to Guided mode and arms system
# Sets attitude target goal
# Monitors system to reach set attitude
# Sets system to original state and arm mode

import rospy
import time
import numpy as np

from std_msgs.msg import String, Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from mavros_msgs.msg import State, AttitudeTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest


#####################################################################################
# SETUP - Edit as Necessary ##################################
##########################################

ATTITUDE_GOAL = [0.5,-0.05,0.05,-1.0] # Orientation x,y,z,w from watching SSH terminal orientation values using
# rostopic echo /nepi/s2x/pixhawk_mavlink/global_position/local
ATTITUDE_ERROR_BOUND = 0.1 # Goal reached when all values within this error value
THRUST_SETTING = 0.2

SET_MODE = 'Guided' # Requires GPS, use mavros_fake_gps_altitude_auto_script.py to fake

# ROS namespace setup
BASE_NAMESPACE = "/nepi/s2x/"
MAVROS_NAMESPACE = BASE_NAMESPACE + "pixhawk_mavlink/"

MAVROS_STATE_TOPIC = MAVROS_NAMESPACE + "state"
MAVROS_ARMING_SERVICE = MAVROS_NAMESPACE + "cmd/arming"
MAVROS_SET_MODE_SERVICE = MAVROS_NAMESPACE + "set_mode"
MAVROS_ORIENTATION_TOPIC = MAVROS_NAMESPACE + "global_position/local"
MAVROS_SET_ATTITUDE_TOPIC = MAVROS_NAMESPACE + "setpoint_raw/attitude"
STATE_UPDATE_INTERVAL_SEC = 0.2

#####################################################################################
# Globals
#####################################################################################
arming_client = rospy.ServiceProxy(MAVROS_ARMING_SERVICE, CommandBool)
set_mode_client = rospy.ServiceProxy(MAVROS_SET_MODE_SERVICE, SetMode)
current_attitude = None
current_state = None
org_mode = None
org_arm = None
set_mode = None # global control 
set_arm = False # global control
pub_enable = True


#####################################################################################
# Methods
#####################################################################################


### System Initialization processes
def initialize_actions():
  global current_state
  global org_mode
  global org_arm
  global set_mode
  global set_arm
  print("Starting get_state_callback")
  rospy.Subscriber(MAVROS_STATE_TOPIC, State, callback = get_state_callback)
  print("Waiting for first state update")
  while current_state is not None:
    time.sleep(0.1)
  time.sleep(1)
  print("Getting Original State")
  print(current_state)
  org_mode = current_state.mode
  org_arm = current_state.armed
  print("Starting get_attiude_callback")
  print(MAVROS_ORIENTATION_TOPIC)
  rospy.Subscriber(MAVROS_ORIENTATION_TOPIC, Odometry, check_attitude_callback)
  print("Starting set_state_callback")
  set_mode = SET_MODE
  set_arm = True
  rospy.Timer(rospy.Duration(STATE_UPDATE_INTERVAL_SEC), set_state_callback)
  print("Publishing mavros setpoint_raw/attitude command")
  orientation = Quaternion()
  orientation.x = ATTITUDE_GOAL[0]
  orientation.y = ATTITUDE_GOAL[1]
  orientation.z = ATTITUDE_GOAL[2]
  orientation.w = ATTITUDE_GOAL[3]
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
  attitude_control_pub = rospy.Publisher(MAVROS_SET_ATTITUDE_TOPIC, AttitudeTarget, queue_size=10)
  attitude_control_pub.publish(attitude_target_msg)
  time.sleep(0.25)
  print("Completed Initialization")

### Callback to get current state
def get_state_callback(state_msg):
  global current_state
  current_state = state_msg

### Callback to check attitude
def check_attitude_callback(odometry_msg):
  time.sleep(.1)
  global current_attitude
  global org_mode
  global org_arm
  global set_mode
  global set_arm
  pose=odometry_msg.pose.pose.orientation
  current_attitude = [pose.x, pose.y, pose.z, pose.w]
  print('')
  print("Current Attitude")
  print(current_attitude)
  current_attitude_errors = np.array(ATTITUDE_GOAL) - np.array(current_attitude)
  print("Current Errors")
  print(current_attitude_errors)
  if np.amax(np.abs(current_attitude_errors[0:2]))< ATTITUDE_ERROR_BOUND:
        print("Attitude Reached")
        rospy.signal_shutdown("Attitude Reached")
  
### Setup a regular set state publisher
def set_state_callback(timer):
  global arming_client
  global set_mode_client
  global set_mode
  global set_arm
  global pub_enable
  if pub_enable:
    if set_mode is not None:
      new_mode = SetModeRequest()
      new_mode.custom_mode = set_mode
      set_mode_client.call(new_mode)
    arm_cmd = CommandBoolRequest()
    arm_cmd.value = set_arm
    arming_client.call(arm_cmd)

### Cleanup processes on node shutdown
def cleanup_actions():
  global current_state
  global org_mode
  global org_arm
  global set_mode
  global set_arm
  global pub_enable
  ("Setting System End_Mode and End_Arm states")
  set_mode=org_mode
  set_arm=org_arm
  while(current_state.mode != org_mode  or current_state.armed != org_arm):
    time.sleep(0.1)
  pub_enable = False
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

