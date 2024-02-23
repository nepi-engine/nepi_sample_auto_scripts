#!/usr/bin/env python
#
# NEPI Dual-Use License
# Project: nepi_sample_auto_scripts
#
# This license applies to any user of NEPI Engine software
#
# Copyright (C) 2023 Numurus, LLC <https://www.numurus.com>
# see https://github.com/numurus-nepi/nepi_edge_sdk_base
#
# This software is dual-licensed under the terms of either a NEPI software developer license
# or a NEPI software commercial license.
#
# The terms of both the NEPI software developer and commercial licenses
# can be found at: www.numurus.com/licensing-nepi-engine
#
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - https://www.numurus.com/licensing-nepi-engine
# - mailto:nepi@numurus.com
#
#

# Sample Solution Startup Script. 
# 1. Launches scripts from list that are not running
# 2. Waits for shutdown
# 3. Stops scripts that were launched by this script

import rospy
import os
import sys
import numpy as np
import math
import time

from std_srvs.srv import Empty, EmptyRequest, Trigger
from nepi_ros_interfaces.srv import GetScriptsQuery,GetRunningScriptsQuery ,LaunchScript, StopScript


#########################################
# USER SETTINGS - Edit as Necessary 
#########################################

SCRIPT_LIST = ["pantilt_ptx_navpose_driver_script.py",
               	"navpose_set_fixed_config_script.py",
		"pantilt_ptx_navpose_config_script.py",
		"ai_detector_config_script.py",
               	"pantilt_object_tracker_action_script.py"] #  Script filenames to start/stop

#########################################
# ROS NAMESPACE SETUP
#########################################

# ROS namespace setup
NEPI_BASE_NAMESPACE = "/nepi/s2x/"

# NEPI Get NAVPOSE Solution Service Name
AUTO_GET_INSTALLED_SCRIPTS_SERVICE_NAME = NEPI_BASE_NAMESPACE + "get_scripts"
AUTO_GET_RUNNING_SCRIPTS_SERVICE_NAME = NEPI_BASE_NAMESPACE + "get_running_scripts"
AUTO_LAUNCH_SCRIPT_SERVICE_NAME = NEPI_BASE_NAMESPACE + "launch_script"
AUTO_STOP_SCRIPT_SERVICE_NAME = NEPI_BASE_NAMESPACE + "stop_script"

#########################################
# Globals
#########################################

get_installed_scripts_service = rospy.ServiceProxy(AUTO_GET_INSTALLED_SCRIPTS_SERVICE_NAME, GetScriptsQuery )
get_running_scripts_service = rospy.ServiceProxy(AUTO_GET_RUNNING_SCRIPTS_SERVICE_NAME, GetRunningScriptsQuery )
launch_script_service = rospy.ServiceProxy(AUTO_LAUNCH_SCRIPT_SERVICE_NAME, LaunchScript)
stop_script_service = rospy.ServiceProxy(AUTO_STOP_SCRIPT_SERVICE_NAME, StopScript)
scripts_installed_at_start = None
scripts_running_at_start = None

#########################################
# Methods
#########################################

### System Initialization processes
def initialize_actions():
  global scripts_installed_at_start
  global scripts_running_at_start
  print("")
  print("***********************")
  print("Starting Initialization")
  ### Get list of installed scripts
  print("Getting list of installed scripts")
  print(["Calling service name: " + AUTO_GET_INSTALLED_SCRIPTS_SERVICE_NAME])
  while scripts_installed_at_start == None and not rospy.is_shutdown():
      scripts_installed_at_start = get_installed_scripts()
      if scripts_installed_at_start == None:
        print("Service call failed, waiting 1 second then retrying")
        time.sleep(1)
  print("Scripts installed at start:")
  print(scripts_installed_at_start)
  ### Get list of running scripts
  print("")
  print("Getting list of running scripts at start")
  print(["Calling service name: " + AUTO_GET_RUNNING_SCRIPTS_SERVICE_NAME])
  while scripts_running_at_start == None and not rospy.is_shutdown():
      scripts_running_at_start = get_running_scripts()
      if scripts_running_at_start == None:
        print("Service call failed, waiting 1 second then retrying")
        time.sleep(1)
  print("Scripts running at start:")
  print(scripts_running_at_start)
  print("Initialization Complete")


### Function to get list of installed scripts
def get_installed_scripts():
  global get_installed_scripts_service
  installed_scripts = None
  try:
    response = get_installed_scripts_service()
    installed_scripts = response.scripts
    #print("Installed Scripts = " + str(installed_scripts))
  except Exception as e:
    print("Get installed scripts service call failed: " + str(e))
  return installed_scripts

### Function to get list of running scripts
def get_running_scripts():
  global get_running_scripts_service
  running_scripts = None
  try:
    response = get_running_scripts_service()
    running_scripts = response.running_scripts
    #print("Running Scripts = " + str(running_scripts))
  except Exception as e:
    print("Get running scripts service call failed: " + str(e))
  return running_scripts


### Function to launch a script
def launch_script(script2launch):
  global launch_script_service
  launch_success=False
  try:
    success = launch_script_service(script=script2launch)
    print("Launched script: " + str(success))
    launch_success=True
  except Exception as e:
    print("Launch script service call failed: " + str(e))
  return launch_success

### Function to stop script
def stop_script(script2stop):
  stop_success=False
  global stop_script_service
  try:
    success = stop_script_service(script=script2stop)
    print("Stopped script: " + str(success))
    stop_success=True
  except Exception as e:
    print("Stop script service call failed: " + str(e))
  return stop_success

### Function to start scripts from list
def launch_scripts(script_list):
  installed_scripts = get_installed_scripts()
  running_scripts = get_running_scripts()
  if installed_scripts is not None and running_scripts is not None:
    for script2launch in script_list:
      script_installed = val_in_list(script2launch,installed_scripts)
      if script_installed:
        script_running = val_in_list(script2launch,running_scripts)
        if script_running is False:
            print("")
            print(["Launching script: " + script2launch])
            script_launch = launch_script(script2launch)
            if script_launch:
              print("Script launch call success")
              script_running = False
              while script_running is False and not rospy.is_shutdown():
                running_scripts = get_running_scripts()
                script_running = val_in_list(script2launch,running_scripts)
                print("Waiting for script to launch")
                time.sleep(.5) # Sleep before checking again
              print("Script started successfully")
            else:
               print("Scipt launch call failed")
        else:
          print("Script already running, skipping launch process")
      else:
        print("Script not found, skipping launch process")
  else:
    print("Failed to get installed and running script list")
  #running_scripts = get_running_scripts()
  #print(running_scripts)
          

### Function to stop scripts from list, a
def stop_scripts(script_list,optional_ignore_script_list=[]):
  installed_scripts = get_installed_scripts()
  running_scripts = get_running_scripts()
  if installed_scripts is not None and running_scripts is not None:
    for script2stop in script_list:
      script_running = val_in_list(script2stop,running_scripts)
      script_ignore = val_in_list(script2stop,optional_ignore_script_list)
      if script_running is True and script_ignore is False:
        script_running = val_in_list(script2stop,running_scripts)
        if script_running is True:
            print("")
            print(["Stopping script: " + script2stop])
            script_stop = stop_script(script2stop)
            if script_stop:
              print("Script stop call success")
            else:
               print("Scipt stop call failed")
        else:
          print("Scipt in ignore list, skipping launch process")
      else:
        print("Script not found, skipping launch process")
  else:
    print("Failed to get installed and running script list")
  #running_scripts = get_running_scripts()
  #print(running_scripts)
    

### Function for checking if val in list
def val_in_list(val2check,list2check):
  in_list = False
  if len(list2check) > 0:
    for list_val in list2check:
      #print(str(val2check) + ' , ' + str(list_val))
      #print(val2check == list_val)
      if val2check == list_val:
        in_list = True
  return in_list
 
### Cleanup processes on node shutdown
def cleanup_actions():
  global scripts_running_at_start
  print("Shutting down: Executing script cleanup actions")
  # Stop scripts from list
  stop_scripts(SCRIPT_LIST,scripts_running_at_start)


### Script Entrypoint
def startNode():
  rospy.loginfo("Starting PanTilt Object Tracker Startup Script")
  rospy.init_node("pantilt_object_tracker_startup_script")
  # Run initialization processes
  initialize_actions()
  # Launch scripts from list
  launch_scripts(SCRIPT_LIST)
  # run cleanup actions on shutdown
  rospy.on_shutdown(cleanup_actions)
  # Spin forever
  rospy.spin()

#########################################
# Main
#########################################

if __name__ == '__main__':
  startNode()


