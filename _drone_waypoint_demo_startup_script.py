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
import sys
import time
from resources import nepi

from std_srvs.srv import Empty, EmptyRequest, Trigger
from nepi_ros_interfaces.srv import GetScriptsQuery,GetRunningScriptsQuery ,LaunchScript, StopScript


#########################################
# USER SETTINGS - Edit as Necessary 
#########################################

SCRIPT_LIST = ["ardupilot_rbx_driver_script.py",
                "ardupilot_rbx_fake_gps_process_script.py",
               	"drone_waypoint_demo_mission_script.py"] #  Script filenames to start/stop

#########################################
# ROS NAMESPACE SETUP
#########################################

# ROS namespace setup
NEPI_BASE_NAMESPACE = "/nepi/s2x/"

#########################################
# Node Class
#########################################

class drone_waypoint_demo_startup(object):

  #######################
  ### Node Initialization
  def __init__(self):
    rospy.loginfo("Starting Initialization Processes")
    ## Initialize Class Variables
    self.scripts_installed_at_start = None
    self.scripts_running_at_start = None
    ## Define Class Namespaces
    AUTO_GET_INSTALLED_SCRIPTS_SERVICE_NAME = NEPI_BASE_NAMESPACE + "get_scripts"
    AUTO_GET_RUNNING_SCRIPTS_SERVICE_NAME = NEPI_BASE_NAMESPACE + "get_running_scripts"
    AUTO_LAUNCH_SCRIPT_SERVICE_NAME = NEPI_BASE_NAMESPACE + "launch_script"
    AUTO_STOP_SCRIPT_SERVICE_NAME = NEPI_BASE_NAMESPACE + "stop_script"
    ## Create Class Service Calls
    self.get_installed_scripts_service = rospy.ServiceProxy(AUTO_GET_INSTALLED_SCRIPTS_SERVICE_NAME, GetScriptsQuery )
    self.get_running_scripts_service = rospy.ServiceProxy(AUTO_GET_RUNNING_SCRIPTS_SERVICE_NAME, GetRunningScriptsQuery )
    self.launch_script_service = rospy.ServiceProxy(AUTO_LAUNCH_SCRIPT_SERVICE_NAME, LaunchScript)
    self.stop_script_service = rospy.ServiceProxy(AUTO_STOP_SCRIPT_SERVICE_NAME, StopScript)
    ## Create Class Publishers
    ## Start Class Subscribers
    ## Start Node Processes
    rospy.loginfo("")
    rospy.loginfo("***********************")
    rospy.loginfo("Starting Initialization")
    ### Get list of installed scripts
    rospy.loginfo("Getting list of installed scripts")
    rospy.loginfo(["Calling service name: " + AUTO_GET_INSTALLED_SCRIPTS_SERVICE_NAME])
    while self.scripts_installed_at_start == None and not rospy.is_shutdown():
        self.scripts_installed_at_start = nepi.get_installed_scripts(self.get_installed_scripts_service)
        if self.scripts_installed_at_start == None:
          rospy.loginfo("Service call failed, waiting 1 second then retrying")
          time.sleep(1)
    #rospy.loginfo("Scripts installed at start:")
    #rospy.loginfo(self.scripts_installed_at_start)
    ### Get list of running scripts
    rospy.loginfo("")
    rospy.loginfo("Getting list of running scripts at start")
    rospy.loginfo(["Calling service name: " + AUTO_GET_RUNNING_SCRIPTS_SERVICE_NAME])
    while self.scripts_running_at_start == None and not rospy.is_shutdown():
        self.scripts_running_at_start = nepi.get_running_scripts(self.get_running_scripts_service)
        if self.scripts_running_at_start == None:
          rospy.loginfo("Service call failed, waiting 1 second then retrying")
          time.sleep(1)
    #rospy.loginfo("Scripts running at start:")
    #rospy.loginfo(self.scripts_running_at_start)
    rospy.loginfo("Initialization Complete")
    # Launch scripts from list
    nepi.launch_scripts(SCRIPT_LIST,self.launch_script_service,self.get_installed_scripts_service, \
                        self.get_running_scripts_service)
    ## Initiation Complete
    rospy.loginfo("Initialization Complete")

  #######################
  ### Node Methods
  

  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    rospy.loginfo("Shutting down: Executing script cleanup actions")
    # Stop scripts from list
    nepi.stop_scripts(SCRIPT_LIST,self.stop_script_service,self.get_installed_scripts_service, \
                      self.get_running_scripts_service,self.scripts_running_at_start)


#########################################
# Main
#########################################
if __name__ == '__main__':
  current_filename = sys.argv[0].split('/')[-1]
  current_filename = current_filename.split('.')[0]
  if current_filename[0] == "_":
    current_filename = current_filename[1:]
  rospy.loginfo(("Starting " + current_filename), disable_signals=True) # Disable signals so we can force a shutdown
  rospy.init_node(name=current_filename)
  #Launch the node
  node_name = current_filename.rpartition("_")[0]
  rospy.loginfo("Launching node named: " + node_name)
  node_class = eval(node_name)
  node = node_class()
  #Set up node shutdown
  rospy.on_shutdown(node.cleanup_actions)
  # Spin forever (until object is detected)
  rospy.spin()


