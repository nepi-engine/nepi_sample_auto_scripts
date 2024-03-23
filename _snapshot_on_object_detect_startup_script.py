#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

# Sample Solution Startup Script. 
# 1. Launches scripts from list that are not running
# 2. Waits for shutdown
# 3. Stops scripts that were launched by this script

import rospy
import sys
import time
from resources import nepi

#########################################
# USER SETTINGS - Edit as Necessary 
#########################################

SCRIPT_LIST = ["ai_detector_config_script.py",
               "navpose_set_fixed_config_script.py",
               "snapshot_on_object_detect_process_script.py",
               "snapshot_event_save_to_disk_action_script.py",
               "snapshot_event_send_to_cloud_action_script.py"] #  Script filenames to start/stop



#########################################
# ROS NAMESPACE SETUP
#########################################

# ROS namespace setup
NEPI_BASE_NAMESPACE = nepi.get_base_namespace()

#########################################
# Node Class
#########################################

class snapshot_on_object_detect_startup(object):

  #######################
  ### Node Initialization
  def __init__(self):
    rospy.loginfo("Starting Initialization Processes")
    nepi.startup_script_initialize(self,NEPI_BASE_NAMESPACE)
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


