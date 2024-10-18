#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

# Sample NEPI Process Script. 
# 1) 

# Requires the following additional scripts are running
# a)

import rospy
import time
import sys
from nepi_edge_sdk_base import nepi_ros 
from nepi_edge_sdk_base import nepi_msg 

from std_msgs.msg import Empty, Float32

#########################################
# USER SETTINGS - Edit as Necessary 
#########################################


#########################################
# ROS NAMESPACE SETUP
#########################################

#########################################
# Node Class
#########################################

class blankAutoScript(object):

  #######################
  ### Node Initialization
  DEFAULT_NODE_NAME = "auto_blank" # Can be overwitten by luanch command
  def __init__(self):
    #### AUTO SCRIPT INIT SETUP ####
    nepi_ros.init_node(name= self.DEFAULT_NODE_NAME)
    self.node_name = nepi_ros.get_node_name()
    self.base_namespace = nepi_ros.get_base_namespace()
    nepi_msg.createMsgPublishers(self)
    nepi_msg.publishMsgInfo(self,"Starting Initialization Processes")
    ##############################

    ## Initialize Class Variables
    ## Define Class Namespaces
    ## Define Class Services Calls
    ## Create Class Sevices    
    ## Create Class Publishers
    ## Start Class Subscribers
    ## Start Node Processes
    ## Initiation Complete

    #########################################################
    ## Initiation Complete
    nepi_msg.publishMsgInfo(self,"Initialization Complete")
    #Set up node shutdown
    nepi_ros.on_shutdown(self.cleanup_actions)
    # Spin forever (until object is detected)
    #nepi_ros.spin()
    #########################################################

  #######################
  ### Node Methods
  

  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    nepi_msg.publishMsgInfo(self,"Shutting down: Executing script cleanup actions")


#########################################
# Main
#########################################
if __name__ == '__main__':
  blankAutoScript()

