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

# NEPI utility script includes
# 1) Topic Utility Functions
# 2) Script Utility Functions
  

import rospy
import time
import sys


from std_msgs.msg import Empty, Float32



#######################
### Topic Utility Functions

### Sleep process that breaks sleep into smaller times for better shutdown
def sleep(sleep_sec,sleep_steps):
  delay_timer = 0
  delay_sec = sleep_sec/sleep_steps
  while delay_timer < sleep_sec and not rospy.is_shutdown():
    time.sleep(delay_sec)
    delay_timer = delay_timer + delay_sec



### Function to get list of active topics
def get_topic_list():
  topic = ""
  topic_list=rospy.get_published_topics(namespace='/')
  return topic_list

### Function to find a topic from list of strings
def find_topic_from_string_list(topic_string_list):
  topic = ""
  topic_list=rospy.get_published_topics(namespace='/')
  #rospy.loginfo(topic_list)
  for topic_entry in topic_list:
    #rospy.loginfo(topic_entry[0])
    if all(x in topic_entry[0] for x in topic_string_list):
      topic = topic_entry[0]
  return topic

### Function to find a topic
def find_topic(topic_name):
  topic = ""
  topic_list=rospy.get_published_topics(namespace='/')
  #rospy.loginfo(topic_list)
  for topic_entry in topic_list:
    #rospy.loginfo(topic_entry[0])
    if topic_entry[0].find(topic_name) != -1:
      topic = topic_entry[0]
  return topic

### Function to check for a topic 
def check_for_topic(topic_name):
  topic_exists = True
  topic=find_topic(topic_name)
  if topic == "":
    topic_exists = False
  return topic_exists

### Function to wait for a topic
def wait_for_topic(topic_name):
  topic = ""
  while topic == "" and not rospy.is_shutdown():
    topic=find_topic(topic_name)
    time.sleep(.1)
  return topic


#######################
### Script Utility Functions

### Function to get list of installed scripts
def get_installed_scripts(get_installed_scripts_service):
  installed_scripts = None
  try:
    response = get_installed_scripts_service()
    installed_scripts = response.scripts
    #rospy.loginfo("Installed Scripts = " + str(installed_scripts))
  except Exception as e:
    rospy.loginfo("Get installed scripts service call failed: " + str(e))
  return installed_scripts

### Function to get list of running scripts
def get_running_scripts(get_running_scripts_service):
  running_scripts = None
  try:
    response = get_running_scripts_service()
    running_scripts = response.running_scripts
    #rospy.loginfo("Running Scripts = " + str(running_scripts))
  except Exception as e:
    rospy.loginfo("Get running scripts service call failed: " + str(e))
  return running_scripts


### Function to launch a script
def launch_script(script2launch,launch_script_service):
  launch_success=False
  try:
    success = launch_script_service(script=script2launch)
    rospy.loginfo("Launched script: " + str(success))
    launch_success=True
  except Exception as e:
    rospy.loginfo("Launch script service call failed: " + str(e))
  return launch_success

### Function to stop script
def stop_script(script2stop,stop_script_service):
  stop_success=False
  try:
    success = stop_script_service(script=script2stop)
    rospy.loginfo("Stopped script: " + str(success))
    stop_success=True
  except Exception as e:
    rospy.loginfo("Stop script service call failed: " + str(e))
  return stop_success

### Function to start scripts from list
def launch_scripts(script_list,launch_script_service,get_installed_scripts_service,get_running_scripts_service):
  installed_scripts = get_installed_scripts(get_installed_scripts_service)
  running_scripts = get_running_scripts(get_running_scripts_service)
  if installed_scripts is not None and running_scripts is not None:
    for script2launch in script_list:
      script_installed = val_in_list(script2launch,installed_scripts)
      if script_installed:
        script_running = val_in_list(script2launch,running_scripts)
        if script_running is False:
            rospy.loginfo("")
            rospy.loginfo(["Launching script: " + script2launch])
            script_launch = launch_script(script2launch,launch_script_service)
            if script_launch:
              rospy.loginfo("Script launch call success")
              script_running = False
              while script_running is False and not rospy.is_shutdown():
                running_scripts = get_running_scripts(get_running_scripts_service)
                script_running = val_in_list(script2launch,running_scripts)
                rospy.loginfo("Waiting for script to launch")
                time.sleep(.5) # Sleep before checking again
              rospy.loginfo("Script started successfully")
            else:
               rospy.loginfo("Scipt launch call failed")
        else:
          rospy.loginfo("Script already running, skipping launch process")
      else:
        rospy.loginfo("Script not found, skipping launch process")
  else:
    rospy.loginfo("Failed to get installed and running script list")
  #running_scripts = get_running_scripts()
  #rospy.loginfo(running_scripts)
          

### Function to stop scripts from list, a
def stop_scripts(script_list,stop_script_service,get_installed_scripts_service,get_running_scripts_service,optional_ignore_script_list=[]):
  installed_scripts = get_installed_scripts(get_installed_scripts_service)
  running_scripts = get_running_scripts(get_running_scripts_service)
  if installed_scripts is not None and running_scripts is not None:
    for script2stop in script_list:
      script_running = val_in_list(script2stop,running_scripts)
      script_ignore = val_in_list(script2stop,optional_ignore_script_list)
      if script_running is True and script_ignore is False:
        script_running = val_in_list(script2stop,running_scripts)
        if script_running is True:
            rospy.loginfo("")
            rospy.loginfo(["Stopping script: " + script2stop])
            script_stop = stop_script(script2stop,stop_script_service)
            if script_stop:
              rospy.loginfo("Script stop call success")
            else:
               rospy.loginfo("Scipt stop call failed")
        else:
          rospy.loginfo("Scipt in ignore list, skipping launch process")
      else:
        rospy.loginfo("Script not found, skipping launch process")
  else:
    rospy.loginfo("Failed to get installed and running script list")
  #running_scripts = get_running_scripts()
  #rospy.loginfo(running_scripts)
    

### Function for checking if val in list
def val_in_list(val2check,list2check):
  in_list = False
  if len(list2check) > 0:
    for list_val in list2check:
      #rospy.loginfo(str(val2check) + ' , ' + str(list_val))
      #rospy.loginfo(val2check == list_val)
      if val2check == list_val:
        in_list = True
  return in_list
 

  