#!/usr/bin/env python
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


# Sample NEPI Helper Script. 
# 1. Cycles through all or list of scripts and replaces line based line starting with org_word_string
# NOTE: Must be run from command line with sudo
# NOTE: You should make a copy of your scipts in a "temp" folder and run there

import os
import sys
import glob
import fileinput


#####################################################################################
# SETUP - Edit as Necessary ##################################
##########################################
SCRIPT_FOLDER = "/mnt/nepi_storage/automation_scripts/"
SCRIPT_LIST = [] # Leave empty to update all files in folder. Add files to limit which files are updated

FIND_REPLACE_LIST = [["resources import nepi","nepi_edge_sdk_base import nepi_ros "],
                     ["resources import nepi_navpose","nepi_edge_sdk_base import nepi_nav "],
                     ["nepi.","nepi_ros."]]


#####################################################################################
# Methods
#####################################################################################

### Function for updating values in automation script files (All or From List)
def update_script_strings(script_folder_path,find_replace_list,optional_script_list):
  filelist=os.listdir(script_folder_path)
  if len(optional_script_list) != 0: # Just These Scripts
    scriptlist=[]
    for ind, file in enumerate(filelist):
      for script in optional_script_list:
        if file.find(script)!= -1:
          scriptlist.append([script_folder_path + file])
  else: # All Scripts
    scriptlist=[]
    for ind, file in enumerate(filelist):
     if file.find(".py") != -1:
       scriptlist.append([script_folder_path + file])
  print("Checking and updating the following files")
  print(scriptlist)
  for file in scriptlist:
    print("")
    print(file)
    lnum = []
    for line_ind, line in enumerate(fileinput.input(file, inplace=1)):
      for pair in find_replace_list:
        if line.find(pair[0]) != -1:
          line = line.replace(pair[0],pair[1])
          lnum.append(line_ind+1)
      print(line[0:-1])
    print('Replaced line numbers')
    print(lnum)

#####################################################################################
# Main
#####################################################################################

if __name__ == '__main__':
  update_script_strings(SCRIPT_FOLDER,FIND_REPLACE_LIST,SCRIPT_LIST)

