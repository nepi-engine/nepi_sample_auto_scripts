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


# Sample AI training data partioning script. 
# Uses onboard python libraries to
# 1. Search all of the subfolders in the same folder as the script
# for image files
# 2) Trys to open the file. If opening the file fails, it deletes the file
# along with its bounding box .xml file if it exists

import os
from os.path import exists
from PIL import Image

import subprocess
import sys
sys.tracebacklimit = None
import glob
import fileinput
import random


##########################################
# SETUP - Edit as Necessary 
##########################################
TEST_DATA_PERCENTAGE = 20

##########################################
# Methods
##########################################

def check_data_set(image_dir):
  path, dirs, files = next(os.walk(image_dir))
  data_size = len(files)
  ind = 0
  for f in os.listdir(image_dir):
    if f.endswith("png"):
      #print('Found image file')
      image_file = (image_dir + '/' + f)
      #print(image_file)
      # Open and verify the image file
      try:
        img = Image.open(image_file) # open the image file
        img.verify()
        #print('Good file')
      except:
        print('')
        print('Found bad image file:')
        print(image_file) # print out the names of corrupt files
        print('Deleting file')
        os.remove(image_file)
        print('Looking for label file')
        label_file = (image_dir + '/' + f.split(".png")[0]+'.xml')
        #print(label_file)
        if exists(label_file):
          print('Found xml label file for bad image:')
          print(label_file) # print out the names of corrupt files
          print('Deleting file')
          os.remove(label_file)
        else:
          print('No label file found for bad image')
            
def get_folder_list(script_folder_path):
  filelist=os.listdir(script_folder_path + '/')
  folder_list=[]
  #print('')
  #print('Files and Folders in Path:')
  #print(script_folder_path)
  #print(filelist)
  for file in enumerate(filelist):
    foldername = (script_folder_path + '/' + file[1])
    #print('Checking file: ')
    #print(foldername)
    if os.path.isdir(foldername): # file is a folder
       folder_list.append(foldername)
  return folder_list


###############################################
# Main
###############################################

if __name__ == '__main__':
  abs_path = os.path.realpath(__file__)
  script_path = os.path.dirname(abs_path)
  ### Check for corrupt image files in folders
  print(script_path)
  folders_to_process=get_folder_list(script_path)
  print('')
  print('Found folders in script directory:')
  print(folders_to_process)
  for folder in folders_to_process:
    print('Evaluting data in folder:')
    print(folder)
    check_data_set(folder)
  ### Wrap Up
  print('')
  print('All done')




