#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

# Sample AI training data partitioning script. 
# Uses onboard python libraries to
# 1. Search all of the subfolders in the same folder as the script
# for labeled image data
# 2) Create randomized AI model train and test data pointer files
# in the same folder as the script named "data_train.txt" and "data_test.txt"
# based on user set partitioning percentages

import os
from os.path import exists
import subprocess
import sys
import glob
import fileinput
import random

IMAGE_FILE_TYPES = ['jpg','JPG','jpeg','png','PNG']

##########################################
# SETUP - Edit as Necessary 
##########################################
TEST_DATA_PERCENTAGE = 20

##########################################
# Methods
##########################################

def add_data_set(image_dir,f_train,f_test,f_unlabeled):
  path, dirs, files = next(os.walk(image_dir))
  data_size = len(files)
  ind = 0
  data_test_size = int(float(1)/float(TEST_DATA_PERCENTAGE) * data_size)
  test_array = random.sample(range(data_size), k=data_test_size)
  files = os.listdir(image_dir)
  #print("Found Files:")
  #print(files)
  for f in files:
    f_ext = os.path.splitext(f)[1]
    f_ext = f_ext.replace(".","")
    try:
      if f_ext in IMAGE_FILE_TYPES:
        print('Found image file"')
        image_file = (image_dir + '/' + f)
        #print(image_file)
        label_file = (image_dir + '/' + f.split(f_ext)[0]+'txt')
        #print('Looking for label file:')
        #print(label_file)
        if exists(label_file):
          #print('Found label file')
          ind += 1
          if ind in test_array:
            #print('Adding image to test file list')
            f_test.write(image_file + '\n')
          else:
            #print('Adding image to train file list')
            f_train.write(image_file + '\n')
        else:
          print("Warning: No label file for image:")
          print(image_file)
          print('Adding image to data_unlabeled file list')
          f_unlabeled.write(image_file + '\n')
    except Exception as e:
      print("Excepton on file write: " + str(e))
            
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

def open_new_file(file):
  print('')
  if os.path.isfile(file):
    print('Deleting existing file:')
    print(file)
    os.remove(file)
  print('Creating new file:')
  print(file)
  fnew = open(file, 'w')
  return fnew

###############################################
# Main
###############################################

if __name__ == '__main__':
  abs_path = os.path.realpath(__file__)
  script_path = os.path.dirname(abs_path)
  print('**************')
  ### Create new train data set file
  file_train = (script_path +'/'+ "data_train.txt")
  fnew_train = open_new_file(file_train)
  ### Create new test data set file
  file_test = (script_path +'/'+ "data_test.txt")
  fnew_test = open_new_file(file_test)
  ### Create new unlabeled data set file
  file_unlabeled = (script_path +'/'+ "data_unlabeled.txt")
  fnew_unlabled = open_new_file(file_unlabeled)

  ### Add files from folders
  print(script_path)
  folders_to_process=get_folder_list(script_path)
  print('')
  print('Found folders in script directory:')
  print(folders_to_process)
  for folder in folders_to_process:
    print('Evaluting data in folder:')
    print(folder)
    add_data_set(folder,fnew_train,fnew_test,fnew_unlabled)

  ### Wrap Up
  print('')
  print('All done')




