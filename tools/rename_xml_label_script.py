#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#


import os
import xml.etree.ElementTree as ET


##########################################
# SETUP - Edit as Necessary 
##########################################
Orig_Label = 'Hard Hat'
New_Label = "HardHat"
##########################################
# Methods
##########################################
def rename_xml_labels(orig_label,new_label,source_folder,out_folder=None):
  if out_folder==None:
    out_folder = source_folder
  if os.path.exists(out_folder) == False:
    try:
      os.makedirs(out_folder)
    except Exception as e:
      print("Failed to create out_folder: " + out_folder + " with exception "  + str(e))
      return
  print('Renaming labels in xml files in folder: ' + source_folder)
  for f in os.listdir(source_folder):
    if f.endswith(".xml"):  
      #print(f)
      src_file = source_folder + '/' + f
      out_file = out_folder  + '/' + f
      tree = ET.parse(src_file)
      root = tree.getroot()
      for ind, o_entry in enumerate(root.findall("object")):
        label = o_entry.find("name").text
        if label == orig_label:
          o_entry.find("name").text = New_Label
        label = o_entry.find("name").text
        #print(label)
      tree.write(out_file)

        


        
           
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
  ### Renaming XML Label Files to TXT
  print(script_path)
  folders_to_process=get_folder_list(script_path)
  print('')
  print('Found folders in script directory:')
  print(folders_to_process)
  for source_folder in folders_to_process:
      out_folder = source_folder + "/new_xml"
      rename_xml_labels(Orig_Label,New_Label,source_folder)
  ### Wrap Up
  print('')
  print('All done')




