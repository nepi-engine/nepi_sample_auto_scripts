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
# for xml label files
# 2) Tries to create Darknet txt label files from xml label files

import os
from os.path import exists

import logging
import declxml as xml



##########################################
# SETUP - Edit as Necessary 
##########################################
TEST_DATA_PERCENTAGE = 20

##########################################
# Methods
##########################################

def convert_xml_files(xml_dir):
  transformer = Transformer(xml_dir)
  transformer.transform()
            
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


##########################################
# Classes
##########################################


class Transformer(object):
    def __init__(self, xml_dir):
        self.xml_dir = xml_dir
        self.out_dir = xml_dir
        self.class_file = (xml_dir + '/classes.txt')

    def transform(self):
        reader = Reader(xml_dir=self.xml_dir)
        xml_files = reader.get_xml_files()
        print(xml_files)
        classes = reader.get_classes(self.class_file)
        print(classes)
        object_mapper = ObjectMapper()
        annotations = object_mapper.bind_files(xml_files, xml_dir=self.xml_dir)
        self.write_to_txt(annotations, classes)

    def write_to_txt(self, annotations, classes):
        for annotation in annotations:
            output_path = os.path.join(self.out_dir, self.darknet_filename_format(annotation.filename))
            if not os.path.exists(os.path.dirname(output_path)):
                os.makedirs(os.path.dirname(output_path))
            with open(output_path, "w+") as f:
                f.write(self.to_darknet_format(annotation, classes))

    def to_darknet_format(self, annotation, classes):
        result = []
        for obj in annotation.objects:
            if obj.name not in classes:
                print("Please, add '%s' to classes.txt file." % obj.name)
                exit()
            x, y, width, height = self.get_object_params(obj, annotation.size)
            result.append("%d %.6f %.6f %.6f %.6f" % (classes[obj.name], x, y, width, height))
        return "\n".join(result)

    @staticmethod
    def get_object_params(obj, size):
        image_width = 1.0 * size.width
        image_height = 1.0 * size.height

        box = obj.box
        absolute_x = box.xmin + 0.5 * (box.xmax - box.xmin)
        absolute_y = box.ymin + 0.5 * (box.ymax - box.ymin)

        absolute_width = box.xmax - box.xmin
        absolute_height = box.ymax - box.ymin

        x = absolute_x / image_width
        y = absolute_y / image_height
        width = absolute_width / image_width
        height = absolute_height / image_height

        return x, y, width, height

    @staticmethod
    def darknet_filename_format(filename):
        pre, ext = os.path.splitext(filename)
        return "%s.txt" % pre


class Reader(object):
    def __init__(self, xml_dir):
        self.xml_dir = xml_dir

    def get_xml_files(self):
        xml_filenames = []
        for root, subdirectories, files in os.walk(self.xml_dir):
            for filename in files:
                if filename.endswith(".xml"):
                    file_path = os.path.join(root, filename)
                    file_path = os.path.relpath(file_path, start=self.xml_dir)
                    xml_filenames.append(file_path)    
        return xml_filenames

    @staticmethod
    def get_classes(filename):
        with open(os.path.join(os.path.dirname(os.path.realpath('__file__')), filename), "r", encoding="utf8") as f:
            lines = f.readlines()
            return {value: key for (key, value) in enumerate(list(map(lambda x: x.strip(), lines)))}

import logging
import os
import declxml as xml


class ObjectMapper(object):
    def __init__(self):
        self.processor = xml.user_object("annotation", Annotation, [
            xml.user_object("size", Size, [
                xml.integer("width"),
                xml.integer("height"),
            ]),
            xml.array(
                xml.user_object("object", Object, [
                    xml.string("name"),
                    xml.user_object("bndbox", Box, [
                        xml.integer("xmin"),
                        xml.integer("ymin"),
                        xml.integer("xmax"),
                        xml.integer("ymax"),
                    ], alias="box")
                ]),
                alias="objects"
            ),
            xml.string("filename")
        ])

    def bind(self, xml_file_path, xml_dir):
        ann = xml.parse_from_file(self.processor, xml_file_path=os.path.join(xml_dir, xml_file_path))
        ann.filename = xml_file_path
        return ann

    def bind_files(self, xml_file_paths, xml_dir):
        result = []
        for xml_file_path in xml_file_paths:
            try:
                result.append(self.bind(xml_file_path=xml_file_path, xml_dir=xml_dir))
            except Exception as e:
                logging.error("%s", e.args)
        return result


class Annotation(object):
    def __init__(self):
        self.size = None
        self.objects = None
        self.filename = None

    def __repr__(self):
        return "Annotation(size={}, object={}, filename={})".format(self.size, self.objects, self.filename)


class Size(object):
    def __init__(self):
        self.width = None
        self.height = None

    def __repr__(self):
        return "Size(width={}, height={})".format(self.width, self.height)


class Object(object):
    def __init__(self):
        self.name = None
        self.box = None

    def __repr__(self):
        return "Object(name={}, box={})".format(self.name, self.box)


class Box(object):
    def __init__(self):
        self.xmin = None
        self.ymin = None
        self.xmax = None
        self.ymax = None

    def __repr__(self):
        return "Box(xmin={}, ymin={}, xmax={}, ymax={})".format(self.xmin, self.ymin, self.xmax, self.ymax)

###############################################
# Main
###############################################

if __name__ == '__main__':
  abs_path = os.path.realpath(__file__)
  script_path = os.path.dirname(abs_path)
  ### Converting XML Label Files to TXT
  print(script_path)
  folders_to_process=get_folder_list(script_path)
  print('')
  print('Found folders in script directory:')
  print(folders_to_process)
  for folder in folders_to_process:
    classes_file = (folder + '/classes.txt')
    if exists(classes_file):
      print('converting xml files in folder:')
      print(folder)
      convert_xml_files(folder)
    else:
      print('no classes file found in folder, skipping')
  ### Wrap Up
  print('')
  print('All done')




