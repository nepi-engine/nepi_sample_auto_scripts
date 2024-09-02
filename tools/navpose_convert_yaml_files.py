#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

# Script to convert navpose yaml files into human readable
# Uses onboard python libraries to
# 1. Search for yaml files create by nav_pose_mgr_nav
# 2. Converts those yaml files to human readable data

import os
import yaml
import rospy
from nepi_edge_sdk_base import nepi_ros
from nepi_edge_sdk_base import nepi_nav
from std_msgs.msg import Float64MultiArray
from nepi_ros_interfaces.srv import NavPoseQuery, NavPoseQueryRequest
from nepi_ros_interfaces.msg import NavPose, NavPoseResponse


##########################################
# SETUP - Edit as Necessary 
##########################################
ENU_ORIENTATION_ENABLE = False
NED_ORIENTATION_ENABLE = True
WGS84_GEO_ENABLE = True
AMSL_GEO_ENABLE = False

NEPI_BASE_NAMESPACE = nepi_ros.get_base_namespace()
NAVPOSE_SERVICE_NAME = NEPI_BASE_NAMESPACE + "nav_pose_query"

def convert_yaml_files(yaml_dir):
  transformer = Transformer(yaml_dir)
  transformer.transform()

class Transformer(object):
    def __init__(self, yaml_dir):
        self.yaml_dir = yaml_dir
        self.out_dir = yaml_dir
    
    def transform(self):
        reader = Reader(yaml_dir=self.yaml_dir)
        yaml_files = reader.get_yaml_files()
        if len(yaml_files) != 0:
            for file in yaml_files:
                parser = YamlParser(file)
        else:
            print("No .yaml files in specified directory")


class YamlParser(object):
    def __init__(self, yaml_file):
        self.yaml_file = os.path.join(r'/mnt/nepi_storage/data', yaml_file)
        self.data = self.load_yaml()

    def load_yaml(self):
        with open(self.yaml_file, 'r') as file:
            data = yaml.safe_load(file)
        nav_pose_data = self.add_yaml_data(data)
        self.convert_yaml(data, nav_pose_data)
        return data
    
    def add_yaml_data(self, data):
        nav_pose_response = NavPoseResponse()

        entry = data.get('data', {})
        if 'gps' in entry:
            location = entry['gps']
            nav_pose_response.nav_pose.fix.latitude = location['latitude']
            nav_pose_response.nav_pose.fix.longitude = location['longitude']
            nav_pose_response.nav_pose.fix.altitude = location['altitude']
        if 'odometry' in entry and 'position' in entry['odometry']:
            position = entry['odometry']['position']
            nav_pose_response.nav_pose.odom.pose.pose.position.x = position['x']
            nav_pose_response.nav_pose.odom.pose.pose.position.y = position['y']
            nav_pose_response.nav_pose.odom.pose.pose.position.z = position['z']
        if 'odometry' in entry and 'orientation' in entry['odometry']:
            orientation = entry['odometry']['orientation']
            nav_pose_response.nav_pose.odom.pose.pose.orientation.x = orientation['quaternion_x']
            nav_pose_response.nav_pose.odom.pose.pose.orientation.y = orientation['quaternion_y']
            nav_pose_response.nav_pose.odom.pose.pose.orientation.z = orientation['quaternion_z']
            nav_pose_response.nav_pose.odom.pose.pose.orientation.w = orientation['quaternion_w']
        
        #print(nav_pose_response)
        return nav_pose_response
            


    def convert_yaml(self, data, nav_pose_data):  
        entry = data.get('data', {})
        if 'gps' in entry:
            current_geoid_height = nepi_nav.get_navpose_geoid_height(nav_pose_data)
            location = entry['gps']
            location['geoid height'] = current_geoid_height
            if WGS84_GEO_ENABLE:
                current_location_wgs84_geo = Float64MultiArray()
                current_location_wgs84_geo.data = nepi_nav.get_navpose_location_wgs84_geo(nav_pose_data) 

                latitude, longitude, altitude = current_location_wgs84_geo.data
                location['latitude'] = latitude
                location['longitude'] = longitude
                location['altitude'] = altitude
            if AMSL_GEO_ENABLE:
                current_location_amsl_geo = Float64MultiArray()
                current_location_amsl_geo.data =  nepi_nav.get_navpose_location_amsl_geo(nav_pose_data)

                latitude, longitude, altitude = current_location_amsl_geo.data
                location['latitude'] = latitude
                location['longitude'] = longitude
                location['altitude'] = altitude

        if 'odometry' in entry and 'orientation' in entry['odometry']:
            orientation = entry['odometry']['orientation']
            if ENU_ORIENTATION_ENABLE:
                current_orientation_enu_degs = Float64MultiArray()
                current_orientation_enu_degs.data = nepi_nav.get_navpose_orientation_enu_degs(nav_pose_data)

                roll, pitch, yaw = current_orientation_enu_degs.data

                roll = float(roll)
                pitch = float(pitch)
                yaw = float(yaw)

                orientation['roll'] = roll
                orientation['pitch'] = pitch
                orientation['yaw'] = yaw
        
                # Remove quaternion keys
                del orientation['quaternion_x']
                del orientation['quaternion_y']
                del orientation['quaternion_z']
                del orientation['quaternion_w']

            if NED_ORIENTATION_ENABLE:
                current_orientation_ned_degs = Float64MultiArray()
                current_orientation_ned_degs.data = nepi_nav.get_navpose_orientation_ned_degs(nav_pose_data)

                roll, pitch, yaw = current_orientation_ned_degs.data
                    
                roll = float(roll)
                pitch = float(pitch)
                yaw = float(yaw)
 
                orientation['roll'] = roll
                orientation['pitch'] = pitch
                orientation['yaw'] = yaw

                # Remove quaternion keys
                del orientation['quaternion_x']
                del orientation['quaternion_y']
                del orientation['quaternion_z']
                del orientation['quaternion_w']

        if 'odometry' in entry and 'position' in entry['odometry']:
            position = entry['odometry']['position']
            if ENU_ORIENTATION_ENABLE:
                current_position_enu_m = Float64MultiArray()
                current_position_enu_m.data = nepi_nav.get_navpose_position_enu_m(nav_pose_data)
    
                x, y, z = current_position_enu_m.data

                position['x (east)'] = x
                position['y (north)'] = y
                position['z (up)'] = z

                del position['x']
                del position['y']
                del position['z']

            if NED_ORIENTATION_ENABLE:
                current_position_ned_m = Float64MultiArray()
                current_position_ned_m.data = nepi_nav.get_navpose_position_ned_m(nav_pose_data)

                x, y, z = current_position_ned_m.data

                position['x (north)'] = x
                position['y (east)'] = y
                position['z (down)'] = z

                del position['x']
                del position['y']
                del position['z']

        base_name = os.path.basename(self.yaml_file)
        dir_name = os.path.dirname(self.yaml_file)
        file_name, file_ext = os.path.splitext(base_name)
        new_filename = os.path.join(dir_name, f"{file_name}_converted{file_ext}")
        with open(new_filename, 'w') as file:
            yaml.safe_dump(data, file, sort_keys=False)

class Reader(object):
    def __init__(self, yaml_dir):
        self.yaml_dir = yaml_dir
    
    def get_yaml_files(self):
        yaml_filenames = []
        for root, subdirectories, files in os.walk(self.yaml_dir):
            for filename in files:
                if "_converted" in filename:
                    pass
                else:
                    if filename.endswith(".yaml"):
                        file_path = os.path.join(root, filename)
                        file_path = os.path.relpath(file_path, start=self.yaml_dir)
                        yaml_filenames.append(file_path)
                
        return yaml_filenames
    
if __name__ == '__main__':
    data_dir = '/mnt/nepi_storage/data'
    ### Converting Yaml files to readable
    convert_yaml_files(data_dir)
    
    

        