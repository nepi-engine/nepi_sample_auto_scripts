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

# Sample NEPI Driver Script. 
# NEPI LSX Driver Script for SeaLite LED Lights


### Set the namespace before importing rospy
import os
os.environ["ROS_NAMESPACE"] = "/nepi/s2x"
import rospy
import serial
import serial.tools.list_ports
import time
import re

from std_msgs.msg import UInt8, Empty, String, Bool, Float32
from nepi_ros_interfaces.msg import LSXStatus
from nepi_ros_interfaces.srv import LSXCapabilitiesQuery, LSXCapabilitiesQueryResponse


#########################################
# DRIVER SETTINGS
#########################################

#Define Discovery Search Parameters
BAUDRATE_LIST = [9600,19200,57600] # Three supported buad rates
ADDRESS_LIST = list(range(1,10))  # Total range 1-255


#########################################
# ROS NAMESPACE SETUP
#########################################

NEPI_BASE_NAMESPACE = "/nepi/s2x/"


#########################################
# Sealite LSX Driver Node Class
#########################################

class Sealite_Node(object):
  #######################
  ### LXS Driver Settings
  # Set driver capability parameters
  HAS_STANDBY_MODE = True
  HAS_INTENSITY_CONTROL = True
  HAS_HW_STROBE_CONTROL = True
  REPORTS_TEMP = True
  # Set Initial LSX Parameters
  STANDBY = False
  INTENSITY = 0.0
  STROBE_ENABLE = False

  # Define some class variables
  ser_port_str = ""
  ser_buad_int = 0
  dev_addr_str = 0
  connected = False
  device_name = ""
  node_name = ""

  #######################
  ### LXS Driver NODE Initialization
  def __init__(self,port_str,buad_int,addr_str):
    self.ser_port_str = port_str
    self.ser_buad_int = buad_int
    self.dev_addr_str = addr_str
    print("")
    print("Starting Initialization")
    # Create LSX Namespaces
    port_str = self.ser_port_str.split("tty")[1]
    self.device_name = "sealite_" + port_str + "_" + self.dev_addr_str
    NEPI_LSX_BASENAME = rospy.get_namespace() + self.device_name  + "/lsx/"
    NEPI_LSX_CAPABILITY_REPORT_SERVICE = NEPI_LSX_BASENAME + self.device_name + "capabilities_query"
    NEPI_LSX_STATUS_TOPIC = NEPI_LSX_BASENAME + "status"
    NEPI_LSX_SET_STANDBY_TOPIC = NEPI_LSX_BASENAME + "set_standby"
    NEPI_LSX_SET_INTENSITY_TOPIC = NEPI_LSX_BASENAME + "set_intensity"
    NEPI_LSX_SET_STROBE_ENABLE_TOPIC = NEPI_LSX_BASENAME + "set_strobe_enable"
    ### LXS Global Variables
    self.serial_port = None
    self.serial_busy = False
    self.lxs_capabilities_report = LSXCapabilitiesQueryResponse()
    self.lxs_status_pub = rospy.Publisher(NEPI_LSX_STATUS_TOPIC, LSXStatus, queue_size=1, latch=True)
    # Initialize some parameters
    self.serial_port = None
    self.serial_num = ""
    self.hw_version = ""
    self.sw_version = ""
    self.standby = self.STANDBY
    self.intensity = self.INTENSITY
    self.strobe_enable = self.STROBE_ENABLE
    temp_c = 0.0
    ### Try and connect to device
    self.connected = self.lsx_connect()
    if self.connected:
      # Initialize Device settings
      self.set_standby(self.STANDBY)
      self.set_intensity(self.INTENSITY)
      self.set_strobe_enable(self.STROBE_ENABLE)
      
      # Create LSX ROS node
      print("Starting " + self.device_name)
      rospy.init_node(self.device_name)
      print("Created node " + rospy.get_name())
      self.node_name = rospy.get_name().split('/')[-1]

      # Publish status message
      self.lsx_status_callback()
      # Populate and advertise LSX node capabilities report
      self.lsx_capabilities_report = LSXCapabilitiesQueryResponse()
      self.lsx_capabilities_report.has_standby_mode = self.HAS_STANDBY_MODE
      self.lsx_capabilities_report.has_intensity_control = self.HAS_INTENSITY_CONTROL
      self.lsx_capabilities_report.has_hw_strobe = self.HAS_HW_STROBE_CONTROL
      self.lsx_capabilities_report.reports_temperature = self.REPORTS_TEMP
      rospy.Service(NEPI_LSX_CAPABILITY_REPORT_SERVICE, LSXCapabilitiesQuery, self.lsx_capabilities_query_callback)
      # Start LSX node control subscribers
      print("Starting LSX control subscribers")
      rospy.Subscriber(NEPI_LSX_SET_STANDBY_TOPIC, Bool, self.lsx_set_standby_callback, queue_size = 1)
      rospy.Subscriber(NEPI_LSX_SET_INTENSITY_TOPIC, Float32, self.lsx_set_intensity_callback, queue_size = 1)
      rospy.Subscriber(NEPI_LSX_SET_STROBE_ENABLE_TOPIC, Bool, self.lsx_set_strobe_enable_callback, queue_size = 1)
      # Initialization Complete
      print("Initialization Complete")
      #rospy.spin()

  #######################
  ### Class Functions and Callbacks
    
  ### Function to try and connect to device at given port and buadrate
  def lsx_connect(self):
    success = False
    print("")
    try:
      # Try and open serial port
      print("Opening serial port " + self.ser_port_str + " with buadrate: " + str(self.ser_buad_int))
      self.serial_port = serial.Serial(self.ser_port_str,self.ser_buad_int,timeout = 0.1)
      print("Serial port opened")
      # Send Message
      print("Requesting info for device: " + self.dev_addr_str)
      ser_msg = ('!' + self.dev_addr_str + ':INFO?')
      print("Sending serial string: " + ser_msg)
      response = self.send_msg(ser_msg)
      if len(response) > 2:
        ret_addr = response[0:3]
        print("Returned address value: " + ret_addr)
        if ret_addr == self.dev_addr_str:
          print("Connected to device at address: " +  self.dev_addr_str)
          res_split = response.split(',')
          if len(res_split) > 4:
          # Update serial, hardware, and software status values
            self.serial_num = res_split[2]
            self.hw_version = res_split[3]
            self.sw_version = res_split[4]
          success = True
        else:
          print("Device returned address: " + ret_addr + " does not match: " +  self.dev_addr_str)
      else:
        print("Device returned invalid response")    
    except Exception as e:
      print("Something went wrong with connect function at serial port at: " + self.ser_port_str)
      print(e)
    return success

  ### callback to provide capabilities report ###
  def lsx_capabilities_query_callback(self, _):
    return self.lsx_capabilities_report

  ### Status callback
  def lsx_status_callback(self):
    # update status values from device
    success=self.update_status_values()
    # Create LSX status message
    status_msg=LSXStatus()
    status_msg.serial_num = self.serial_num
    status_msg.hw_version = self.hw_version
    status_msg.sw_version = self.sw_version
    status_msg.standby = self.standby
    status_msg.intensity = self.intensity
    status_msg.strobe_enable = self.strobe_enable
    status_msg.temp_c = self.temp_c
    if not rospy.is_shutdown():
      self.lxs_status_pub.publish(status_msg)

  ### Function to upadate status data
  def update_status_values(self):
    success = True
    print("")
    # Update standby status
    print("Updating standby setting")
    ser_msg= ('!' + self.dev_addr_str + ':STBY?')
    response = self.send_msg(ser_msg)
    if response == "0":
      self.standby = False
    elif response == "1":
      self.standby = True
    else:
      success = False
    if success:
      print("Standby: " + str(self.standby))
    # Update intensity status
    print("Updating intensity setting")
    ser_msg= ('!' + self.dev_addr_str + ':LOUT?')
    response = self.send_msg(ser_msg)
    try:
      self.intensity = float(response)/100
      print("Intensity Ratio: " + str(self.intensity))
    except Exception as i:
      print("Level response was not valid number")
      success = False
    # Update strobe enable status
    print("Updating strobe enable setting")
    ser_msg= ('!' + self.dev_addr_str + ':PMOD?')
    response = self.send_msg(ser_msg)
    if response == "0":
      self.strobe_enable = False
    elif response == "1" or response == "2":
     self.strobe_enable = True
    else:
      success = False
    if success:
      print("Strobe Enable: " + str(self.strobe_enable))
    # Update temp status
    print("Updating temp setting")
    ser_msg= ('!' + self.dev_addr_str + ':TEMP?')
    response = self.send_msg(ser_msg)
    try:
      self.temp_c = float(response)
      print("Temp Deg C: " + str(self.temp_c))
    except Exception as t:
      print("Temp response was not valid number")
      success = False
    return success

  ### Set standby callback
  def lsx_set_standby_callback(self, standby_msg):
    print("Recieved standby message")
    print(standby_msg)
    standby=standby_msg.data
    success=self.set_standby(standby)
    self.lsx_status_callback()

  ### Function for setting standby mode
  def set_standby(self,standby_val):
    success = False
    if standby_val == True:
      ser_msg= ('!' + self.dev_addr_str + ':STBY=1')
    else:
      ser_msg= ('!' + self.dev_addr_str + ':STBY=0')
    response = self.send_msg(ser_msg)
    if response == "ACK":
      success = True
    return success

  ### Set intensity callback
  def lsx_set_intensity_callback(self, intensity_msg):
    print("Recieved intensity message")
    print(intensity_msg)
    intensity=intensity_msg.data
    success=self.set_intensity(intensity)
    self.lsx_status_callback()

  ### Function for setting standby mode
  def set_intensity(self,intensity_ratio):
    success = False
    if intensity_ratio < 0:
      intensity_ratio = 0
    elif intensity_ratio > 1:
      intensity_ration = 1
    level_val = int(100*intensity_ratio)
    level_str = str(level_val)
    zero_prefix_len = 3-len(level_str)
    for z in range(zero_prefix_len):
      level_str = ('0' + level_str)
    ser_msg= ('!' + self.dev_addr_str + ':LOUT=' +  level_str)
    response = self.send_msg(ser_msg)
    if response == "ACK":
      success = True
    return success

  ### Set strobe enable callback
  def lsx_set_strobe_enable_callback(self, strobe_enable_msg):
    print("Recieved strobe enable message")
    print(strobe_enable_msg)
    strobe_enable=strobe_enable_msg.data
    success=self.set_strobe_enable(strobe_enable)
    self.lsx_status_callback()

  ### Function for setting strobe enable mode
  def set_strobe_enable(self,strobe_enable_val):
    success = False
    if strobe_enable_val == True:
      ser_msg= ('!' + self.dev_addr_str + ':PMOD=1')
    else:
      ser_msg= ('!' + self.dev_addr_str + ':PMOD=0')
    response = self.send_msg(ser_msg)
    if response == "ACK":
      success = True
    return success  

  def send_msg(self,ser_msg):
    response = ""
    if self.serial_port is not None:
      ser_str = (ser_msg + '\r\n')
      b=bytearray()
      b.extend(map(ord, ser_str))
      try:
        while self.serial_busy == True:
          time.sleep(0.1) # Wait for serial port to be available
        self.serial_busy = True
        print("Sending " + ser_msg + " message")
        self.serial_port.write(b)
        time.sleep(.01)
        bs = self.serial_port.readline()
        self.serial_busy = False
        #print(bs)
        response = bs.decode()
        print("Send response received: " + response[0:-2])
      except Exception as e:
        print("Failed to send message")
    else:
      print("serial port not defined, returning empty string")
    return response

  #######################
  ### Cleanup processes on node shutdown
  def cleanup_actions(self):
    global serial_port
    print("Shutting down: Executing script cleanup actions")
    if serial_port is not None:
      serial_port.close()


#########################################
# Sealite Discover Method
#########################################

### Function to try and connect to device
def sealite_discover(buad_list,addr_list,active_port_list):
  dev_ports=[]
  dev_buads=[]
  dev_addrs=[]
  dev_count = 0
  print("")
  # Find serial ports
  print("Looking for serail ports on device")
  port_list = []
  ports = serial.tools.list_ports.comports()
  for loc, desc, hwid in sorted(ports):
    print("Found serial_port at: " + loc)
    port_list.append(loc)
  # Checking for devices on available serial ports
  if len(port_list) > 0:
    for port_str in port_list:
      if port_str not in active_port_list:
        for buad_int in buad_list:
          print("#################")
          print("Connecting to serial port " + port_str + " with buadrate: " + str(buad_int))
          try:
            # Try and open serial port
            print("Opening serial port " + port_str + " with buadrate: " + str(buad_int))
            serial_port = serial.Serial(port_str,buad_int,timeout = 0.005)
            for addr in addr_list:
              addr_str = str(addr)
              zero_prefix_len = 3-len(addr_str)
              for z in range(zero_prefix_len):
                addr_str = ('0' + addr_str)
              # Create message string
              ser_msg= ('!' + addr_str + ':INFO?')
              ser_str = (ser_msg + '\r\n')
              # Send Serial String
              #print("")
              #print("Sending serial message: " + ser_msg)
              b=bytearray()
              b.extend(map(ord, ser_str))
              serial_port.write(b)
              #print("Waiting for response")
              time.sleep(.005)
              bs = serial_port.readline()
              response = bs.decode()
              if len(response) > 2:
                print("Got response: " + response)
                if response[3] == ',':
                  addr_str = response[0:3]
                  try:
                    addr_int = int(addr)
                    print("Found device at address: " + addr_str)
                    dev_ports.append(port_str)
                    dev_buads.append(buad_int)
                    dev_addrs.append(addr_str)
                    dev_count = dev_count + 1
                  except Exception as a:
                    print("Returned device message not valid")
                    print(a)
            # Close the port afterwards
            print("Closing serial port " + port_str)
            serial_port.close()
          except Exception as e:
            print("Unable to open serial port " + port_str + " with buadrate: " + str(buad_int))
            print(e)
      else:
        print("Serial port allready active")
  else:
    print("No serial ports found")
  print("Found " + str(dev_count) + " new devices")
  for i in range(dev_count):
    print(dev_ports[i] + "  " + str(dev_buads[i]) + " " + dev_addrs[i])
  return dev_ports,dev_buads,dev_addrs,port_list


#########################################
# Main
#########################################

if __name__ == '__main__':
  active_port_list = []
  active_node_list = []
  while not rospy.is_shutdown():
    # Run Discovery Process at Start
    dev_ports,dev_buads,dev_addrs,port_list = sealite_discover(BAUDRATE_LIST,ADDRESS_LIST,active_port_list)
    # Kill nodes if their port is no longer found
    for i, ap in enumerate(active_port_list):
      if ap not in port_list:
        print("Port " + ap + " no longer available")
        print("Killing node " + active_node_list[i] )
        kill_node = active_node_list[i]
        exec(f'del {kill_node}')
    # Purge active port and node lists
    print("")
    print("Purging active node list")
    keep_ind = []
    for i, node in enumerate(active_node_list):
      print("Looking for active nodes")
      try:
        print("Checking node: " + node)
        exec(f'connected = {node}.connected')
        print("Still active")
        keep_ind.append(i)
      except Exception as n:
        print("Node " + node + " no longer connected")
    if len(keep_ind) != len(active_node_list):
      new_apl = []
      new_anl = []
      for ind in keep_ind:
        new_apl.append(active_port_list[ind])
        new_anl.append(active_node_list[ind])
      active_port_list = new_apl
      active_node_list = new_anl
    else:
      print("No nodes to purge")



    
    # Try and create a LSX Node for each found device
    print("")
    if len(port_list) > 0: 
      for i, port in enumerate(dev_ports):
        if port not in active_port_list:
          port_str = port.split("tty")[1]
          node_name = "lsx_node_" + port_str + "_" + dev_addrs[i]
          exec(f'{node_name}=Sealite_Node(dev_ports[i],dev_buads[i],dev_addrs[i])')
          active_port_list.append(dev_ports[i])
          active_node_list.append(node_name)
    else:
      print("No devices found to connect to")
    print("")
    print("Current active node list")
    print(active_node_list)


       
       
    time.sleep(3)
  rospy.spin()


