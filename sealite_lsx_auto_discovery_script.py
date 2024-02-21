import os
os.environ["ROS_NAMESPACE"] = "/nepi/s2x"
import rospy
import serial
import serial.tools.list_ports
import time
import re
import subprocess

#Define Discovery Search Parameters
BAUDRATE_LIST = [9600,19200,57600] # Three supported buad rates
ADDRESS_LIST = list(range(1,10))  # Total range 1-255

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
  active_subproc_list = []
    
  rospy.init_node('sealite_lsx_detector.py')
  while not rospy.is_shutdown():
    # Run Discovery Process at Start
    dev_ports,dev_buads,dev_addrs,port_list = sealite_discover(BAUDRATE_LIST,ADDRESS_LIST,active_port_list)
    print("Current Port List")
    print(port_list)
    # Remove nodes from active list if no longer found
    updated_apl = []
    updated_anl = []
    for i in range(len(active_node_list)):
      ap = active_port_list[i]
      an = active_node_list[i]
##      print("Checking active port " + ap + " is still a valid serial port")
##      if ap not in port_list:
##        print("Port " + ap + " no longer available")
##        print("Killing node " + active_node_list[i] )
##        kill_proc = active_subproc_list[i]
##        kill_proc.terminate()
##      else:
      check_topic_name = (an + "/lsx/active")
      print("Checking for topic name: " + check_topic_name)
      print("Checking active node " + an + " is still a valid ros node")
      node_exists = False
      topic_list=rospy.get_published_topics(namespace='/')
      for topic_entry in topic_list:
        if topic_entry[0].find(check_topic_name) != -1:
          updated_apl.append(ap)
          updated_anl.append(an)
          node_exists = True
          print("Found topic name")
          break
      if node_exists is False:
        print("Node " + an + " no longer active")
      else:
        print("Node " + an + " still active")
    active_port_list = updated_apl
    active_node_list = updated_anl
    time.sleep(1)
    # Try and create a LSX Node for each found device
    print("")
    if len(port_list) > 0: 
      for i, port in enumerate(dev_ports):
        if port not in active_port_list:
          port_str = port.split("tty")[1]
          lsx_node_name = "sealite_" + port_str + "_" + dev_addrs[i]
          
          # First, load some params on param server for the node we are about to launch
          namespace = rospy.get_namespace()
          print("Discover name: " + namespace)
          rospy.set_param(namespace + lsx_node_name + '/port_str', dev_ports[i])
          rospy.set_param(namespace + lsx_node_name + '/baud_int', dev_buads[i])
          rospy.set_param(namespace + lsx_node_name + '/addr_str', dev_addrs[i])
          time.sleep(2)
          # Pass the name as a regular cmd-line arg since we can't rosrun this new node as it is not currently installed in ROS path
          node_run_cmd = ['python', 'sealite_lsx_driver_script.py', lsx_node_name] 
          p = subprocess.Popen(node_run_cmd)
          active_port_list.append(dev_ports[i])
          active_node_list.append(lsx_node_name)
          active_subproc_list.append(p)
    else:
      print("No devices found to connect to")  
    print("Current active port list")
    print(active_port_list)
    print("Current active node list")
    print(active_node_list)

    time.sleep(3)
