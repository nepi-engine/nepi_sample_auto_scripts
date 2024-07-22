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
import rospy
import time
import threading
import subprocess
import numpy as np
import math
import tf
import random
import sys
import cv2

from nepi_edge_sdk_base import nepi_ros
from nepi_edge_sdk_base import nepi_img
from nepi_edge_sdk_base import nepi_pc
from nepi_edge_sdk_base import nepi_nav



from std_msgs.msg import Empty, Int8, UInt8, UInt32, Bool, String, Float32, Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, NavSatFix, BatteryState
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped
from geographic_msgs.msg import GeoPoint, GeoPose, GeoPoseStamped
from mavros_msgs.msg import State, AttitudeTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL, CommandHome
from pygeodesy.ellipsoidalKarney import LatLon
from cv_bridge import CvBridge

from nepi_ros_interfaces.msg import RBXInfo, RBXStatus, AxisControls, RBXErrorBounds, RBXGotoErrors, RBXMotorControl, \
     RBXGotoPose, RBXGotoPosition, RBXGotoLocation
from nepi_ros_interfaces.srv import NavPoseQuery, NavPoseQueryRequest, RBXCapabilitiesQuery, RBXCapabilitiesQueryResponse, \
     NavPoseCapabilitiesQuery, NavPoseCapabilitiesQueryResponse

from nepi_edge_sdk_base.settings_if import SettingsIF
from nepi_edge_sdk_base.save_data_if import SaveDataIF
from nepi_edge_sdk_base.save_cfg_if import SaveCfgIF


NEPI_BASE_NAMESPACE = nepi_ros.get_base_namespace()

class ROSRBXRobotIF:
    # Default Global Values
    BAD_NAME_CHAR_LIST = [" ","/","'","-","$","#"]
    STATUS_UPDATE_RATE_HZ = 10
    UPDATE_NAVPOSE_RATE_HZ = 10
    CHECK_SAVE_DATA_RATE_HZ = 40
    
    # Factory Control Values 
    FACTORY_GOTO_MAX_ERROR_M = 2.0 # Goal reached when all translation move errors are less than this value
    FACTORY_GOTO_MAX_ERROR_DEG = 2.0 # Goal reached when all rotation move errors are less than this value
    FACTORY_GOTO_STABILIZED_SEC = 1.0 # Window of time that setpoint error values must be good before proceeding
    FACTORY_CMD_TIMEOUT_SEC = 25 # Any action that changes 
    FACTORY_IMAGE_INPUT_TOPIC_NAME = "color_2d_image" # Partial or full ROS namespace string, "" for black image 
    FACTORY_HOME_LOCATION = [47.6540828,-122.3187578,0.0]

    NEPI_BASE_NAMESPACE = nepi_ros.get_base_namespace()


    # Define class variables
    factory_device_name = None
    init_device_name = None
    factory_controls = None
 

    states = []
    modes = []
    actions = []
    data_products = ['image']

    settings_if = None
    save_data_if = None
    save_cfg_if = None
    
    rbx_status_pub_interval = float(1)/float(STATUS_UPDATE_RATE_HZ)
    check_save_data_interval_sec = float(1)/CHECK_SAVE_DATA_RATE_HZ
    update_navpose_interval_sec = float(1)/UPDATE_NAVPOSE_RATE_HZ

    ## Initialize Class Variables
    current_heading_deg = None
    current_orientation_enu_degs = None
    current_orientation_ned_degs = None
    current_position_enu_m = None
    current_position_ned_m = None
    current_location_amsl_geo = None
    current_location_wgs84_geo = None
    current_geoid_height_m = 0
    
    home_location = [0,0,0]

    rbx_state_last = None
    rbx_mode_last = None
    rbx_image_blank = np.zeros((350, 700, 3), dtype = np.uint8) # Empty Black Image
    cv2_img = np.zeros((350, 700, 3), dtype = np.uint8) # Empty Black Image
    rbx_image_source_last = "None"

    def resetFactoryCb(self, msg):
        rospy.loginfo("RBX_IF: Received RBX Driver Factory Reset")
        #rospy.loginfo(msg)
        self.resetFactory()

    def resetFactory(self):
        rospy.set_param('~rbx/device_name', self.factory_device_name)
        rospy.set_param('~rbx/max_error_m', self.FACTORY_GOTO_MAX_ERROR_M)
        rospy.set_param('~rbx/max_error_deg', self.FACTORY_GOTO_MAX_ERROR_DEG)
        rospy.set_param('~rbx/stabilized_sec', self.FACTORY_GOTO_STABILIZED_SEC)
        rospy.set_param('~rbx/cmd_timeout', self.FACTORY_CMD_TIMEOUT_SEC)
        rospy.set_param('~rbx/image_source', self.FACTORY_IMAGE_INPUT_TOPIC_NAME)   
        rospy.set_param('~rbx/image_status_overlay', False)  
        rospy.set_param('~rbx/fake_gps_enabled', False)
        rospy.set_param('~rbx/home_location', self.FACTORY_HOME_LOCATION)


        if self.setMotorControlRatio is not None:
          mc = RBXMotorControl()
          mc.speed_ratio = 0.0
          for i in range(len(self.getMotorControlRatios())):
            mc.motor_ind = i
            self.setMotorControlRatio(mc)
        self.settings_if.resetFactorySettings()
        self.updateFromParamServer()
        self.publishInfo()

    def updateDeviceNameCb(self, msg):
        rospy.loginfo("RBX_IF: Received Device Name update msg")
        #rospy.loginfo(msg)
        new_device_name = msg.data
        self.updateDeviceName(new_device_name)

    def updateDeviceName(self, new_device_name):
        valid_name = True
        for char in self.BAD_NAME_CHAR_LIST:
            if new_device_name.find(char) != -1:
                valid_name = False
        if valid_name is False:
            self.update_error_msg("Received invalid device name update: " + new_device_name)
        else:
            rospy.set_param('~rbx/device_name', new_device_name)
        self.device_save_config_pub.publish(Empty())
        self.publishInfo()


    def resetDeviceNameCb(self,msg):
        rospy.loginfo("RBX_IF: Received Device Name reset msg")
        #rospy.loginfo(msg)
        self.resetDeviceName()

    def resetDeviceName(self):
        rospy.set_param('~rbx/device_name', self.factory_device_name)
        self.device_save_config_pub.publish(Empty())
        self.publishInfo()

    def resetControlsCb(self, msg):
        rospy.loginfo("RBX_IF: Resetting IDX Sensor Controls")
        #rospy.loginfo(msg)
        self.resetControls()

    def resetControls(self):
        rospy.set_param('~rbx/device_name', self.init_device_name)
        rospy.set_param('~rbx/max_error_m', self.init_max_error_m)
        rospy.set_param('~rbx/max_error_deg', self.init_max_error_deg)
        rospy.set_param('~rbx/stabilized_sec', self.init_stabilized_sec)
        rospy.set_param('~rbx/cmd_timeout', self.init_cmd_timeout)
        rospy.set_param('~rbx/image_source', self.init_image_source)
        rospy.set_param('~rbx/image_status_overlay', self.init_image_status_overlay)
        rospy.set_param('~rbx/fake_gps_enabled', self.init_fake_gps_enabled)
        rospy.set_param('~rbx/home_location', self.init_home_location)
        self.updateFromParamServer()
        self.publishInfo()
    

    ##############################
    ### Update image source subscriber
    def imageSubscriberCb(self,img_msg):
        #Convert image from ros to cv2
        bridge = CvBridge()
        cv2_img = bridge.imgmsg_to_cv2(img_msg, "bgr8")
       # Overlay status info on image
        if self.rbx_info.image_status_overlay:
            box_x = 10
            box_y = 10
            box_w = 350
            box_h = 450
            # Add status box overlay
            cv2.rectangle(cv2_img, (box_x, box_y), (box_w, box_h), (255, 255, 255), -1)
            cv2_img = self.cv2_img # Initialize status image
            # Overlay Status Text List
            x=box_x+10 
            y=box_y+20
            status_str_msg = copy.deepcopy(self.status_str_msg)
            for text in status_str_msg:
                self.statusTextOverlay(cv2_img,text,x, y)
                y = y + 20
        # Create ROS Image message
        bridge = CvBridge()
        img_out_msg = bridge.cv2_to_imgmsg(cv2_img,"bgr8")#desired_encoding='passthrough')
        # Publish new image to ros
        if not rospy.is_shutdown():
            self.rbx_image_pub.publish(img_out_msg)
            # You can view the enhanced_2D_image topic at 
            # //192.168.179.103:9091/ in a connected web browser
        self.save_img2file('image',cv2_img,img_msg.header.stamp)

    ##############################
    # RBX Settings Topic Callbacks

    # ToDo: Create a custom RBX status message
    ### Callback to set state
    def setStateCb(self,state_msg):
        rospy.loginfo("RBX_IF: Received set state message")
        rospy.loginfo(state_msg)
        state_val = state_msg.data
        self.setState(state_val)
        

    ### Function to set state
    def setState(self,new_state_ind):
        if new_state_ind < 0 or new_state_ind > (len(self.states)-1):
            self.update_error_msg("No matching rbx state found")
        else:
            self.update_current_errors( [0,0,0,0,0,0,0] )
            self.rbx_status.process_current = self.states[new_state_ind]
            self.rbx_state_last = self.rbx_info.state
            rospy.loginfo("RBX_IF: Waiting for rbx state " + self.states[new_state_ind] + " to set")
            rospy.loginfo("RBX_IF: Current rbx state is " + self.states[self.rbx_info.state])
            self.setStateIndFunction(new_state_ind)
            self.update_prev_errors( [0,0,0,0,0,0,0] )
            self.rbx_status.process_last = self.states[new_state_ind]
            self.rbx_status.process_current = "None"
        self.publishInfo()
        

    ### Callback to set mode
    def setModeCb(self,mode_msg):
        rospy.loginfo("RBX_IF: Received set mode message")
        rospy.loginfo(mode_msg)
        mode_val = mode_msg.data
        self.setMode(mode_val)

    ### Function to set mode
    def setMode(self,new_mode_ind):
        if new_mode_ind < 0 or new_mode_ind > (len(self.modes)-1):
            self.update_error_msg("No matching rbx mode found")
        else:
            self.update_current_errors( [0,0,0,0,0,0,0] )
            self.rbx_status.process_current = self.modes[new_mode_ind]
            rospy.loginfo("RBX_IF: Setting rbx mode to : " + self.modes[new_mode_ind])
            rospy.loginfo("RBX_IF: Calling rbx mode function: " + self.modes[new_mode_ind])
            if self.has_fake_gps:
                self.fake_gps_set_mode_pub.publish(new_mode_ind)
            self.setModeIndFunction(new_mode_ind)
            self.update_prev_errors( [0,0,0,0,0,0,0] )
            self.rbx_status.process_last = self.modes[new_mode_ind]
            self.rbx_status.process_current = "None"
        self.publishInfo()


    ### Callback to start rbx set goto goals process
    def setErrorBoundsCb(self,goto_goals_msg):
        rospy.loginfo("RBX_IF: Received set goals message")
        rospy.loginfo(goto_goals_msg)
        rospy.set_param('~rbx/max_error_m', goto_goals_msg.max_distance_error_m)
        rospy.set_param('~rbx/max_error_deg', goto_goals_msg.max_rotation_error_deg)
        rospy.set_param('~rbx/stabilized_sec', goto_goals_msg.max_stabilize_time_s)
        self.publishInfo()

    ### Callback to set cmd timeout
    def setCmdTimeoutCb(self,cmd_timeout_msg):
        rospy.loginfo("RBX_IF: Received set timeout message")
        rospy.loginfo(cmd_timeout_msg)
        rospy.set_param('~rbx/cmd_timeout', cmd_timeout_msg.data)
        self.publishInfo()


    ### Callback to image topic source
    def setImageTopicCb(self,set_image_topic_msg):
        rospy.loginfo("RBX_IF: Received set image topic message")
        rospy.loginfo(set_image_topic_msg)
        rospy.set_param('~rbx/image_source', set_image_topic_msg.data)
        self.publishInfo()


    ### Callback to add overlay to image topic source
    def enableImageOverlayCb(self,enable_msg):
        rospy.loginfo("RBX_IF: Received enable image overlay message")
        rospy.loginfo(enable_msg)
        rospy.set_param('~rbx/image_status_overlay', enable_msg.data)

    ### Callback to set current process name
    def setProcessNameCb(self,set_process_name_msg):
        rospy.loginfo("RBX_IF: Received set process name message")
        rospy.loginfo(set_process_name_msg)
        self.rbx_status.process_current = (set_process_name_msg.data)
         
    

    ##############################
    # RBX Control Topic Callbacks

   ### Callback to set manual motor control ratio
    def setMotorControlCb(self,motor_msg):
        rospy.loginfo("RBX_IF: Received set motor control ratio message")
        rospy.loginfo(motor_msg)
        new_motor_ctrl = mode_msg.data
        self.setMotorControl(new_motor_ctrl)

    ### Function to set motor control
    def setMotorControl(self,new_motor_ctrl):
        if self.manualControlsReadyFunction() is True:
            m_ind = new_motor_ctrl.motor_ind
            m_sr = new_motor_ctrl.speed_ratio
            m_len = len(self.getMotorControlRatios())
            if m_ind > (m_len -1):
                self.update_error_msg("New Motor Control Ind " + str(m_ind) + " is out of range")
            elif new_motor_ctrl < 0 or new_motor_ctrl > 1:
                self.update_error_msg("New Motor Control Speed Ratio " + str(m_sr) + " is out of range")
            elif self.setMotorControlRatio is not None:
                self.setMotorControlRatio(m_ind,m_sr)
        else:
            self.update_error_msg("Ignoring Set Motor Control msg, Manual Controls not Ready")     


   ### Callback to execute action
    def setActionCb(self,action_msg):
        rospy.loginfo("RBX_IF: *******************************")
        rospy.loginfo("RBX_IF: Received go action message")
        rospy.loginfo(action_msg)
        action_ind = action_msg.data
        if self.setActionIndFunction is not None:
            if action_ind < 0 or action_ind > (len(self.actions)-1):
                self.update_error_msg("No matching rbx action found")
            else:
                if self.rbx_status.ready is False:
                    self.update_error_msg("Another GoTo Command Process is Active")
                    self.update_error_msg("Ignoring this Request")
                else:
                    self.rbx_status.process_current = self.actions[action_ind]
                    self.rbx_status.ready = False
                    self.rbx_cmd_success_current = False
                    self.update_current_errors( [0,0,0,0,0,0,0] )
                    rospy.loginfo("RBX_IF: Starting action: " + self.actions[action_ind])
                    time.sleep(1)
                    if self.has_fake_gps:
                        self.fake_gps_go_action_pub.publish(action_msg)
                    success = self.setActionIndFunction(action_ind)
                    self.rbx_cmd_success_current = success
                    if success:
                      rospy.loginfo("RBX_IF: Finished action: " + self.actions[action_ind])
                    else:
                      rospy.loginfo("RBX_IF: Action: " + self.actions[action_ind] + " Failed to complete")
                    self.rbx_status.process_last = self.actions[action_ind]
                    self.rbx_status.process_current = "None"
                    self.rbx_status.cmd_success = self.rbx_cmd_success_current
                    time.sleep(1)
                    self.rbx_status.ready = True
        else:
            self.update_error_msg("Ignoring Go Action command, no Set Action Function")


    ### Callback to set home
    def setHomeCb(self,geo_msg):
        rospy.loginfo("RBX_IF: *******************************")
        rospy.loginfo("RBX_IF: Received set home message")
        rospy.loginfo(geo_msg)
        new_home_location = [geo_msg.latitude,geo_msg.longitude,geo_msg.altitude]
        if self.setHomeFunction is not None:
            self.setHomeFunction(new_home_location)
        if self.has_fake_gps:
            self.fake_gps_set_home_pub.publish(geo_msg)
        self.home_location = new_home_location


    ### Callback to set home current
    def setHomeCurrentCb(self,empty_msg):
        rospy.loginfo("RBX_IF: *******************************")
        rospy.loginfo("RBX_IF: Received set home current message")
        if self.setHomeFunction is not None:
            self.setHomeFunction(self.current_location_wgs84_geo)
        if self.has_fake_gps:
            self.fake_gps_set_home_current_pub.publish(empty_msg)
        self.home_location = self.current_location_wgs84_geo
          


    ### Callback to start rbx go home
    def goHomeCb(self,home_msg):
        rospy.loginfo("RBX_IF: *******************************")
        rospy.loginfo("RBX_IF: Received go home message")
        time.sleep(1)
        if self.autonomousControlsReadyFunction() is True:
            if self.goHomeFunction is not None:
                self.rbx_status.process_current = "Go Home"
                self.rbx_cmd_success_current = False
                self.rbx_status.ready = False
                self.update_current_errors( [0,0,0,0,0,0,0] )
                self.rbx_cmd_success_current = self.goHomeFunction()
                if self.has_fake_gps:
                  self.fake_gps_go_home_pub.publish(home_msg)
                time.sleep(1)
                self.rbx_status.process_last = "Go Home"
                self.rbx_status.process_current = "None"
                self.rbx_status.cmd_success = self.rbx_cmd_success_current
                time.sleep(1)
                self.rbx_status.ready = True
        else:
            self.update_error_msg("Ignoring Go command, Autononous Controls not Ready")

    ### Callback to start rbx stop
    def goStopCb(self,stop_msg):
        rospy.loginfo("RBX_IF: *******************************")
        rospy.loginfo("RBX_IF: Received go stop message")
        rospy.loginfo(stop_msg)
        time.sleep(1)
        if self.autonomousControlsReadyFunction() is True:
            if self.goStopFunction is not None:
                self.rbx_status.process_current = "Stop"
                self.rbx_cmd_success_current = False
                self.rbx_status.ready = False
                self.update_current_errors( [0,0,0,0,0,0,0] )
                self.rbx_cmd_success_current = self.goStopFunction()
                time.sleep(1)
                self.rbx_status.process_last = "Stop"
                self.rbx_status.process_current = "None"
                self.rbx_status.cmd_success = self.rbx_cmd_success_current
                time.sleep(1)
                self.rbx_status.ready = True
        else:
            self.update_error_msg("Ignoring Go command, Autononous Controls not Ready")

 

    ### Callback to start rbx goto pose process
    def gotoPoseCb(self,pose_cmd_msg):
        rospy.loginfo("RBX_IF: *******************************")
        rospy.loginfo("RBX_IF: Recieved GoTo Pose Message")
        rospy.loginfo("RBX_IF: ")
        rospy.loginfo(pose_cmd_msg)
        time.sleep(1)
        if self.autonomousControlsReadyFunction() is True:
            setpoint_data=[pose_cmd_msg.roll_deg,pose_cmd_msg.pitch_deg,pose_cmd_msg.yaw_deg]
            if self.rbx_status.ready is False:
                self.update_error_msg("Ignoring GoTo POSE Request, Another GoTo Command Process is Active")
            else:
                self.rbx_status.process_current = "GoTo Pose"
                self.rbx_status.ready = False
                self.rbx_cmd_success_current = False
                self.update_current_errors( [0,0,0,0,0,0,0] )
                time.sleep(1)
                self.rbx_cmd_success_current = self.setpoint_attitude_ned(setpoint_data,self.rbx_info.cmd_timeout)
                self.rbx_status.process_last = "GoTo Pose"
                self.rbx_status.process_current = "None"
                self.rbx_status.cmd_success = self.rbx_cmd_success_current
                time.sleep(1)
                self.rbx_status.ready = True
        else:
            self.update_error_msg("Ignoring Go command, Autononous Controls not Ready")



    ### Callback to start rbx goto position process
    def gotoPositionCb(self,position_cmd_msg):
        rospy.loginfo("RBX_IF: *******************************")
        rospy.loginfo("RBX_IF: Recieved GoTo Position Command Message")
        rospy.loginfo("RBX_IF: ")
        rospy.loginfo(position_cmd_msg)
        time.sleep(1)
        if self.rbx_status.manual_motor_control_mode_ready is False:
            setpoint_data=[position_cmd_msg.x_meters,position_cmd_msg.y_meters,position_cmd_msg.z_meters,position_cmd_msg.yaw_deg]
            if self.rbx_status.ready is False:
                self.update_error_msg("Ignoring GoTo Position Request, Another GoTo Command Process is Active")
            else:
                self.rbx_status.process_current = "GoTo Position"
                self.rbx_status.ready = False
                self.rbx_cmd_success_current = False
                self.update_current_errors( [0,0,0,0,0,0,0] )
                time.sleep(1)
                self.rbx_status.ready = False
                if self.has_fake_gps:
                  self.fake_gps_goto_position_pub.publish(position_cmd_msg)
                self.rbx_cmd_success_current = self.setpoint_position_local_body(setpoint_data,self.rbx_info.cmd_timeout)
                self.rbx_status.process_last = "GoTo Position"
                self.rbx_status.process_current = "None"
                self.rbx_status.cmd_success = self.rbx_cmd_success_current
                time.sleep(1)
                self.rbx_status.ready = True
        else:
            self.update_error_msg("Ignoring Go command, Autononous Controls not Ready")

    ### Callback to start rbx goto location subscriber
    def gotoLocationCb(self,location_cmd_msg):
        rospy.loginfo("RBX_IF: *******************************")
        rospy.loginfo("RBX_IF: Recieved GoTo Location Message")
        rospy.loginfo("RBX_IF: ")
        rospy.loginfo(location_cmd_msg)
        if self.autonomousControlsReadyFunction() is True:
            setpoint_data=[location_cmd_msg.lat,location_cmd_msg.long,location_cmd_msg.altitude_meters,location_cmd_msg.yaw_deg]
            if self.rbx_status.ready is False:
                self.update_error_msg("Ignoring GoTo Location Request, Another GoTo Command Process is Active")
            else:
                self.rbx_status.process_current = "GoTo Location"
                self.rbx_status.ready = False
                self.rbx_cmd_success_current = False
                self.update_current_errors( [0,0,0,0,0,0,0] )
                if self.has_fake_gps:
                  self.fake_gps_goto_location_pub.publish(location_cmd_msg)
                self.rbx_cmd_success_current = self.setpoint_location_global_wgs84(setpoint_data,self.rbx_info.cmd_timeout)
                self.rbx_status.process_last = "GoTo Location"
                self.rbx_status.process_current = "None"
                self.rbx_status.cmd_success = self.rbx_cmd_success_current
                time.sleep(1)
                self.rbx_status.ready = True
        else:
            self.update_error_msg("Ignoring Go command, Autononous Controls not Ready")

    ### Callback to enble fake gps
    def fakeGPSEnableCb(self,msg):
        rospy.loginfo("RBX_IF: Received set set fake gps enable message")
        rospy.loginfo(msg)
        rospy.set_param("~fake_gps_eabled",msg.data)
        self.fake_gps_enable_pub.publish(msg)

    ### Callback to enble fake gps
    def fakeGPSResetCb(self,msg):
        rospy.loginfo("RBX_IF: Received set set fake gps reset message")
        self.fake_gps_reset_pub.publish(Empty())

    ### Setup a regular background navpose get and update navpose data
    def updateNavPoseCb(self,timer):
        # Get current NEPI NavPose data from NEPI ROS nav_pose_query service call
        try:
            nav_pose_response = self.get_navpose_service(NavPoseQueryRequest())
            #rospy.loginfo(nav_pose_response)
            # Get current navpose
            current_navpose = nav_pose_response.nav_pose
            # Get current heading in degrees
            self.current_heading_deg = nepi_nav.get_navpose_heading_deg(nav_pose_response)
            # Get current orientation vector (roll, pitch, yaw) in degrees enu frame
            self.current_orientation_enu_degs = nepi_nav.get_navpose_orientation_enu_degs(nav_pose_response)
            # Get current orientation vector (roll, pitch, yaw) in degrees ned frame +-180
            self.current_orientation_ned_degs = nepi_nav.get_navpose_orientation_ned_degs(nav_pose_response)
            # Get current position vector (x, y, z) in meters enu frame
            self.current_position_enu_m = nepi_nav.get_navpose_position_enu_m(nav_pose_response)
            # Get current position vector (x, y, z) in meters ned frame
            self.current_position_ned_m = nepi_nav.get_navpose_position_ned_m(nav_pose_response)
            # Get current geoid hieght
            self.current_geoid_height_m =  nepi_nav.get_navpose_geoid_height(nav_pose_response)
            # Get current location vector (lat, long, alt) in geopoint data with WGS84 height
            self.current_location_wgs84_geo =  nepi_nav.get_navpose_location_wgs84_geo(nav_pose_response) 
            # Get current location vector (lat, long, alt) in geopoint data with AMSL height
            self.current_location_amsl_geo =  nepi_nav.get_navpose_location_amsl_geo(nav_pose_response)
    ##      rospy.loginfo("RBX_IF: ")
    ##      rospy.loginfo(self.current_geoid_height_m)
    ##      rospy.loginfo(self.current_location_wgs84_geo)
    ##      rospy.loginfo(self.current_location_amsl_geo)
        except Exception as e:
            self.update_error_msg("navpose service call failed: " + str(e))


    def updateFromParamServer(self):
        if self.has_fake_gps:
          home_location = rospy.get_param('~rbx/home_location', self.init_home_location)
          new_home = GeoPoint()
          new_home.latitude = home_location[0]
          new_home.longitude = home_location[1]
          new_home.altitude = home_location[2]
          self.fake_gps_set_home_pub.publish(new_home)
          fake_gps_enabled = rospy.get_param('~rbx/fake_gps_enabled', self.init_fake_gps_enabled)
          self.fake_gps_enable_pub.publish(fake_gps_enabled) 
          self.fake_gps_reset_pub.publish(Empty()) 
        self.settings_if.updateFromParamServer()
   
    def capabilities_query_callback(self, _):
        return self.capabilities_report
    
    def navpose_capabilities_query_callback(self, _):
        return self.navpose_capabilities_report  

    def setCurrentAsDefault(self):
        pass # We only use the param server, no member variables to apply to param server
           
    def __init__(self, device_info, capSettings, 
                 factorySettings, settingUpdateFunction, getSettingsFunction,
                 axisControls,getBatteryPercentFunction,
                 states,getStateIndFunction,setStateIndFunction,
                 modes,getModeIndFunction,setModeIndFunction,
                 actions, setActionIndFunction,
                 getHomeFunction=None,setHomeFunction=None,
                 manualControlsReadyFunction=None,
                 getMotorControlRatios=None,
                 setMotorControlRatio=None,
                 autonomousControlsReadyFunction=None,
                 goHomeFunction=None, goStopFunction=None, 
                 gotoPoseFunction=None, gotoPositionFunction=None, gotoLocationFunction=None,
                 gpsTopic=None,odomTopic=None,headingTopic=None,
                 fake_gps_namespace = None
                 ):
        
        
        self.node_name = device_info["node_name"]
        self.device_name = device_info["device_name"]
        self.identifier = device_info["identifier"]
        self.serial_num = device_info["serial_number"]
        self.hw_version = device_info["hw_version"]
        self.sw_version = device_info["sw_version"]

        self.states = states
        self.getStateIndFunction = getStateIndFunction
        self.setStateIndFunction = setStateIndFunction

        self.modes = modes
        self.getModeIndFunction = getModeIndFunction
        self.setModeIndFunction = setModeIndFunction
        

        # Create the CV bridge. Do this early so it can be used in the threading run() methods below 
        # TODO: Need one per image output type for thread safety?
        self.cv_bridge = CvBridge()


        # Create and start initializing Capabilities values
        self.nav_pose_capabilities_report = NavPoseCapabilitiesQueryResponse()
        self.nav_pose_capabilities_report.has_gps = gpsTopic is not None
        self.nav_pose_capabilities_report.has_orientation = odomTopic is not None
        self.nav_pose_capabilities_report.has_heading = headingTopic is not None

        self.capabilities_report = RBXCapabilitiesQueryResponse()
        if axisControls == None:
          axis_controls = AxisControls()
          axis_controls.x = False
          axis_controls.y = False
          axis_controls.z = False
          axis_controls.roll = False
          axis_controls.pitch = False
          axis_controls.yaw = False
        self.capabilities_report.control_support = axisControls
        
        self.capabilities_report.state_options = str(states)
        self.capabilities_report.mode_options = str(modes)
        self.capabilities_report.action_options = str(actions)
        self.capabilities_report.data_products = str(self.data_products)
        
        # Initialize Home location value
        self.init_home_location = rospy.get_param('~rbx/home_location', self.FACTORY_HOME_LOCATION)
        rospy.set_param('~rbx/home_location', self.init_home_location)

        # Setup fake gps interfaces if available
        self.init_fake_gps_enabled = rospy.get_param('~rbx/fake_gps_enabled', False)
        rospy.set_param('~rbx/fake_gps_enabled', self.init_fake_gps_enabled)
        if fake_gps_namespace is not None:
          self.has_fake_gps = True
          self.capabilities_report.has_fake_gps = self.has_fake_gps
          rospy.Subscriber("~rbx/set_fake_gps_enable", Bool, self.fakeGPSEnableCb)
          self.fake_gps_enable_pub = rospy.Publisher(fake_gps_namespace + "set_enable", Bool, queue_size=1)
          rospy.Subscriber("~rbx/reset_fake_gps", Empty, self.fakeGPSResetCb)
          self.fake_gps_reset_pub = rospy.Publisher(fake_gps_namespace + "reset",Empty, queue_size=1)
          rospy.loginfo("Setting up fake gps publishers on namespace: " + fake_gps_namespace)
          self.fake_gps_goto_position_pub = rospy.Publisher(fake_gps_namespace + "goto_position", RBXGotoPosition, queue_size=1)
          self.fake_gps_goto_location_pub = rospy.Publisher(fake_gps_namespace + "goto_location", RBXGotoLocation, queue_size=1)
          self.fake_gps_set_home_pub = rospy.Publisher(fake_gps_namespace + "set_home",GeoPoint , queue_size=1)
          self.fake_gps_set_home_current_pub = rospy.Publisher(fake_gps_namespace + "set_home_current", Empty, queue_size=1)
          self.fake_gps_go_home_pub = rospy.Publisher(fake_gps_namespace + "go_home", Empty, queue_size=1)
          self.fake_gps_set_mode_pub = rospy.Publisher(fake_gps_namespace + "set_mode" , UInt8, queue_size=1)
          self.fake_gps_go_action_pub = rospy.Publisher(fake_gps_namespace + "go_action", UInt8, queue_size=1)
          



        self.getBatteryPercentFunction = getBatteryPercentFunction
        if self.getBatteryPercentFunction is not None:
          self.capabilities_report.has_battery_feedback = True
        else:
          self.capabilities_report.has_battery_feedback = False
       
        # Create and start initializing Status values
        self.factory_device_name = device_info["sensor_name"] + "_" + device_info["identifier"]
        self.init_device_name = rospy.get_param('~rbx/device_name', self.factory_device_name)
        rospy.set_param('~rbx/device_name', self.init_device_name)
        rospy.Subscriber('~rbx/update_device_name', String, self.updateDeviceNameCb, queue_size=1) # start local callbac
        rospy.Subscriber('~rbx/reset_device_name', Empty, self.resetDeviceNameCb, queue_size=1) # start local callback


        self.rbx_info=RBXInfo()
        self.rbx_info.connected = False
        self.rbx_info.serial_num = self.serial_num
        self.rbx_info.hw_version = self.hw_version
        self.rbx_info.sw_version = self.sw_version
        self.rbx_info.standby = False
        self.rbx_info.state = -999
        self.rbx_info.mode = -999
        self.rbx_info.home_lat = self.home_location[0]
        self.rbx_info.home_long = self.home_location[1]
        self.rbx_info.home_alt = self.home_location[2]
        error_bounds = RBXErrorBounds()
        error_bounds.max_distance_error_m = rospy.get_param('~rbx/max_error_m', self.FACTORY_GOTO_MAX_ERROR_M)
        error_bounds.max_rotation_error_deg = rospy.get_param('~rbx/max_error_deg', self.FACTORY_GOTO_MAX_ERROR_DEG)
        error_bounds.max_stabilize_time_s = rospy.get_param('~rbx/stabilized_sec', self.FACTORY_GOTO_STABILIZED_SEC)
        self.rbx_info.error_bounds = error_bounds
        self.rbx_info.cmd_timeout = rospy.get_param('~rbx/cmd_timeout', self.FACTORY_CMD_TIMEOUT_SEC)
        self.rbx_info.image_source = rospy.get_param('~rbx/image_source', self.FACTORY_IMAGE_INPUT_TOPIC_NAME)
        self.rbx_info.image_status_overlay = rospy.get_param('~rbx/image_status_overlay', False) 
        self.rbx_info.state = self.states[self.getStateIndFunction()]
        self.rbx_info.mode = self.modes[self.getModeIndFunction()]
        ## Update Control Info
        self.getMotorControlRatios = getMotorControlRatios
        if self.getMotorControlRatios is not None:
            motor_controls_info_msg = self.get_motor_controls_status_msg(self.getMotorControlRatios())
        else:
            motor_controls_info_msg = self.get_motor_controls_status_msg([])

        self.rbx_status=RBXStatus()
        self.rbx_status.process_current = "None"
        self.rbx_status.process_last = "None"
        self.rbx_status.ready = False
        self.rbx_status.battery = 0

        # Setup Error Tracking
        self.init_max_error_m = rospy.get_param('~rbx/max_error_m', self.FACTORY_GOTO_MAX_ERROR_M)
        rospy.set_param('~rbx/max_error_m', self.init_max_error_m)
        self.init_max_error_deg = rospy.get_param('~rbx/max_error_deg', self.FACTORY_GOTO_MAX_ERROR_DEG)
        rospy.set_param('~rbx/max_error_deg', self.init_max_error_deg)
        self.init_stabilized_sec = rospy.get_param('~rbx/stabilized_sec', self.FACTORY_GOTO_STABILIZED_SEC)
        rospy.set_param('~rbx/stabilized_sec', self.init_stabilized_sec)
        rospy.Subscriber("~rbx/set_goto_error_bounds", RBXErrorBounds, self.setErrorBoundsCb)
        self.rbx_status.errors_current = RBXGotoErrors()
        self.rbx_status.errors_current.x_m = 0
        self.rbx_status.errors_current.y_m = 0
        self.rbx_status.errors_current.z_m = 0
        self.rbx_status.errors_current.heading_deg = 0
        self.rbx_status.errors_current.roll_deg = 0
        self.rbx_status.errors_current.pitch_deg = 0
        self.rbx_status.errors_current.yaw_deg = 0

        self.rbx_status.errors_prev = RBXGotoErrors()
        self.rbx_status.errors_prev.x_m = 0
        self.rbx_status.errors_prev.y_m = 0
        self.rbx_status.errors_prev.z_m = 0
        self.rbx_status.errors_prev.heading_deg = 0
        self.rbx_status.errors_prev.roll_deg = 0
        self.rbx_status.errors_prev.pitch_deg = 0
        self.rbx_status.errors_prev.yaw_deg = 0

        self.rbx_status.last_error_message = "" 


        ### Start RBX Config Subscribe Topics
        rospy.Subscriber("~rbx/set_state", UInt8, self.setStateCb)
        rospy.Subscriber("~rbx/set_mode", UInt8, self.setModeCb)

        # Set Up Manual Motor Controls
        self.manualControlsReadyFunction = manualControlsReadyFunction
        if self.manualControlsReadyFunction is not None:
          self.rbx_status.manual_motor_control_mode_ready = self.manualControlsReadyFunction()
          manual_controls_ready = self.manualControlsReadyFunction()
          if manual_controls_ready:
            if self.setMotorControlRatio is not None:
              mc = RBXMotorControl()
              mc.speed_ratio = 0.0
              for i in range(len(self.getMotorControlRatios())):
                mc.motor_ind = i
                self.setMotorControlRatio(mc)
        else:
          self.rbx_status.manual_motor_control_mode_ready = False

        if self.getMotorControlRatios is not None:
          motor_controls_status_msg = self.get_motor_controls_status_msg(self.getMotorControlRatios())
        else:
          motor_controls_status_msg = self.get_motor_controls_status_msg([])
        self.rbx_status.current_motor_control_settings = motor_controls_status_msg

        self.setMotorControlRatio = setMotorControlRatio
        if self.setMotorControlRatio is None:
            self.capabilities_report.has_motor_controls = False
        else:
            self.capabilities_report.has_motor_controls = True
            rospy.Subscriber("~rbx/set_motor_control", RBXMotorControl, self.setMotorControlCb)


        # Set Up Autonomous Contros

        self.autonomousControlsReadyFunction = autonomousControlsReadyFunction
        if self.autonomousControlsReadyFunction is not None:
          self.rbx_status.autonomous_control_mode_ready = self.autonomousControlsReadyFunction()
          self.capabilities_report.has_autonomous_controls = True
        else:
          self.rbx_status.autonomous_control_mode_ready = False
          self.capabilities_report.has_autonomous_controls = False

        self.init_cmd_timeout = rospy.get_param('~rbx/cmd_timeout', self.FACTORY_CMD_TIMEOUT_SEC)
        rospy.set_param('~rbx/cmd_timeout', self.init_cmd_timeout)
        rospy.Subscriber("~rbx/set_goto_timeout", UInt32, self.setCmdTimeoutCb)

        self.actions = actions
        self.setActionIndFunction = setActionIndFunction
        rospy.Subscriber("~rbx/go_action", UInt8, self.setActionCb) 
  
        self.getHomeFunction = getHomeFunction
        self.setHomeFunction  = setHomeFunction
        if self.setHomeFunction is None and self.has_fake_gps is None:
            self.capabilities_report.has_set_home = False
        else:
            self.capabilities_report.has_set_home = True
            rospy.Subscriber("~rbx/set_home", GeoPoint, self.setHomeCb)       
            rospy.Subscriber("~rbx/set_home_current", Empty, self.setHomeCurrentCb)          

        self.goHomeFunction = goHomeFunction
        if self.goHomeFunction is None:
            self.capabilities_report.has_go_home = False
        else:
            self.capabilities_report.has_go_home = True
            rospy.Subscriber("~rbx/go_home", Empty, self.goHomeCb)

        self.goStopFunction = goStopFunction
        if self.goStopFunction is None:
            self.capabilities_report.has_go_stop = False
        else:
            self.capabilities_report.has_go_stop = True
            rospy.Subscriber("~rbx/go_stop", Empty, self.goStopCb)

        self.gotoPoseFunction = gotoPoseFunction
        if self.gotoPoseFunction is None:
            self.capabilities_report.has_goto_pose = False
        else:
            self.capabilities_report.has_goto_pose = True            
            rospy.Subscriber("~rbx/goto_pose", RBXGotoPose, self.gotoPoseCb)

        self.gotoPositionFunction = gotoPositionFunction
        if self.gotoPositionFunction is None:
            self.capabilities_report.has_goto_position = False
        else:
            self.capabilities_report.has_goto_position = True            
            rospy.Subscriber("~rbx/goto_position", RBXGotoPosition, self.gotoPositionCb)

        self.gotoLocationFunction = gotoLocationFunction
        if self.gotoLocationFunction is None:
            self.capabilities_report.has_goto_location = False
        else:
            self.capabilities_report.has_goto_location = True
            rospy.Subscriber("~rbx/goto_location", RBXGotoLocation, self.gotoLocationCb)

        self.rbx_status.cmd_success = False

        ## Start NavPose Processes
        # Define NavPose Namespaces
        NEPI_SET_NAVPOSE_GPS_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose_mgr/set_gps_fix_topic"
        NEPI_SET_NAVPOSE_HEADING_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose_mgr/set_heading_topic"
        NEPI_SET_NAVPOSE_ORIENTATION_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose_mgr/set_orientation_topic"
        NEPI_ENABLE_NAVPOSE_GPS_CLOCK_SYNC_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose_mgr/enable_gps_clock_sync"
        # Define NavPose Services Calls
        NAVPOSE_SERVICE_NAME = NEPI_BASE_NAMESPACE + "nav_pose_query"
        self.get_navpose_service = rospy.ServiceProxy(NAVPOSE_SERVICE_NAME, NavPoseQuery)
        set_gps_pub = rospy.Publisher(NEPI_SET_NAVPOSE_GPS_TOPIC, String, queue_size=1)
        set_orientation_pub = rospy.Publisher(NEPI_SET_NAVPOSE_ORIENTATION_TOPIC, String, queue_size=1)
        set_heading_pub = rospy.Publisher(NEPI_SET_NAVPOSE_HEADING_TOPIC, String, queue_size=1)
        set_gps_timesync_pub = rospy.Publisher(NEPI_ENABLE_NAVPOSE_GPS_CLOCK_SYNC_TOPIC, Bool, queue_size=1)
        time.sleep(1)
        # Start NavPose Data Updater
        rospy.Timer(rospy.Duration(self.update_navpose_interval_sec), self.updateNavPoseCb)
        # Connect to NEPI NavPose Solution
        # Set GPS Topic
        if gpsTopic is not None:
            set_gps_pub.publish(gpsTopic)
            rospy.loginfo("RBX_IF: GPS Topic Set to: " + gpsTopic)
            # Sync NEPI clock to GPS timestamp
            set_gps_timesync_pub.publish(data=True)
            rospy.loginfo("RBX_IF: Setup complete")
        # Set Orientation Topic
        if odomTopic is not None:
            set_orientation_pub.publish(odomTopic)
            rospy.loginfo("RBX_IF: Orientation Topic Set to: " + odomTopic)
        # Set Heading Topic
        if headingTopic is not None:
            set_heading_pub.publish(headingTopic)
            rospy.loginfo("RBX_IF: Heading Topic Set to: " + headingTopic)

 
        # Setup interface classes and update
        self.settings_if = SettingsIF(capSettings, factorySettings, settingUpdateFunction, getSettingsFunction)
        self.save_data_if = SaveDataIF(data_product_names = self.data_products)
        self.save_cfg_if = SaveCfgIF(updateParamsCallback=self.setCurrentAsDefault, paramsModifiedCallback=self.updateFromParamServer)

        # Start capabilities services
        rospy.Service("~rbx/capabilities_query", RBXCapabilitiesQuery, self.capabilities_query_callback)
        rospy.Service("~rbx/navpose_capabilities_query", NavPoseCapabilitiesQuery, self.navpose_capabilities_query_callback)

        # Start Status Publisher
        self.init_image_source = rospy.get_param('~rbx/image_source', self.FACTORY_IMAGE_INPUT_TOPIC_NAME)
        rospy.set_param('~rbx/image_source', self.init_image_source)   
        rospy.Subscriber("~rbx/set_image_topic", String, self.setImageTopicCb)

        self.init_image_status_overlay = rospy.get_param('~rbx/image_status_overlay', False)
        rospy.set_param('~rbx/image_status_overlay', self.init_image_status_overlay)       
        rospy.Subscriber("~rbx/enable_image_overlay", Bool, self.enableImageOverlayCb)


        self.rbx_info_pub = rospy.Publisher("~rbx/info", RBXInfo, queue_size=1, latch = True)
        rospy.Subscriber("~rbx/publish_info", Empty, self.publishInfoCb)
        self.rbx_status_pub = rospy.Publisher("~rbx/status", RBXStatus, queue_size=1, latch = True)
        rospy.Subscriber("~rbx/publish_status", Empty, self.publishStatusCb)
        self.rbx_status_str_pub = rospy.Publisher("~rbx/status_str", String, queue_size=1, latch = True)
        self.rbx_image_pub = rospy.Publisher("~rbx/image", Image, queue_size=1)

        rospy.Timer(rospy.Duration(self.rbx_status_pub_interval), self.statusPublishCb)

        # Start additional subscribers
        rospy.Subscriber('~rbx/reset_factory', Empty, self.resetFactoryCb, queue_size=1) # start local callback
        rospy.Subscriber("~rbx/set_process_name" , String, self.setProcessNameCb)

        # Start additional publishers

        ## Initiation Complete
        rospy.loginfo("RBX_IF: RBX IF Initialization Complete")
        self.publishInfo()
        self.publishStatus()
        self.rbx_info.connected = True
        self.rbx_status.ready = True


  # RBX Status Topic Publishers

    def publishInfoCb(self, msg):
        self.publishInfo()
    
    
    def publishInfo(self):
        self.rbx_info.device_name = rospy.get_param('~rbx/device_name', self.init_device_name)
        error_bounds = RBXErrorBounds()
        error_bounds.max_distance_error_m = rospy.get_param('~rbx/max_error_m', self.init_max_error_m)
        error_bounds.max_rotation_error_deg = rospy.get_param('~rbx/max_error_deg', self.init_max_error_deg)
        error_bounds.max_stabilize_time_s = rospy.get_param('~rbx/stabilized_sec', self.init_stabilized_sec)
        self.rbx_info.error_bounds = error_bounds
        self.rbx_info.cmd_timeout = rospy.get_param('~rbx/cmd_timeout', self.init_cmd_timeout)
        self.rbx_info.image_source = rospy.get_param('~rbx/image_source', self.init_image_source)
        self.rbx_info.image_status_overlay = rospy.get_param('~rbx/image_status_overlay', self.init_image_status_overlay) 
        self.rbx_info.state = self.getStateIndFunction()
        self.rbx_info.mode = self.getModeIndFunction()
        if self.getHomeFunction is not None:
          home_location = self.getHomeFunction()
        else: 
          home_location = self.home_location        
        self.rbx_info.home_lat = home_location[0]
        self.rbx_info.home_long = home_location[1]
        self.rbx_info.home_alt = home_location[2]

       ## Update image source topic and subscriber if changed from last time.
        if self.rbx_info.image_source != self.rbx_image_source_last:
        #  If currently set, first unregister current image topic
            if self.rbx_image_source_last != "None": 
              try:
                self.rbx_image_sub.unregister()
                rospy.loginfo("RBX_IF: Unsubscribed from image source: " + self.rbx_info.image_source)
                time.sleep(1)
              except Exception as e:
                rospy.loginfo(e)
            # Try to find and subscribe to new image source topic
            #rospy.loginfo("RBX_IF: Looking for topic: " + self.rbx_info.image_source)
            image_topic = nepi_ros.find_topic(self.rbx_info.image_source)
            rospy.loginfo("RBX_IF: Image source search returned: " + image_topic)
            if image_topic != "None":  # If image topic exists subscribe
                self.rbx_image_sub = rospy.Subscriber(image_topic, Image, self.imageSubscriberCb, queue_size = 1)
                self.rbx_image_source_last = self.rbx_info.image_source
                self.rbx_info.image_source = image_topic
                rospy.loginfo("RBX_IF: Subscribed to new image source: " + image_topic)
            else:
                self.update_error_msg("Unable to find image topic " + self.rbx_info.image_source)
                self.rbx_info.image_source = "None"
                self.cv2_img = self.rbx_image_blank # Set to blank image if source topic is cleared.
            rospy.set_param('~rbx/image_source', self.rbx_info.image_source)
        if not rospy.is_shutdown():
            #rospy.loginfo(self.rbx_info)
            self.rbx_info_pub.publish(self.rbx_info)

    ### Callback for rbx status publisher
    def statusPublishCb(self,timer):
        self.publishStatus()

    def publishStatusCb(self, msg):
        self.publishStatus()

    def publishStatus(self):
        # Update NavPose Info
        if self.getBatteryPercentFunction is not None:
          self.rbx_battery = self.getBatteryPercentFunction()
        else:
          self.rbx_battery = -999
        self.rbx_status.current_lat = self.current_location_wgs84_geo[0]
        self.rbx_status.current_long  = self.current_location_wgs84_geo[1]
        self.rbx_status.current_altitude  = self.current_location_wgs84_geo[2]
        self.rbx_status.current_heading = self.current_heading_deg
        self.rbx_status.current_roll = self.current_orientation_ned_degs[0]
        self.rbx_status.current_pitch  = self.current_orientation_ned_degs[1]
        self.rbx_status.current_yaw = self.current_orientation_ned_degs[2]


        ## Update Control Info
        if self.manualControlsReadyFunction is not None:
          self.rbx_status.manual_motor_control_mode_ready = self.manualControlsReadyFunction()
        else:
          self.rbx_status.manual_motor_control_mode_ready = False

        if self.getMotorControlRatios is not None:
            motor_controls_msg = self.get_motor_controls_status_msg(self.getMotorControlRatios())
        else:
            motor_controls_msg = self.get_motor_controls_status_msg([])
        self.rbx_status.current_motor_control_settings = motor_controls_msg


        if self.autonomousControlsReadyFunction is not None:
          self.rbx_status.autonomous_control_mode_ready = self.autonomousControlsReadyFunction()
        else:
          self.rbx_status.autonomous_control_mode_ready = False


         # Create Status Info Text List
        status_str_msg = ["RBX Status"]
        if self.rbx_status.battery < 0.1:
            battery_string = "No Reading"
        else:
            battery_string = '%.2f' % self.rbx_status.battery
        status_str_msg.append("State: " + self.states[self.getStateIndFunction()])
        status_str_msg.append("Mode Current: " + self.modes[self.getModeIndFunction()])
        if self.rbx_mode_last is not None:
          status_str_msg.append("Mode Last: " + self.modes[self.rbx_mode_last])
        else:
          status_str_msg.append("Mode Last: None")
        status_str_msg.append("Ready: " + str(self.rbx_status.ready))
        status_str_msg.append("")
        status_str_msg.append("Current Process: " + self.rbx_status.process_current)
        status_str_msg.append(" X,Y,Z Errors Meters: ")
        status_str_msg.append(" " + '%.2f' % self.rbx_status.errors_current.x_m + "," + '%.2f' % self.rbx_status.errors_current.y_m + "," + '%.2f' % self.rbx_status.errors_current.z_m)
        status_str_msg.append(" R,P,Y Errors Degrees: ")
        status_str_msg.append(" " + '%.2f' % self.rbx_status.errors_current.roll_deg + "," + '%.2f' % self.rbx_status.errors_current.pitch_deg + "," + '%.2f' % self.rbx_status.errors_current.yaw_deg)
        status_str_msg.append("")
        status_str_msg.append("Last Process: " + self.rbx_status.process_last)
        status_str_msg.append(" Success: " + str(self.rbx_status.cmd_success))
        status_str_msg.append(" X,Y,Z Errors Meters: ")
        status_str_msg.append(" " + '%.2f' % self.rbx_status.errors_prev.x_m + "," + '%.2f' % self.rbx_status.errors_prev.y_m + "," + '%.2f' % self.rbx_status.errors_prev.z_m)
        status_str_msg.append(" R,P,Y Errors Degrees: ")
        status_str_msg.append(" " + '%.2f' % self.rbx_status.errors_prev.roll_deg + "," + '%.2f' % self.rbx_status.errors_prev.pitch_deg + "," + '%.2f' % self.rbx_status.errors_prev.yaw_deg)
        status_str_msg.append("")
        self.status_str_msg = status_str_msg
        if not rospy.is_shutdown():
            self.rbx_status_pub.publish(self.rbx_status)
            self.rbx_status_str_pub.publish(status_str_msg)
 
 
        
    ## Status Text Overlay Function
    def statusTextOverlay(self,cv_image,status_text,x,y):
        font                   = cv2.FONT_HERSHEY_SIMPLEX
        bottomLeftCornerOfText = (x,y)
        fontScale              = 0.5
        fontColor              = (0, 0, 0)
        thickness              = 1
        lineType               = 1
        cv2.putText(cv_image,status_text, 
            bottomLeftCornerOfText, 
            font, 
            fontScale,
            fontColor,
            thickness,
            lineType)

    #######################
    # Define saving functions for saving callbacks
    def save_img2file(self,data_product,cv2_img,ros_timestamp):
        if self.save_data_if is not None:
            saving_is_enabled = self.save_data_if.data_product_saving_enabled(data_product)
            snapshot_enabled = self.save_data_if.data_product_snapshot_enabled(data_product)
            # Save data if enabled
            if saving_is_enabled or snapshot_enabled:
                if cv2_img is not None:
                    device_name = rospy.get_param('~pc_app/device_name', self.init_device_name)
                    if (self.save_data_if.data_product_should_save(data_product) or snapshot_enabled):
                        full_path_filename = self.save_data_if.get_full_path_filename(nepi_ros.get_datetime_str_from_stamp(ros_timestamp), 
                                                                                                device_name + "-" + data_product, 'png')
                        if os.path.isfile(full_path_filename) is False:
                            cv2.imwrite(full_path_filename, cv2_img)
                            self.save_data_if.data_product_snapshot_reset(data_product)


    def saveImgThread(self,timer):
        data_product = 'image'
        eval("self.save_img2file(data_product,self." + data_product + ",self." + data_product + "_timestamp)")

    #######################
    # RBX IF Methods

    ### Function to set and check setpoint attitude NED command
    ###################################################
    # Input is [ROLL_NED_DEG, PITCH_NED_DEG, YEW_NED_DEGREES]
    # Converted to ENU befor sending message
    ###################################################
    def setpoint_attitude_ned(self,setpoint_attitude,timeout_sec=10):
      # setpoint_attitude is [ROLL_NED_DEG, PITCH_NED_DEG, YEW_NED_DEGREES]
      # Use value -999 to use current value
      cmd_success = True
      self.update_current_errors( [0,0,0,0,0,0,0] )
      rospy.loginfo("RBX_IF: Starting Setpoint Attitude Create-Send-Check Process")
      ##############################################
      # Capture Current NavPose Data
      ##############################################
      start_orientation_ned_degs=list(self.current_orientation_ned_degs)
      rospy.loginfo('')
      rospy.loginfo("RBX_IF: Attitude Current NED Degrees")
      rospy.loginfo("RBX_IF:  Roll, Pitch, Yaw")
      rospy.loginfo(["%.2f" % start_orientation_ned_degs[0],"%.2f" % start_orientation_ned_degs[1],"%.2f" % start_orientation_ned_degs[2]])
      ##############################################
      # Condition Inputs
      ##############################################
      input_attitude_ned_degs = list(setpoint_attitude)
      rospy.loginfo('')
      rospy.loginfo("RBX_IF: Attitude Input NED Degrees")
      rospy.loginfo("RBX_IF:  Roll, Pitch, Yaw")
      rospy.loginfo(["%.2f" % input_attitude_ned_degs[0],"%.2f" % input_attitude_ned_degs[1],"%.2f" % input_attitude_ned_degs[2]])
      # Set new attitude in degs NED
      new_attitude_ned_degs=list(start_orientation_ned_degs) # Initialize with start values
      for ind in range(3): # Overwrite current with new if set and valid
        if setpoint_attitude[ind] != -999:
          new_attitude_ned_degs[ind]=setpoint_attitude[ind]
        # Condition to +-180 deg
        if new_attitude_ned_degs[ind] > 180:
          new_attitude_ned_degs[ind] = new_attitude_ned_degs[ind] - 360
      rospy.loginfo('')
      rospy.loginfo("RBX_IF: Attitude Input Conditioned NED Degrees")
      rospy.loginfo("RBX_IF:  Roll, Pitch, Yaw")
      rospy.loginfo(["%.2f" % new_attitude_ned_degs[0],"%.2f" % new_attitude_ned_degs[1],"%.2f" % new_attitude_ned_degs[2]])
      ##############################################
      # Convert NED attitude to Pose
      ##############################################
      # Convert to ROS ENU attitude degs and create ENU quaternion setpoint attitude goal
      yaw_enu_deg = nepi_nav.convert_yaw_ned2enu(new_attitude_ned_degs[2])
      new_attitude_enu_degs = [new_attitude_ned_degs[0],new_attitude_ned_degs[1],yaw_enu_deg]
      rospy.loginfo('')
      rospy.loginfo("RBX_IF: Attitude Goal ENU Degrees")
      rospy.loginfo("RBX_IF:  Roll, Pitch, Yaw")
      rospy.loginfo(["%.2f" % new_attitude_enu_degs[0],"%.2f" % new_attitude_enu_degs[1],"%.2f" % new_attitude_enu_degs[2]])
      ##############################################
      ## Send Setpoint Message and Check for Success
      ##############################################
      rospy.loginfo('')
      rospy.loginfo("RBX_IF: Sending Setpoint Attitude Command at 50 Hz and")
      rospy.loginfo("RBX_IF: Waiting for Attitude Setpoint to complete")
      setpoint_attitude_reached = False
      stabilize_timer=0
      timeout_timer = 0 # Initialize timeout timer
      print_timer = 0
      attitude_errors = [] # Initialize running list of errors
      while setpoint_attitude_reached is False and not rospy.is_shutdown():  # Wait for setpoint goal to be set
        if timeout_timer > timeout_sec:
          self.update_error_msg("Setpoint cmd timed out")
          cmd_success = False
          break
        time2sleep = 0.02
        time.sleep(time2sleep) # update setpoint position at 50 Hz
        stabilize_timer=stabilize_timer+time2sleep # Increment rospy.loginfo message timer
        timeout_timer = timeout_timer+time2sleep
        self.gotoPoseFunction(new_attitude_enu_degs)
        # Calculate setpoint attitude errors
        cur_attitude_ned_degs = [self.current_orientation_ned_degs[0],self.current_orientation_ned_degs[1],self.current_orientation_ned_degs[2]]
        attitude_errors_degs = np.array(new_attitude_ned_degs) - np.array(cur_attitude_ned_degs)
        for ind in range(3):
          if input_attitude_ned_degs[ind] == -999.0: # Ignore error check if set to current
            attitude_errors_degs[ind]=0.0
        max_attutude_error_deg = max(abs(attitude_errors_degs))
        # Check for setpoint position local point goal
        if  setpoint_attitude_reached is False:
          if stabilize_timer > self.rbx_info.error_bounds.max_stabilize_time_s:
            max_attitude_errors = max(attitude_errors) # Get max from error window
            attitude_errors = [max_attutude_error_deg] # reset running list of errors
          print_timer = print_timer + time2sleep
          if print_timer > 1:
            print_timer = 0
            rospy.loginfo("RBX_IF: ")
            rospy.loginfo("RBX_IF: Goto Pose Updates")
            # rospy.loginfo some information
            rospy.loginfo('')
            rospy.loginfo("RBX_IF: Current Attitude NED Degrees")
            rospy.loginfo("RBX_IF:  Roll, Pitch, Yaw")
            rospy.loginfo(["%.2f" % self.current_orientation_ned_degs[0],"%.2f" % self.current_orientation_ned_degs[1],"%.2f" % self.current_orientation_ned_degs[2]])
            rospy.loginfo('')
            rospy.loginfo("RBX_IF: Current Goal NED Degrees")
            rospy.loginfo(["%.2f" % new_attitude_ned_degs[0],"%.2f" % new_attitude_ned_degs[1],"%.2f" % new_attitude_ned_degs[2]])
            rospy.loginfo('')
            rospy.loginfo("RBX_IF: Current Attitude Errors")
            rospy.loginfo(["%.3f" % attitude_errors_degs[0],"%.3f" % attitude_errors_degs[1],"%.3f" % attitude_errors_degs[2]])
            rospy.loginfo("RBX_IF: Max Error from Stabilized Check Window Meters")
            rospy.loginfo(["%.2f" % max_attitude_errors])
            if max_attitude_errors < self.rbx_info.error_bounds.max_rotation_error_deg:
              rospy.loginfo('')
              rospy.loginfo("RBX_IF: Attitude Setpoint Reached")
              setpoint_attitude_reached = True
          else:
            attitude_errors.append(max_attutude_error_deg) # append last
        # Reset rospy.loginfo timer if past
        if stabilize_timer > self.rbx_info.error_bounds.max_stabilize_time_s:
          stabilize_timer=0 # Reset rospy.loginfo timer
        self.update_current_errors( [0,0,0,0,attitude_errors_degs[0],attitude_errors_degs[1],attitude_errors_degs[2]]  )
      if cmd_success:
        rospy.loginfo("RBX_IF: ************************")
        rospy.loginfo("RBX_IF: Setpoint Reached")
      self.update_current_errors( [0,0,0,0,0,0,0] )
      self.update_prev_errors( [0,0,0,0,attitude_errors_degs[0],attitude_errors_degs[1],attitude_errors_degs[2]] )
      return cmd_success
      


    ### Function to set and check setpoint position local body command
    ###################################################
    # Input is [X_BODY_METERS, Y_BODY_METERS, Z_BODY_METERS, YEW_BODY_DEGREES]
    # Converted to Local ENU Frame before sending
    # Local Body Position Setpoint Function use these body relative x,y,z,yaw conventions
    # x+ axis is forward
    # y+ axis is right
    # z+ axis is down
    # Only yaw orientation updated
    # yaw+ clockwise, yaw- counter clockwise from x axis (0 degrees faces x+ and rotates positive using right hand rule around z+ axis down)
    #####################################################
    def setpoint_position_local_body(self,setpoint_position,timeout_sec=10):
      # setpoint_position is [X_BODY_METERS, Y_BODY_METERS, Z_BODY_METERS, YEW_BODY_DEGREES]
      # use value 0 for no change
      cmd_success = True
      self.update_current_errors( [0,0,0,0,0,0,0] )
      rospy.loginfo('')
      rospy.loginfo("RBX_IF: Starting Setpoint Position Local Create-Send-Check Process")
      ##############################################
      # Capture Current NavPose Data
      ##############################################
      start_geopoint_wgs84 = list(self.current_location_wgs84_geo)
      rospy.loginfo('')
      rospy.loginfo("RBX_IF: Start Location WSG84 geopoint")
      rospy.loginfo("RBX_IF:  Lat, Long, Alt")
      rospy.loginfo(["%.2f" % start_geopoint_wgs84[0],"%.2f" % start_geopoint_wgs84[1],"%.2f" % start_geopoint_wgs84[2]])
      start_position_ned_m = list(self.current_position_ned_m)
      rospy.loginfo('')
      rospy.loginfo("RBX_IF: Start Position NED degs")
      rospy.loginfo("RBX_IF:  X, Y, Z")
      rospy.loginfo(["%.2f" % start_position_ned_m[0],"%.2f" % start_position_ned_m[1],"%.2f" % start_position_ned_m[2]])   
      start_orientation_ned_degs=list(self.current_orientation_ned_degs)
      rospy.loginfo('')
      rospy.loginfo("RBX_IF: Start Orientation NED degs")
      rospy.loginfo("RBX_IF:  Roll, Pitch, Yaw")
      rospy.loginfo(["%.2f" % start_orientation_ned_degs[0],"%.2f" % start_orientation_ned_degs[1],"%.2f" % start_orientation_ned_degs[2]])
      rospy.loginfo('')
      start_yaw_ned_deg = start_orientation_ned_degs[2]
      rospy.loginfo('')
      rospy.loginfo("RBX_IF: Start Yaw NED degs")
      rospy.loginfo(start_yaw_ned_deg) 
      start_heading_deg=self.current_heading_deg
      rospy.loginfo('')
      rospy.loginfo("RBX_IF: Start Heading degs")
      rospy.loginfo(start_heading_deg)   
      ##############################################
      # Condition Body Input Data
      ##############################################
      # Condition Point Input
      input_point_body_m=setpoint_position[0:3]
      rospy.loginfo('')
      rospy.loginfo("RBX_IF: Point Input Body Meters")
      rospy.loginfo("RBX_IF:  X, Y, Z")
      rospy.loginfo(["%.2f" % input_point_body_m[0],"%.2f" % input_point_body_m[1],"%.2f" % input_point_body_m[2]])
      new_point_body_m=list(input_point_body_m) # No conditioning required
      rospy.loginfo('')
      rospy.loginfo("RBX_IF: Point Conditioned Body Meters")
      rospy.loginfo("RBX_IF:  X, Y, Z")
      rospy.loginfo(["%.2f" % new_point_body_m[0],"%.2f" % new_point_body_m[1],"%.2f" % new_point_body_m[2]])
      # Condition Orienation Input
      input_yaw_body_deg = setpoint_position[3]
      rospy.loginfo('')
      rospy.loginfo("RBX_IF: Yaw Input Body Degrees")
      rospy.loginfo(["%.2f" % input_yaw_body_deg])
      new_yaw_body_deg = input_yaw_body_deg
      # Condition to +-180 deg
      if new_yaw_body_deg > 180:
        new_yaw_body_deg = new_yaw_body_deg - 360
      rospy.loginfo('')
      rospy.loginfo("RBX_IF: Yaw Input Conditioned Body Degrees")
      rospy.loginfo(["%.2f" % new_yaw_body_deg])      
      ##############################################
      # Convert Body Data to NED Data
      ##############################################
      # Set new yaw orientation in NED degrees
      offset_ned_m = nepi_nav.convert_point_body2ned(new_point_body_m,start_yaw_ned_deg)
      rospy.loginfo('')
      rospy.loginfo("RBX_IF: Point Goal Offsets NED Meters")
      rospy.loginfo("RBX_IF:  X, Y, Z")
      rospy.loginfo(["%.2f" % offset_ned_m[0],"%.2f" % offset_ned_m[1],"%.2f" % offset_ned_m[2]])
      new_x_ned_m = start_position_ned_m[0] + offset_ned_m[0]
      new_y_ned_m = start_position_ned_m[1] + offset_ned_m[1]
      new_z_ned_m = start_position_ned_m[2] + offset_ned_m[2]
      new_point_ned_m = [new_x_ned_m,new_y_ned_m,new_z_ned_m]
      rospy.loginfo('')
      rospy.loginfo("RBX_IF: Point Goal NED Meters")
      rospy.loginfo("RBX_IF:  X, Y, Z")
      rospy.loginfo(["%.2f" % new_point_ned_m[0],"%.2f" % new_point_ned_m[1],"%.2f" % new_point_ned_m[2]])
      new_yaw_ned_deg = nepi_nav.convert_yaw_body2ned(new_yaw_body_deg,start_yaw_ned_deg)
      rospy.loginfo('')
      rospy.loginfo("RBX_IF: Yaw Goal NED Degrees")
      rospy.loginfo(["%.2f" % new_yaw_ned_deg])
      ##############################################
      # Convert NED Data to ENU Data
      ##############################################
      # New Point ENU in meters
      new_point_enu_m=Point()
      new_point_enu_m.x = new_point_ned_m[1]
      new_point_enu_m.y = new_point_ned_m[0]
      new_point_enu_m.z = - new_point_ned_m[2]
      rospy.loginfo('')
      rospy.loginfo("RBX_IF: Point Goal ENU Meters")
      rospy.loginfo("RBX_IF:  X, Y, Z")
      rospy.loginfo(["%.2f" % new_point_enu_m.x,"%.2f" % new_point_enu_m.y,"%.2f" % new_point_enu_m.z])
      new_yaw_enu_deg = nepi_nav.convert_yaw_ned2enu(new_yaw_ned_deg)
      rospy.loginfo('')
      rospy.loginfo("RBX_IF: Yaw Goal ENU Degrees")
      rospy.loginfo(["%.2f" % new_yaw_enu_deg])
      ##############################################
      # Create Local ENU Position and Orienation Setpoint Values
      ##############################################
      # New Local Position ENU in meters
      new_point_enu_m=Point()
      new_point_enu_m.x = new_point_enu_m.x
      new_point_enu_m.y = new_point_enu_m.y
      new_point_enu_m.z = new_point_enu_m.z
      rospy.loginfo('')
      rospy.loginfo("RBX_IF: Position Goal ENU Meters")
      rospy.loginfo("RBX_IF:  X, Y, Z")
      rospy.loginfo(["%.2f" % new_point_enu_m.x,"%.2f" % new_point_enu_m.y,"%.2f" % new_point_enu_m.z])
      # New Local Orienation ENU in meters  
      new_orientation_enu_deg = [start_orientation_ned_degs[0],start_orientation_ned_degs[1],new_yaw_enu_deg]
      rospy.loginfo('')
      rospy.loginfo("RBX_IF: Orienation Goal ENU Degrees")
      rospy.loginfo("RBX_IF:  Roll, Pitch, Yaw")
      rospy.loginfo(["%.2f" % new_orientation_enu_deg[0],"%.2f" % new_orientation_enu_deg[1],"%.2f" % new_orientation_enu_deg[2]])
      ##############################################
      ## Send Message and Check for Setpoint Success
      ##############################################

      rospy.loginfo('')
      rospy.loginfo("RBX_IF: Sending Setpoint Position Local Command at 50 Hz and")
      rospy.loginfo("RBX_IF: Waiting for Attitude Setpoint to complete")
      setpoint_position_local_point_reached = False
      setpoint_position_local_yaw_reached = False
      stabilize_timer=0
      point_errors = [] # Initialize running list of errors
      yaw_errors = [] # Initialize running list of errors
      timeout_timer = 0 # Initialize timeout timer
      print_timer_1 = 0
      print_timer_2 = 0
      while setpoint_position_local_point_reached is False or setpoint_position_local_yaw_reached is False and not rospy.is_shutdown():  # Wait for setpoint goal to be set
        if timeout_timer > timeout_sec:
          self.update_error_msg("Setpoint cmd timed out")
          cmd_success = False
          break
        time2sleep = 0.02
        time.sleep(time2sleep) # update setpoint position at 50 Hz
        stabilize_timer=stabilize_timer+time2sleep # Increment rospy.loginfo message timer
        timeout_timer = timeout_timer+time2sleep
        self.gotoPositionFunction(new_point_enu_m,new_orientation_enu_deg)
        # Calculate setpoint position ned errors    
        point_ned_error_m = np.array(self.current_position_ned_m) - np.array(new_point_ned_m)
        for ind in range(3):
          if input_point_body_m == -999: # Ignore error check if set to current
            point_ned_error_m[ind] = 0
        max_point_ned_error_m = np.max(np.abs(point_ned_error_m))
        # Calculate setpoint yaw ned error
        if input_yaw_body_deg == -999: # Ignore error check if set to current
          setpoint_position_local_yaw_reached = True
          max_yaw_ned_error_deg = 0
        else:
          cur_yaw_ned_deg = self.current_orientation_ned_degs[2]
          yaw_ned_error_deg =  cur_yaw_ned_deg - new_yaw_ned_deg
          max_yaw_ned_error_deg = abs(yaw_ned_error_deg)
        # Check for setpoint position local point goal
        if  setpoint_position_local_point_reached is False:
          if stabilize_timer > self.rbx_info.error_bounds.max_stabilize_time_s:
            max_point_errors = max(point_errors) # Get max from error window
            point_errors = [max_point_ned_error_m] # reset running list of errors
          print_timer_1 = print_timer_1 + time2sleep
          if print_timer_1 > 1:
            print_timer_1 = 0
            rospy.loginfo("RBX_IF: Goto Position Position Updates")
            # rospy.loginfo some information every second
            rospy.loginfo('')
            rospy.loginfo("RBX_IF: Current Position NED Meters")
            rospy.loginfo("RBX_IF:  X, Y, Z")
            rospy.loginfo(["%.2f" % self.current_position_ned_m[0],"%.2f" % self.current_position_ned_m[1],"%.2f" % self.current_position_ned_m[2]])
            rospy.loginfo("RBX_IF: Current Goal NED Meters")
            rospy.loginfo("RBX_IF:  X, Y, Z")
            rospy.loginfo(["%.2f" % new_point_ned_m[0],"%.2f" % new_point_ned_m[1],"%.2f" % new_point_ned_m[2]])
            rospy.loginfo("RBX_IF: Current Errors Meters")
            rospy.loginfo("RBX_IF:  X, Y, Z")
            rospy.loginfo(["%.2f" % point_ned_error_m[0],"%.2f" % point_ned_error_m[1],"%.2f" % point_ned_error_m[2]])
            rospy.loginfo("RBX_IF: Max Error from Stabilized Check Window Meters")
            rospy.loginfo(["%.2f" % max_point_errors])
            if max_point_errors < self.rbx_info.error_bounds.max_distance_error_m:
              rospy.loginfo('')
              rospy.loginfo("RBX_IF: Position Setpoint Reached")
              setpoint_position_local_point_reached = True
          else:
            point_errors.append(max_point_ned_error_m) # append last
        # Check for setpoint position yaw point goal
        if  setpoint_position_local_yaw_reached is False:
          if stabilize_timer > self.rbx_info.error_bounds.max_stabilize_time_s:
            max_yaw_errors = max(yaw_errors) # Get max from error window
            yaw_errors = [max_yaw_ned_error_deg] # reset running list of errors
          print_timer_2 = print_timer_2 + time2sleep
          if print_timer_2 > 1:
            print_timer_2 = 0
            rospy.loginfo("RBX_IF: ")
            rospy.loginfo("RBX_IF: Goto Position Yaw Updates")
            # rospy.loginfo some information every second
            rospy.loginfo('')
            rospy.loginfo("RBX_IF: Current Yaw NED Degrees")
            rospy.loginfo(self.current_orientation_ned_degs[2])
            rospy.loginfo("RBX_IF: Current Goal NED Degrees")
            rospy.loginfo(new_yaw_ned_deg)
            rospy.loginfo("RBX_IF: Current Error Degree")
            rospy.loginfo(max_yaw_ned_error_deg)
            rospy.loginfo("RBX_IF: Max Error from Stabilized Check Window Meters")
            rospy.loginfo(["%.2f" % max_yaw_errors])
            if max_yaw_errors < self.rbx_info.error_bounds.max_rotation_error_deg:
              rospy.loginfo('')
              rospy.loginfo("RBX_IF: Yaw Setpoint Reached")
              setpoint_position_local_yaw_reached = True
          else:
            yaw_errors.append(max_yaw_ned_error_deg) # append last
        # Reset rospy.loginfo timer if past
        if stabilize_timer > self.rbx_info.error_bounds.max_stabilize_time_s:
          stabilize_timer=0 # Reset rospy.loginfo timer
        self.update_current_errors(  [point_ned_error_m[0],point_ned_error_m[1],point_ned_error_m[2],0,0,0,max_yaw_ned_error_deg] )
      if cmd_success:
        rospy.loginfo("RBX_IF: ************************")
        rospy.loginfo("RBX_IF: Setpoint Reached")
      self.update_current_errors( [0,0,0,0,0,0,0] )
      self.update_prev_errors(  [point_ned_error_m[0],point_ned_error_m[1],point_ned_error_m[2],0,0,0,max_yaw_ned_error_deg] )
      return cmd_success



    ### Function to set and check setpoint location global geopoint and yaw command
    ###################################################
    # Input is [LAT, LONG, ALT_WGS84, YAW_NED_DEGREES]
    # Converted to AMSL Altitude and ENU Yaw berore sending
    # Altitude is specified as meters above the WGS-84 and converted to AMSL before sending
    # Yaw is specified in NED frame degrees 0-360 or +-180 
    #####################################################
    def setpoint_location_global_wgs84(self,setpoint_location,timeout_sec=10):
      # setpoint_location is [LAT, LONG, ALT_WGS84, YEW_NED_DEGREES 0-360 or +-180]
      # Use value -999 to use current value
      cmd_success = True
      self.update_current_errors( [0,0,0,0,0,0,0] )
      rospy.loginfo('')
      rospy.loginfo("RBX_IF: Starting Setpoint Location Global Create-Send-Check Process")
      ##############################################
      # Capture Current NavPose Data
      ##############################################
      start_geopoint_wgs84 = list(self.current_location_wgs84_geo)  
      rospy.loginfo('')
      rospy.loginfo("RBX_IF: Start Location WSG84 geopoint")
      rospy.loginfo("RBX_IF:  Lat, Long, Alt")
      rospy.loginfo(["%.6f" % start_geopoint_wgs84[0],"%.6f" % start_geopoint_wgs84[1],"%.2f" % start_geopoint_wgs84[2]])
      start_orientation_ned_degs=list(self.current_orientation_ned_degs)
      rospy.loginfo('')
      rospy.loginfo("RBX_IF: Start Orientation NED degs")
      rospy.loginfo("RBX_IF:  Roll, Pitch, Yaw")
      rospy.loginfo(["%.6f" % start_orientation_ned_degs[0],"%.6f" % start_orientation_ned_degs[1],"%.2f" % start_orientation_ned_degs[2]])
      rospy.loginfo('')
      start_yaw_ned_deg = start_orientation_ned_degs[2]
      if start_yaw_ned_deg < 0:
        start_yaw_ned_deg = start_yaw_ned_deg + 360
      rospy.loginfo('')
      rospy.loginfo("RBX_IF: Start Yaw NED degs 0-360")
      rospy.loginfo(start_yaw_ned_deg) 
      start_heading_deg=self.current_heading_deg
      rospy.loginfo('')
      rospy.loginfo("RBX_IF: Start Heading degs")
      rospy.loginfo(start_heading_deg)
      start_geoid_height_m = self.current_geoid_height_m
      ##############################################
      # Condition NED Input Data
      ##############################################
      # Condition Location Input
      input_geopoint_wgs84 = list(setpoint_location[0:3])
      rospy.loginfo('')
      rospy.loginfo("RBX_IF: Location Input Global Geo")
      rospy.loginfo("RBX_IF:  Lat, Long, Alt_WGS84")
      rospy.loginfo(["%.8f" % input_geopoint_wgs84[0],"%.8f" % input_geopoint_wgs84[1],"%.2f" % input_geopoint_wgs84[2]])
      new_geopoint_wgs84=list(start_geopoint_wgs84) # Initialize with start
      for ind in range(3): # Overwrite current with new if set and valid
        if input_geopoint_wgs84[ind] != -999:
          new_geopoint_wgs84[ind]=input_geopoint_wgs84[ind]
      rospy.loginfo('')
      rospy.loginfo("RBX_IF: Location Input Conditioned Global Geo")
      rospy.loginfo("RBX_IF:  Lat, Long, Alt_WGS84")
      rospy.loginfo(["%.8f" % new_geopoint_wgs84[0],"%.8f" % new_geopoint_wgs84[1],"%.2f" % new_geopoint_wgs84[2]])
      # Condition Yaw Input
      input_yaw_ned_deg = setpoint_location[3]
      rospy.loginfo('')
      rospy.loginfo("RBX_IF: Yaw Input NED Degrees")
      rospy.loginfo(["%.2f" % input_yaw_ned_deg])
      new_yaw_ned_deg = start_yaw_ned_deg # Initialize to current
      if input_yaw_ned_deg != -999: # Replace if not -999
        new_yaw_ned_deg = input_yaw_ned_deg
      # Condition to 0-360 degs
      if new_yaw_ned_deg < 0:
        new_yaw_ned_deg = new_yaw_ned_deg + 360
      rospy.loginfo('')
      rospy.loginfo("RBX_IF: Yaw Input Conditioned NED Degrees 0-360")
      rospy.loginfo(["%.2f" % new_yaw_ned_deg])      
      ##############################################
      # Create Global AMSL Location and NED Orienation Setpoint Values
      ##############################################
      # New Global location ENU in meters
      new_geopoint_amsl=GeoPoint()
      new_geopoint_amsl.latitude = new_geopoint_wgs84[0]
      new_geopoint_amsl.longitude = new_geopoint_wgs84[1]
      new_geopoint_amsl.altitude = new_geopoint_wgs84[2] + start_geoid_height_m
      rospy.loginfo('')
      rospy.loginfo("RBX_IF: Location Goal AMSL Meters")
      rospy.loginfo("RBX_IF:  Lat, Long, Alt_AMSL")
      rospy.loginfo(["%.8f" % new_geopoint_amsl.latitude,"%.8f" % new_geopoint_amsl.longitude,"%.2f" % new_geopoint_amsl.altitude])
      # New Local Orienation NED in degs  
      new_orientation_ned_deg = [start_orientation_ned_degs[0],start_orientation_ned_degs[1],new_yaw_ned_deg]
      rospy.loginfo('')
      rospy.loginfo("RBX_IF: Orienation Goal NED Degrees")
      rospy.loginfo("RBX_IF:  Roll, Pitch, Yaw")
      rospy.loginfo(["%.2f" % new_orientation_ned_deg[0],"%.2f" % new_orientation_ned_deg[1],"%.2f" % new_orientation_ned_deg[2]])
      ##############################################
      ## Send Message and Check for Setpoint Success
      ##############################################
  
      rospy.loginfo("RBX_IF: Sending MAVLINK Setpoint Position Local Command at 50 Hz and")
      rospy.loginfo("RBX_IF:  checking for Setpoint Reached")
      setpoint_location_global_geopoint_reached = False
      setpoint_location_global_yaw_reached = False 
      rospy.loginfo('')
      rospy.loginfo("RBX_IF: Waiting for Position Local Setpoint to complete")
      stabilize_timer=0
      geopoint_errors = [] # Initialize running list of errors
      yaw_errors = [] # Initialize running list of errors
      timeout_timer = 0 # Initialize timeout timer
      print_timer_1 = 0
      print_timer_2 = 0
      while setpoint_location_global_geopoint_reached is False or setpoint_location_global_yaw_reached is False and not rospy.is_shutdown(): # Wait for setpoint goal to be set
        if timeout_timer > timeout_sec:
          self.update_error_msg("Setpoint cmd timed out")
          cmd_success = False
          break
        time2sleep = 0.02
        time.sleep(time2sleep) # update setpoint position at 50 Hz
        stabilize_timer=stabilize_timer+time2sleep # Increment self.update_error_msg message timer
        timeout_timer = timeout_timer+time2sleep
        self.gotoLocationFunction(new_geopoint_amsl,new_orientation_ned_deg)
        
        # Calculate setpoint position and yaw errors
        geopoint_errors_geo = np.array(self.current_location_wgs84_geo) - np.array(new_geopoint_wgs84)
        geopoint_errors_m = [geopoint_errors_geo[0]*111139,geopoint_errors_geo[1]*111139,geopoint_errors_geo[2]]
        for ind in range(3):  # Ignore error check if set to current
          if input_geopoint_wgs84[ind] == -999.0:
            geopoint_errors_m[ind] = 0
        max_geopoint_error_m = np.max(np.abs(geopoint_errors_m))
        if input_yaw_ned_deg == -999: # Ignore error check if set to current
          setpoint_location_global_yaw_reached = True
          max_yaw_ned_error_deg = 0
        else:
          cur_yaw_ned_deg = self.current_orientation_ned_degs[2]
          if cur_yaw_ned_deg < 0:
            cur_yaw_ned_deg = cur_yaw_ned_deg + 360
          yaw_ned_error_deg =  cur_yaw_ned_deg - new_yaw_ned_deg
          max_yaw_ned_error_deg = abs(yaw_ned_error_deg)
        # Check for setpoint position global goal
        if  setpoint_location_global_geopoint_reached is False:
          if stabilize_timer > self.rbx_info.error_bounds.max_stabilize_time_s:
            max_geopoint_errors = max(geopoint_errors) # Get max from error window
            geopoint_errors = [max_geopoint_error_m] # reset running list of errors
          print_timer_1 = print_timer_1 + time2sleep
          if print_timer_1 > 1:
            print_timer_1 = 0
            rospy.loginfo("RBX_IF: ")
            rospy.loginfo("RBX_IF: Goto Location Location Updates")
            #rospy.loginfo(self.rbx_status.errors_current)
            # rospy.loginfo some information every second
            rospy.loginfo('')
            rospy.loginfo("RBX_IF: Current Location WGS84")
            rospy.loginfo("RBX_IF:  Lat, Long, Alt_WGS84")
            rospy.loginfo(["%.7f" % self.current_location_wgs84_geo[0],"%.7f" % self.current_location_wgs84_geo[1],"%.2f" % self.current_location_wgs84_geo[2]])
            rospy.loginfo("RBX_IF: Current Goal WGS84")
            rospy.loginfo("RBX_IF:  Lat, Long, Alt_WGS84")
            rospy.loginfo(["%.7f" % new_geopoint_wgs84[0],"%.7f" % new_geopoint_wgs84[1],"%.2f" % new_geopoint_wgs84[2]])
            rospy.loginfo("RBX_IF: Current Errors Meters")
            rospy.loginfo("RBX_IF:  Lat, Long, Alt")
            rospy.loginfo(["%.2f" % geopoint_errors_m[0],"%.2f" % geopoint_errors_m[1],"%.2f" % geopoint_errors_m[2]])
            rospy.loginfo("RBX_IF: Max Error from Stabilized Check Window Meters")
            rospy.loginfo(["%.2f" % max_geopoint_errors])
            if max_geopoint_errors < self.rbx_info.error_bounds.max_distance_error_m:
              rospy.loginfo('')
              rospy.loginfo("RBX_IF: Location Setpoint Reached")
              setpoint_location_global_geopoint_reached = True
          else:
            geopoint_errors.append(max_geopoint_error_m) # append last
        # Check for setpoint position yaw goal
        if  setpoint_location_global_yaw_reached is False:
          if stabilize_timer > self.rbx_info.error_bounds.max_stabilize_time_s:
            max_yaw_errors = max(yaw_errors) # Get max from error window
            yaw_errors = [max_yaw_ned_error_deg] # reset running list of errors
          print_timer_2 = print_timer_2 + time2sleep
          if print_timer_2 > 1:
            print_timer_2 = 0
            rospy.loginfo("RBX_IF: ")
            rospy.loginfo("RBX_IF: Goto Location Yaw Updates")
            # rospy.loginfo some information every second
            rospy.loginfo('')
            rospy.loginfo("RBX_IF: Current Yaw NED Degrees")
            rospy.loginfo(cur_yaw_ned_deg)
            rospy.loginfo("RBX_IF: Current Goal NED Degrees")
            rospy.loginfo(new_yaw_ned_deg)
            rospy.loginfo("RBX_IF: Current Error Degree")
            rospy.loginfo(max_yaw_ned_error_deg)
            rospy.loginfo("RBX_IF: Max Error from Stabilized Check Window Degs")
            rospy.loginfo(["%.2f" % max_yaw_errors])
            if max_yaw_errors < self.rbx_info.error_bounds.max_rotation_error_deg:
              rospy.loginfo('')
              rospy.loginfo("RBX_IF: Yaw Setpoint Reached")
              setpoint_location_global_yaw_reached = True
          else:
            yaw_errors.append(max_yaw_ned_error_deg) # append last
        # Reset rospy.loginfo timer if past
        if stabilize_timer > 1:
          stabilize_timer=0 # Reset rospy.loginfo timer
        self.update_current_errors( [geopoint_errors_m[0],geopoint_errors_m[1],geopoint_errors_m[2],0,0,0,max_yaw_ned_error_deg] )
      if cmd_success:
        rospy.loginfo("RBX_IF: ************************")
        rospy.loginfo("RBX_IF: Setpoint Reached")
      self.update_current_errors( [0,0,0,0,0,0,0] )
      self.update_prev_errors( [geopoint_errors_m[0],geopoint_errors_m[1],geopoint_errors_m[2],0,0,0,max_yaw_ned_error_deg] )
      return cmd_success

    #######################
    # Class Utility Functions
    
    ### Function for updating current goto error values
    def update_current_errors(self,error_list):
      if len(error_list) == 7:
        self.rbx_status.errors_current.x_m = error_list[0]
        self.rbx_status.errors_current.y_m = error_list[1]
        self.rbx_status.errors_current.z_m = error_list[2]
        self.rbx_status.errors_current.heading_deg = error_list[3]
        self.rbx_status.errors_current.roll_deg = error_list[4]
        self.rbx_status.errors_current.pitch_deg = error_list[5]
        self.rbx_status.errors_current.yaw_deg = error_list[6]
      else:
        rospy.loginfo("RBX_IF: Skipping current error update. Error list to short")

    ### Function for updating last goto error values
    def update_prev_errors(self,error_list):
      if len(error_list) == 7:
        self.rbx_status.errors_prev.x_m = error_list[0]
        self.rbx_status.errors_prev.y_m = error_list[1]
        self.rbx_status.errors_prev.z_m = error_list[2]
        self.rbx_status.errors_prev.heading_deg = error_list[3]
        self.rbx_status.errors_prev.roll_deg = error_list[4]
        self.rbx_status.errors_prev.pitch_deg = error_list[5]
        self.rbx_status.errors_prev.yaw_deg = error_list[6]
      else:
        rospy.loginfo("RBX_IF: Skipping current error update. Error list to short")

    def update_error_msg(self,error_msg):
      rospy.loginfo(error_msg)
      self.rbx_status.last_error_message = error_msg

    def get_motor_controls_status_msg(self,motor_controls):
      mcs = []
      for i in range(len(motor_controls)):
        mcs.append([str(i),str(motor_controls[i])])
      return str(mcs)


       


    
