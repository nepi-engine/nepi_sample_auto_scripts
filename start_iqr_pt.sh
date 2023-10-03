#!/bin/bash

# This script starts IQR PT detect process.

source /opt/nepi/ros/setup.bash
export ROS_NAMESPACE=/nepi/s2x
rosrun nepi_edge_sdk_ptx iqr_ros_pan_tilt_node

