#!/bin/bash

# This bash scrict updates a NEPI device's CONNECT Topic List
# You must reboot your device after updating for changes to take effect

filename=nepi_link_ros_bridge.yaml.user
cd /opt/nepi/ros/etc/nepi_link_ros_bridge/
cat > $filename <<EOF
#####################################
# Edit the user CONNECT config content below
#####################################
auto_attempts_per_hour: 6
enabled: true
hb: {auto_data_offload: false, data_source_folder: /mnt/nepi-storage/data, enabled: false,
  session_max_time_s: 300}
lb:
  available_data_sources:
#####################################
# Edit Your Camera Image Topic List Here, Comment or Delete Devices Not Present, Add New Devices If Needed
#####################################
  - {conversion_call: lb_convert_image, enabled: true, msg_mod: sensor_msgs.msg, msg_type: Image,
    snippet_id: 0, snippet_type: img, topic: nexigo_n60_fhd_webcam_audio/idx/color_2d_image}
#  - {conversion_call: lb_convert_image, enabled: false, msg_mod: sensor_msgs.msg, msg_type: Image,
#    snippet_id: 0, snippet_type: img, topic: see3cam_cu81/idx/color_2d_image}
#  - {conversion_call: lb_convert_image, enabled: false, msg_mod: sensor_msgs.msg, msg_type: Image,
#    snippet_id: 0, snippet_type: img, topic: econ_routecam/idx/color_2d_image}
#  - {conversion_call: lb_convert_image, enabled: false, msg_mod: sensor_msgs.msg, msg_type: Image,
#    snippet_id: 0, snippet_type: img, topic: onwote_hd_poe/idx/color_2d_image}
#  - {conversion_call: lb_convert_image, enabled: false, msg_mod: sensor_msgs.msg, msg_type: Image,
#    snippet_id: 0, snippet_type: img, topic: sidus_ss400/idx/color_2d_image}
#####################################
# Edit Other System Topic List Here
#####################################
  - {conversion_call: lb_ros_msg_to_yaml, enabled: true, service: nav_pose_query,
    service_mod: nepi_ros_interfaces.srv, service_type: NavPoseQuery, snippet_id: 0,
    snippet_type: nav}
  - {conversion_call: lb_ros_msg_to_yaml, enabled: false, msg_mod: darknet_ros_msgs.msg,
    msg_type: BoundingBoxes, snippet_id: 0, snippet_type: bbx, topic: classifier/bounding_boxes}
  - {conversion_call: lb_convert_image, enabled: false, msg_mod: sensor_msgs.msg,
    msg_type: Image, snippet_id: 5, snippet_type: img, topic: classifier/detection_image}
  - {conversion_call: lb_convert_image, enabled: false, msg_mod: sensor_msgs.msg,
    msg_type: Image, snippet_id: 6, snippet_type: img, topic: target_localizer/localization_image}
#####################################
# Edit Other CONNECT Config Items Here If Needed
#####################################
  data_sets_per_hour: 12
  enabled: true
  max_data_wait_time_s: 10.0
  session_max_time_s: 60
nepi_bot_root_folder: /opt/nepi/nepi_link/nepi-bot
nepi_log_storage_enabled: false
nepi_log_storage_folder: /mnt/nepi-storage/data/nepi-bot
reboot_sys_on_sw_update: true
snapshot_event: {onboard_save_duration_s: 2.0, onboard_save_rate_hz: 1.0, triggers_lb: true}
#####################################
# Don't Edit Past This Point
#####################################

EOF

# make sure symbolic link for CONNECT topics points to this file
ln -sf nepi_link_ros_bridge.yaml.user  nepi_link_ros_bridge.yaml
