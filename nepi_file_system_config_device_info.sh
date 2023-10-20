#!/bin/bash

# This bash scrict updates a NEPI device's information
# You must reboot your device after updating for changes to take effect

rm /opt/nepi/.sys_env.bash.swp
#####################################################
### Edit Below, Or Comment Out Lines to Skip
#####################################################
###--Serial Number Updating ------------------------------------------------------
Serial_Number=100100
sed -i "s/DEVICE_SN=.*/DEVICE_SN=$Serial_Number/" /opt/nepi/sys_env.bash
###--Device ID Updating ------------------------------------------------------
ID=s2x
sed -i "s/DEVICE_ID=.*/DEVICE_ID=$ID/" /opt/nepi/sys_env.bash
#####################################################




