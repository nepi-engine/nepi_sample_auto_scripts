#!/bin/bash

# This bash scrict updates a NEPI device's Serial Number
# You must reboot your device after updating for changes to take effect

#####################################################
### Edit Serial Number Below
#####################################################
###--Serial Number Updating ------------------------------------------------------
Serial_Number=100100
#####################################################

rm /opt/nepi/.sys_env.bash.swp
sed -i "s/DEVICE_SN=.*/DEVICE_SN=$Serial_Number/" /opt/nepi/sys_env.bash

