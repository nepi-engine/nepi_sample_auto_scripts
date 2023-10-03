#!/bin/bash

# This script restores a previously archived NUID and associated SSH key pair.
# It should be run AFTER updating a complete system image. Run it from the image partition that
# was updated (so typically after the post-install reboot)

ARCHIVE_PATH="/mnt/nepi_storage/nepi_full_img_archive/nepi-bot/devinfo"
FS_PATH="/opt/nepi/nepi_link/nepi-bot/devinfo"

if [ -d $ARCHIVE_PATH ]
then
  cp $ARCHIVE_PATH/* $FS_PATH
  chown nepi:nepi $FS_PATH/* 
  exit $?
fi

# If we get here, failed to find archive path
echo "Unable to find archive path $ARCHIVE_PATH" > $2
exit 1

