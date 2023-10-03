#!/bin/bash

# This script archives the NUID and associated SSH key pair to the user partition (nepi_storage).
# It should be run before updating a complete system image. Run it from an image partition that
# has the desirable NUID configuration to be restored after the update.

ARCHIVE_PATH="/mnt/nepi_storage/nepi_full_img_archive/nepi-bot/devinfo"
FS_PATH="/opt/nepi/nepi_link/nepi-bot/devinfo"

mkdir -p $ARCHIVE_PATH
if [ $? -ne 0 ]
then
  echo "Failed to create archive location $ARCHIVE_PATH" >$2
  exit
fi

cp $FS_PATH/* $ARCHIVE_PATH
if [ $? -ne 0 ]
then
  echo "Failed to transfer nepi-bot files from $FS_PATH to $ARCHIVE_PATH" > $2
  exit
fi

exit 0
