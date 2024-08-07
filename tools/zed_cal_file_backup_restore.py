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
from nepi_edge_sdk_base import nepi_ros

     
if __name__ == '__main__':
  CAL_SRC_PATH = "/usr/local/zed/settings"
  USER_CFG_PATH = "/mnt/nepi_storage/user_cfg"
  CAL_BACKUP_PATH = USER_CFG_PATH + "/zed_cals"
  # Try to backup camera calibration files
  [success,files_copied,files_not_copied] = nepi_ros.copy_files_from_folder(CAL_SRC_PATH,CAL_BACKUP_PATH)
  if success:
    #print("Backed up zed cal files")
    if len(files_copied) > 0:
      strList = str(files_copied)
      print("Backed up zed cal files: " + strList)
  else:
    print("Failed to back up up zed cal files")


    # Try to restore camera calibration files from
  [success,files_copied,files_not_copied] = nepi_ros.copy_files_from_folder(CAL_BACKUP_PATH,CAL_SRC_PATH)
  if success:
    if len(files_copied) > 0:
      strList = str(files_copied)
      print("Restored zed cal files: " + strList)
  else:
    print("Failed to restore zed cal files")


