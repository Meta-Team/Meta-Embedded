#!/bin/sh

# ----------------------------------------------------------------
# This script invokes openocd as the dbg server
# Chip won't get over-written or reset
#
# Author: liuzikai
# Date: July 16 2018
#
# Notice: working directory should be set to current directory (config)
#         this script should be run manually before arm-embi-gdb and terminated manually after debugging
# ----------------------------------------------------------------

openocd -f RM_Board_2017.cfg