#!/bin/sh

# ----------------------------------------------------------------
# This script invokes openocd as the dbg server using pipe
#
# Author: liuzikai
# Date: Aug 6 2018
#
# NOTICE: Please change the following directory to the absolute path of the project root BEFORE using this script.
# ----------------------------------------------------------------

cd /Users/liuzikai/Documents/RoboMaster/Meta-Infantry
openocd -l openocd.log -c "gdb_port pipe" -f config/RM_Board_Mac.cfg
