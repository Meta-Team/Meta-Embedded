#!/bin/sh

# ----------------------------------------------------------------
# This script invokes openocd to write the program and reset the chip
#
# Author: liuzikai
# Date: July 16 2018
#
# Notice: working directory should be set to current directory (config)
# ----------------------------------------------------------------

openocd -f RM_Board_2017.cfg -c "reset_config trst_only combined" -c "program ../dev/.build/ch.elf verify reset exit"
