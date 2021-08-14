#!/bin/sh
# ----------------------------------------------------------------
# This script invokes OpenOCD to write the program and reset the chip
#
# Initial version. July 16 2018 liuzikai
# Move reset config to .cfg file and support --no-exit argument. Jan 30 2019 liuzikai
# ----------------------------------------------------------------

if [[ $1 != "--no-exit" ]]; then
    exit_cmd="exit"
else
    exit_cmd=""
    echo "[Notice] OpenOCD will flash, reset and halt the program, and keep running as server after writing program."
fi
set -x
openocd -f RM_Board_Mac.cfg -c "program ../build/meta.elf verify reset $exit_cmd"
