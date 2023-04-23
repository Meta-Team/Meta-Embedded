#!/bin/sh
# ----------------------------------------------------------------
# This script invokes OpenOCD to write the program and reset the chip
#
# Initial version. July 16 2018 liuzikai
# Move reset config to .cfg file and support --no-exit argument. Jan 30 2019 liuzikai
# 
# Add explicit flash function. April 16 2023 Tony Zhang
#
# ----------------------------------------------------------------

config_base=$(dirname $(realpath $0))
cd "$config_base"

# default behavior: exit
exit_cmd="exit"
# default flash file: hero
flash_file="../build/HERO.elf"
# default flash config file: RM_Board_Win.cfg
config_file="RM_Board_Win.cfg"



while [ $# -gt 0 ]
do
	case "$1" in
		--no-exit)
			exit_cmd=""
			;;
		--cfg=*)
			cfg="${1#*=}"
			case "$cfg" in
				windows)
					config_file="RM_Board_Win.cfg"
					;;
				macos)
					config_file="RM_Board_Mac.cfg"
					;;
				linux)
					config_file="RM_Board_Linux.cfg"
					;;
				*)
					echo "invalid --cfg option $cfg, valid options:"
					echo "windows macos linux"
					echo "exit..."
					exit
					;;
			esac
			;;
		"hero")
			flash_file="../build/HERO.elf"
			;;
		"infantry4")
			flash_file="../build/INFANTRY_FOUR.elf"
			;;
		"infantry3")
			flash_file="../build/INFANTRY_THREE.elf"
			;;
		"sentry")
			flash_file="../build/SENTRY.elf"
			;;
		"autosentry")
			flash_file="../build/AUTO_SENTRY.elf"
			;;
		"ca_ahrs_hero")
			flash_file="../build/ca_ahrs_hero.elf"
			;;
		"ca_ahrs_infantry")
			flash_file="../build/ca_ahrs_infantry.elf"
			;;
		"pa_gimbal")
			flash_file="../build/pa_gimbal.elf"
			;;
		"ut_ahrs")
			flash_file="../build/ut_ahrs.elf"
			;;
		"ut_ahrs_ext")
			flash_file="../build/ut_ahrs_ext.elf"
			;;
		"ut_blink")
			flash_file="../build/ut_blink.elf"
			;;
		"ut_buzzer")
			flash_file="../build/ut_buzzer.elf"
			;;
		"ut_chassis")
			flash_file="../build/ut_chassis.elf"
			;;
		"ut_oled")
			flash_file="../build/ut_oled.elf"
			;;
		"ut_remote_interpreter")
			flash_file="../build/ut_remote_interpreter.elf"
			;;
		"ut_sd_card")
			flash_file="../build/ut_sd_card.elf"
			;;
		*)
			echo "$0:Invalid argument $1, valid options:"
			echo "hero infantry4 infantry3 sentry auto_sentry"
			echo "ca_ahrs_hero ca_ahrs_infantry"
			echo "pa_gimbal"
			echo "ut_ahrs ut_ahrs_ext ut_blink ut_buzzer ut_chassis ut_oled" 
			echo "ut_remote_interpreter ut_sd_card"
			exit
			;;
	esac
	shift
done
# check file existence
if [ ! -f "$flash_file" ];
	then
		echo "flash file $flash_file does not exist"
		echo "exit now..."
	exit
fi

if [ "$exit_cmd" = "" ]; then
    echo "[Notice] OpenOCD will flash, reset and halt the program, and keep running as server after writing program."
fi
set -x
openocd -f "$config_file" -c "program $flash_file verify reset $exit_cmd"

