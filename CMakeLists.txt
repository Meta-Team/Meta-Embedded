# Created by liuzikai 2018-01-29

# =========== General Part ===========

cmake_minimum_required(VERSION 3.10)

include(config/toolchain.cmake)  # include cross compile configurations

set(CMAKE_CXX_STANDARD 11)

# Check dev board definition
# NOTICE: ${BOARD_NAME} needs to be specific externally
# NOTICE: if you want to adapt this project to a new board, be sure to also set startup file in compile_options.cmake
if (NOT DEFINED BOARD_NAME)
    set(BOARD_NAME rm_board_2018_a)
    #    message(FATAL_ERROR "[ERROR] No specific dev board. Do you forget to add -DBOARD_NAMD=\"***\" to CMake options?")
endif ()

if ((${BOARD_NAME} STREQUAL rm_board_2017) OR (${BOARD_NAME} STREQUAL rm_board_2018_a))
    message("[Notice] Current profile is configurated for dev board \"${BOARD_NAME}\".")
else ()
    message(FATAL_ERROR "[ERROR] Dev board \"${BOARD_NAME}\" is not configured.")
endif ()

project(Meta_Infantry ASM C CXX)
# NOTICE: project() must be in CMakeList.txt, rather than any cmake file. It must be after toolchain configs and before
#         flag configs for CMake to test compiler.

include(config/compile_options.cmake)  # include compile flags and startup file option


# =========== Options ===========

set(CMAKE_VERBOSE_MAKEFILE OFF)  # Enable this if you want to see verbose log


# =========== Common Files and Targets Configurations ===========

include(os/ch.cmake)  # add files and directories from ChibiOS

add_subdirectory(cmsis)

include_directories(  # common include directories
        dev
        dev/common
        dev/debug
        dev/debug/shell
        dev/interface
        dev/interface/ahrs
        dev/module
        dev/scheduler
        dev/logic
        dev/vehicle)

set(BASE_SRC  # common cpp sources, which are included to every target at post configuration below
        dev/debug/shell/shell.cpp
        dev/debug/shell/shell_base.c
        dev/debug/shell/shell_base_cmd.c
        dev/debug/shell/shell_dbg_cmd.cpp
        dev/debug/shell/printf.c
        dev/interface/led.cpp
        dev/interface/buzzer_interface.cpp
        dev/scheduler/buzzer_scheduler.cpp)

set(VEHICLE_COMMON_SRC
        dev/module/pid_controller.hpp
        dev/interface/can_interface.cpp
        dev/interface/remote_interpreter.cpp
        dev/common/CRC8.cpp
        dev/common/CRC16.cpp
        dev/interface/sd_card_interface.cpp)

set(AHRS_SRC
        dev/interface/ahrs/ahrs_math.hpp
        dev/interface/ahrs/ahrs.cpp
        dev/interface/ahrs/ahrs_lib_middleware.c
        dev/interface/ahrs/imu_on_board.cpp)

set(AHRS_LIB
        ${PROJECT_SOURCE_DIR}/dev/interface/ahrs/ahrs_lib.lib)

set(CHASSIS_SRC
        dev/interface/chassis_interface.cpp
        dev/scheduler/chassis_scheduler.cpp
        dev/logic/chassis_logic.cpp)

set(VISION_SRC
        dev/interface/vision_interface.cpp
        dev/scheduler/vision_scheduler.cpp
#        dev/interface/vision.cpp
        )

set(REFEREE_SRC
        dev/interface/referee_interface.cpp
        dev/scheduler/referee_UI_update_scheduler.cpp
        dev/logic/referee_UI_logic.cpp)

include_directories(dev/board/${BOARD_NAME})  # include board configs

# Reuse ChibiOS object files (without linking) to avoid repeat compile
# They are included to every target at post configuration below
add_library(ChibiOS OBJECT ${CHIBIOS_XASM_SRC} ${CHIBIOS_C_SRC} ${CHIBIOS_CPP_SRC} dev/board/${BOARD_NAME}/board.c)


# ========================== Files and Targets Configurations ==========================

# NOTICE: UPDATE README IF THE CONFIGURATIONS IS UPDATED

# ========================== Robot's Control Programs ===========================
set(INFANTRY_SRC
        ${VEHICLE_COMMON_SRC}
        ${AHRS_SRC}
        ${CHASSIS_SRC}
        ${VISION_SRC}
        ${REFEREE_SRC}

        # Infantry gimbal
        dev/interface/gimbal_interface.cpp
        dev/scheduler/gimbal_scheduler.cpp
        dev/logic/gimbal_logic.cpp

        # Infantry shoot
        dev/scheduler/shoot_scheduler.cpp
        dev/logic/shoot_logic.cpp

        # Infantry has super capacitor
        dev/interface/super_capacitor_port.cpp

        # Main
        dev/vehicle/infantry/inspector_infantry.cpp
        dev/vehicle/infantry/user_infantry.cpp
        dev/vehicle/infantry/main_infantry.cpp)

set(INFANTRY_DEFS
        INFANTRY
        INFANTRY_GIMBAL_ENABLE=1
        INFANTRY_CHASSIS_ENABLE=1
        INFANTRY_SUPER_CAPACITOR_ENABLE=1
        INFANTRY_VISION_ENABLE=1)

set(INFANTRY_LIBS
        ${AHRS_LIB})

# Main Program -- Infantry #3
add_executable(INFANTRY_THREE ${INFANTRY_SRC})
target_include_directories(INFANTRY_THREE PRIVATE dev/vehicle/infantry)
target_compile_definitions(INFANTRY_THREE PRIVATE ${INFANTRY_DEFS} INFANTRY_THREE)
target_link_libraries(INFANTRY_THREE ${INFANTRY_LIBS})

# Main Program -- Infantry #4
add_executable(INFANTRY_FOUR ${INFANTRY_SRC})
target_include_directories(INFANTRY_FOUR PRIVATE dev/vehicle/infantry)
target_compile_definitions(INFANTRY_FOUR PRIVATE ${INFANTRY_DEFS} INFANTRY_FOUR)
target_link_libraries(INFANTRY_FOUR ${INFANTRY_LIBS})

# Main Program -- Infantry #5
add_executable(INFANTRY_FIVE ${INFANTRY_SRC})
target_include_directories(INFANTRY_FIVE PRIVATE dev/vehicle/infantry)
target_compile_definitions(INFANTRY_FIVE PRIVATE ${INFANTRY_DEFS} INFANTRY_FIVE)
target_link_libraries(INFANTRY_FIVE ${INFANTRY_LIBS})

# Main Program -- Hero
add_executable(HERO
        ${VEHICLE_COMMON_SRC}
        ${AHRS_SRC}
        ${CHASSIS_SRC}
        ${VISION_SRC}
        ${REFEREE_SRC}

        # Hero gimbal
        dev/interface/gimbal_interface.cpp
        dev/scheduler/gimbal_scheduler.cpp
        dev/logic/gimbal_logic.cpp

        # Hero shoot
        dev/scheduler/shoot_scheduler.cpp
        dev/logic/shoot_logic.cpp

        # Hero has super capacitor
        dev/interface/super_capacitor_port.cpp

        # Main
        dev/vehicle/hero/inspector_hero.cpp
        dev/vehicle/hero/user_hero.cpp
        dev/vehicle/hero/main_hero.cpp)
target_include_directories(HERO PRIVATE dev/vehicle/hero)
target_compile_definitions(HERO PRIVATE HERO)
target_link_libraries(HERO ${AHRS_LIB})

# Main Program -- Sentry
add_executable(SENTRY
        dev/interface/can_interface.cpp
        dev/interface/remote_interpreter.cpp
#        dev/interface/referee_interface.cpp
        dev/interface/sd_card_interface.cpp
#        ${VISION_SRC}
#        ${AHRS_SRC}
        dev/common/CRC8.cpp
        dev/common/CRC16.cpp

        dev/interface/new_sentry_chassis_interface.cpp
        dev/scheduler/new_sentry_chassis_scheduler.cpp
        dev/logic/new_sentry_chassis_logic.cpp

#        dev/interface/gimbal_interface.cpp
#        dev/scheduler/gimbal_scheduler.cpp
#        dev/logic/gimbal_logic.cpp

#        dev/scheduler/shoot_scheduler.cpp
#        dev/logic/shoot_logic.cpp

        dev/vehicle/sentry/main_sentry.cpp
#        dev/vehicle/sentry/inspector_sentry.cpp
#        dev/vehicle/sentry/settings_sentry.cpp
        dev/vehicle/sentry/user_sentry.cpp

        )
target_include_directories(SENTRY PRIVATE dev/vehicle/sentry)
target_compile_definitions(SENTRY PRIVATE SENTRY)

#Main Program -- Aerial
add_executable(AERIAL
        dev/module/pid_controller.hpp
        dev/interface/can_interface.cpp
        dev/interface/remote_interpreter.cpp
        dev/interface/sd_card_interface.cpp

        dev/interface/gimbal_interface.cpp
        dev/scheduler/gimbal_scheduler.cpp
        dev/logic/gimbal_logic.cpp

        dev/interface/referee_interface.cpp
        dev/common/CRC8.cpp
        dev/common/CRC16.cpp
        ${VISION_SRC}

        dev/interface/ahrs/ahrs_ext.cpp

        dev/scheduler/shoot_scheduler.cpp
        dev/logic/shoot_logic.cpp

        dev/vehicle/aerial/main_aerial.cpp
        dev/vehicle/aerial/user_aerial.cpp
        dev/vehicle/aerial/inspector_aerial.cpp
        dev/vehicle/aerial/settings_aerial.cpp
        )
target_include_directories(AERIAL PRIVATE dev/vehicle/aerial)
target_compile_definitions(AERIAL PRIVATE AERIAL)

# Main Program -- Engineer
add_executable(ENGINEER
        dev/interface/can_interface.cpp
        dev/interface/remote_interpreter.cpp
        dev/interface/referee_interface.cpp
        dev/interface/sd_card_interface.cpp
        dev/common/CRC8.cpp
        dev/common/CRC16.cpp

        dev/interface/chassis_interface.cpp
        dev/interface/dms_interface.cpp
        dev/interface/engineer_elevator_interface.cpp
        dev/interface/engineer_gimbal.cpp
        dev/interface/robotic_arm_interface.cpp
        dev/scheduler/engineer_chassis_skd.cpp
        dev/scheduler/engineer_elevator_skd.cpp
        dev/scheduler/robotic_arm_skd.cpp
        dev/logic/engineer_elevator_logic.cpp
        dev/vehicle/engineer/main_engineer.cpp
        dev/vehicle/engineer/user_engineer.cpp
        dev/vehicle/engineer/inspector_engineer.cpp
        dev/vehicle/engineer/settings_engineer.cpp
        dev/interface/engineer_interface.cpp
        dev/interface/engineer_interface.h
        dev/interface/air_tank_interface.cpp
        dev/interface/air_tank_interface.h
        dev/scheduler/engineer_rescue_skd.cpp
        dev/scheduler/engineer_rescue_skd.h
        dev/logic/robotic_arm_logic.cpp
        dev/logic/robotic_arm_logic.h)
target_include_directories(ENGINEER PRIVATE dev/vehicle/engineer)
target_compile_definitions(ENGINEER PRIVATE ENGINEER)


# ======================================= Unit Tests Programs =======================================

# Includes Unit Tests and Param Adjust programs

# --------------------------------------- Unit Tests ---------------------------------------
add_executable(ut_shoot
        dev/debug/Unit_Tests/ut_shoot.cpp
        dev/interface/can_interface.cpp
        dev/module/pid_controller.hpp
        dev/interface/remote_interpreter.cpp)

add_executable(ut_engineer
        dev/debug/Unit_Tests/ut_engineer.cpp
        dev/scheduler/engineer_chassis_skd.cpp
        dev/logic/engineer_elevator_logic.cpp
        dev/scheduler/engineer_elevator_skd.cpp
        dev/interface/engineer_elevator_interface.cpp
        dev/scheduler/robotic_arm_skd.cpp
        dev/interface/chassis_interface.cpp
        dev/interface/robotic_arm_interface.cpp
        dev/interface/dms_interface.cpp
        dev/interface/referee_interface.cpp
        dev/interface/can_interface.cpp)

# Blink
add_executable(ut_blink
        dev/module/button_monitor.cpp
        dev/debug/Unit_Tests/ut_led.cpp)

# Remote Interpreter: include remote_interpreter sources and the unit test modules.
add_executable(ut_remote_interpreter
        dev/interface/remote_interpreter.cpp
        dev/debug/Unit_Tests/ut_remoteIF.cpp)

# MPU6500: include MP6500 interface and unit test.
add_executable(ut_mpu6500
        dev/interface/ahrs/imu_on_board.cpp
        dev/debug/Unit_Tests/ut_mpu6500.cpp)

# IST8310
add_executable(ut_ist8310
        dev/interface/ahrs/imu_on_board.cpp
        dev/debug/Unit_Tests/ut_ist8310.cpp)

# IMU: include IMU interface and unit test.
add_executable(ut_imu
        dev/interface/ahrs/ahrs_math.hpp
        dev/interface/ahrs/imu_on_board.cpp
        dev/interface/ahrs/ahrs.cpp
        dev/debug/Unit_Tests/ut_ahrs.cpp)

# AHRS
add_executable(ut_ahrs
        dev/interface/ahrs/ahrs_math.hpp
        dev/interface/ahrs/imu_on_board.cpp
        dev/interface/ahrs/ahrs_lib_middleware.c
        dev/interface/ahrs/ahrs.cpp
        dev/debug/Unit_Tests/ut_ahrs.cpp)
target_link_libraries(ut_ahrs ${AHRS_LIB})

# AHRSExt
add_executable(ut_ahrs_ext
        dev/interface/can_interface.cpp
        dev/interface/ahrs/ahrs_ext.cpp
        dev/debug/Unit_Tests/ut_ahrs_ext.cpp)

# BuzzerSKD: inlcude BuzzerSKD interface and unit test.
add_executable(ut_buzzer
        dev/interface/buzzer_interface.cpp
        dev/scheduler/buzzer_scheduler.cpp
        dev/debug/Unit_Tests/ut_buzzer.cpp)

# CAN Interface: include CANInterface
add_executable(ut_can
        dev/interface/can_interface.cpp
        dev/interface/buzzer_interface.cpp
        dev/debug/shell/shell.cpp
        dev/interface/led.cpp
        dev/debug/Unit_Tests/ut_can_interface.cpp)

# Elevator Interface: include CANInterface, ElevatorInterface, and the unit test modules.
add_executable(ut_elevator_interface
        dev/interface/can_interface.cpp
        dev/debug/Unit_Tests/ut_elevator_IF.cpp)

# Engineer Gimbal Unit tests.
add_executable(ut_engineer_gimbal
        dev/interface/engineer_gimbal.cpp
        dev/interface/remote_interpreter.cpp
        dev/debug/Unit_Tests/ut_engineer_gimbal.cpp)

# Gimbal Interface
add_executable(ut_gimbal_interface
        dev/interface/gimbal_interface.cpp
        dev/interface/can_interface.cpp
        dev/scheduler/buzzer_scheduler.cpp
        dev/debug/Unit_Tests/ut_gimbalIF.cpp)

# Robotic Arm
add_executable(ut_robotic_arm
        dev/debug/Unit_Tests/ut_engineer_robotic_arm.cpp
        dev/interface/can_interface.cpp
        dev/interface/robotic_arm_interface.cpp
        dev/scheduler/robotic_arm_skd.cpp)

# Auto Robotic Arm
add_executable(ut_auto_robotic_arm
        dev/scheduler/new_robotic_arm_skd.cpp)

# Referee
add_executable(ut_referee_interface
        dev/common/CRC8.cpp
        dev/common/CRC16.cpp
        dev/interface/referee_interface.cpp
        dev/debug/Unit_Tests/ut_refereeIF.cpp)

add_executable(ut_sd_card
        dev/interface/sd_card_interface.cpp
        dev/common/CRC16.cpp
        dev/debug/Unit_Tests/ut_sd_card.cpp)

add_executable(ut_super_capacitor
        dev/interface/super_capacitor_port.cpp
        dev/interface/can_interface.cpp
        dev/debug/shell/shell.cpp
        dev/debug/Unit_Tests/ut_super_capacitance.cpp
        dev/scheduler/buzzer_scheduler.cpp
        dev/interface/led.cpp
        dev/interface/referee_interface.cpp
        dev/common/CRC8.cpp
        dev/common/CRC16.cpp)

add_executable(ut_adc
        dev/debug/Unit_Tests/ut_adc.cpp
        dev/interface/dms_interface.cpp)

add_executable(ut_sensors_sound
        dev/debug/Unit_Tests/ut_sensors_sound.cpp
        )

# --------------------------------------- Param Adjusts ---------------------------------------

add_executable(pa_hero_gimbal
        ${VEHICLE_COMMON_SRC}
        ${AHRS_SRC}

        # Hero gimbal
        dev/interface/gimbal_interface.cpp
        dev/scheduler/gimbal_scheduler.cpp

        # PAUser
        dev/debug/param_adjusts/pa_user_gimbal.cpp
        )
target_include_directories(pa_hero_gimbal PRIVATE dev/vehicle/hero)
target_compile_definitions(pa_hero_gimbal PRIVATE
        PARAM_ADJUST
        PARAM_ADJUST_INCLUDE="debug/param_adjusts/pa_user_gimbal.h"
        PARAM_ADJUST_CLASS=PAUserGimbal
        HERO
        HERO_GIMBAL_ENABLE=1
        HERO_SHOOT_ENABLE=0
        HERO_CHASSIS_ENABLE=0
        HERO_VISION_ENABLE=0)
target_link_libraries(pa_hero_gimbal ${AHRS_LIB})

# Hero Shoot
add_executable(pa_hero_shoot
        dev/interface/can_interface.cpp
        dev/interface/gimbal_interface.cpp
        dev/debug/param_adjusts/pa_hero_shoot.cpp)
target_compile_definitions(pa_hero_shoot PUBLIC HERO)

# Chassis
add_executable(pa_chassis
        dev/interface/can_interface.cpp
        dev/interface/chassis_interface.cpp
        dev/scheduler/chassis_scheduler.cpp
        dev/interface/gimbal_interface.cpp
        dev/scheduler/gimbal_scheduler.cpp
        dev/debug/param_adjusts/pa_chassis.cpp)
target_compile_definitions(pa_chassis PUBLIC INFANTRY)

# Shoot: include CANInterface, GimbalIF, GimbalController, GimbalFeedbackThread and adjustment modules.
add_executable(pa_infantry_shoot
        dev/common/CRC16.cpp
        dev/interface/sd_card_interface.cpp
        dev/interface/remote_interpreter.cpp
        ${AHRS_SRC}
        dev/interface/can_interface.cpp
        dev/interface/gimbal_interface.cpp
        dev/module/pid_controller.hpp
        dev/scheduler/gimbal_scheduler.cpp
        dev/scheduler/shoot_scheduler.cpp
        dev/scheduler/buzzer_scheduler.cpp
        dev/interface/ahrs/ahrs.cpp
        dev/debug/param_adjusts/pa_infantry_shoot.cpp)
target_compile_definitions(pa_infantry_shoot PRIVATE SHELL_NO_ECHO_MODE)
target_link_libraries(pa_infantry_shoot ${AHRS_LIB})

# Gimbal: include CANInterface, GimbalInterface, GimbalController, GimbalFeedbackThread and adjustment modules.
add_executable(pa_gimbal
        dev/common/CRC16.cpp
        dev/interface/sd_card_interface.cpp
        ${AHRS_SRC}
        dev/interface/can_interface.cpp
        dev/interface/gimbal_interface.cpp
        dev/module/pid_controller.hpp
        dev/scheduler/gimbal_scheduler.cpp
        dev/scheduler/shoot_scheduler.cpp
        dev/scheduler/buzzer_scheduler.cpp
        dev/interface/remote_interpreter.cpp
        dev/interface/ahrs/ahrs.cpp
        dev/debug/param_adjusts/pa_gimbal.cpp)
target_compile_definitions(pa_gimbal PRIVATE SHELL_NO_ECHO_MODE)
target_link_libraries(pa_gimbal ${AHRS_LIB})

add_executable(pa_new_gimbal
        dev/common/CRC16.cpp
        dev/interface/sd_card_interface.cpp
        dev/interface/remote_interpreter.cpp
        ${AHRS_SRC}
        dev/interface/can_interface.cpp
        dev/interface/gimbal_interface.cpp
        dev/module/pid_controller.hpp
        dev/scheduler/gimbal_scheduler.cpp
        dev/scheduler/shoot_scheduler.cpp
        dev/scheduler/buzzer_scheduler.cpp
        dev/interface/ahrs/ahrs.cpp
        dev/debug/param_adjusts/pa_new_gimbal.cpp)
target_link_libraries(pa_new_gimbal ${AHRS_LIB})

# Elevator: include CANInterface, ElevatorInterface, ElevatorController, and the unit test modules.
add_executable(pa_elevator
        dev/interface/can_interface.cpp
        dev/module/pid_controller.hpp
        dev/logic/engineer_elevator_logic.cpp
        dev/scheduler/engineer_elevator_skd.cpp
        dev/interface/engineer_elevator_interface.cpp
        dev/debug/param_adjusts/pa_engineer_elevator.cpp)

add_executable(pa_engineer_chassis
        dev/interface/can_interface.cpp
        dev/module/pid_controller.hpp
        dev/scheduler/engineer_chassis_skd.cpp
        dev/debug/param_adjusts/pa_engineer_chassis.cpp)

# Adjust Params of Skywalker-20A for friction wheels
add_executable(pa_skywalker
        dev/debug/param_adjusts/pa_skywalker.cpp
        dev/interface/remote_interpreter.cpp
        dev/scheduler/buzzer_scheduler.cpp)


# Adjust Params of Sentry Chassis
add_executable(pa_sentry_chassis
        dev/interface/can_interface.cpp
        dev/module/pid_controller.hpp
        dev/interface/sentry_chassis_interface.cpp
        dev/scheduler/sentry_chassis_scheduler.cpp
        dev/debug/param_adjusts/pa_sentry_chassis.cpp
        dev/interface/referee_interface.cpp
        dev/common/CRC8.cpp
        dev/common/CRC16.cpp)

add_executable(ca_ahrs_infantry
        ${AHRS_SRC}
        dev/common/CRC16.cpp
        dev/interface/sd_card_interface.cpp
        dev/debug/Unit_Tests/ca_ahrs.cpp)
target_link_libraries(ca_ahrs_infantry ${AHRS_LIB})
target_include_directories(ca_ahrs_infantry PRIVATE dev/vehicle/infantry)
target_compile_definitions(ca_ahrs_infantry PRIVATE INFANTRY)

add_executable(ca_ahrs_hero
        ${AHRS_SRC}
        dev/common/CRC16.cpp
        dev/interface/sd_card_interface.cpp
        dev/debug/Unit_Tests/ca_ahrs.cpp)
target_link_libraries(ca_ahrs_hero ${AHRS_LIB})
target_include_directories(ca_ahrs_hero PRIVATE dev/vehicle/hero)
target_compile_definitions(ca_ahrs_hero PRIVATE HERO)

# ================================= Post Configurations for all targets =================================

# Make all targets compile to this filename to allow upload script to work properly
set(ELF_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build)
set(ELF_OUTPUT_NAME meta.elf)
message("[Notice] All executables are built to ${ELF_OUTPUT_PATH}/${ELF_OUTPUT_NAME}")

add_definitions(-DBUILD_TARGET_NAME="$<TARGET_PROPERTY:NAME>")

get_directory_property(TARGET_LIST BUILDSYSTEM_TARGETS)
foreach (target ${TARGET_LIST})
    if (NOT ${target} STREQUAL "ChibiOS" AND NOT ${target} STREQUAL "CMSIS_DSP")  # OBJECT library doesn't have PRE_BUILD or POST_BUILD
        # Add ChibiOS files and common sources
        target_sources(${target} PRIVATE $<TARGET_OBJECTS:ChibiOS> ${BASE_SRC})

        # Link CMSIS DSP
        target_link_libraries(${target} CMSIS_DSP)

        # Echo target dev board before link
        add_custom_command(TARGET ${target} PRE_BUILD
                COMMENT "Build for ${BOARD_NAME}")

        # Run size utility after link to show the size of elf file
        add_custom_command(TARGET ${target} POST_BUILD
                COMMAND echo
                COMMAND ${CMAKE_SIZE_UTILITY} ${ELF_OUTPUT_PATH}/${ELF_OUTPUT_NAME}
                COMMAND echo)  # execute size utility to show size of executable file

        # Set unique output file
        set_target_properties(${target} PROPERTIES
                RUNTIME_OUTPUT_DIRECTORY ${ELF_OUTPUT_PATH}
                OUTPUT_NAME ${ELF_OUTPUT_NAME})  #
    endif ()
endforeach (target)


