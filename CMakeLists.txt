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
        dev/interface/can
        dev/interface/virtual_COM
        dev/module
        dev/scheduler
        dev/logic
        dev/vehicle)

set(BASE_SRC  # common cpp sources, which are included to every target at post configuration below
        dev/interface/usb_serial_interface.cpp
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
        dev/interface/can/can_interface.cpp
        dev/interface/can/can_motor_feedback.cpp
        dev/interface/can/can_motor_interface.cpp
        dev/interface/can/can_motor_scheduler.cpp
        dev/interface/remote_interpreter.cpp
        dev/interface/oled_interface.cpp
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
        dev/scheduler/mecanum_chassis_scheduler.cpp
        dev/logic/chassis_logic.cpp)

set(VISION_SRC
        dev/interface/vision_interface.cpp
        dev/scheduler/vision_scheduler.cpp
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
        dev/scheduler/gimbal_scheduler.cpp
        dev/logic/gimbal_logic.cpp

        # Infantry shoot
        dev/scheduler/shoot_scheduler.cpp
        dev/logic/shoot_logic.cpp

        # Infantry has super capacitor
        dev/interface/capacitor_interface.cpp

        # Main
        dev/vehicle/infantry/CANBUS_MOTOR_CFG.cpp
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
        dev/scheduler/gimbal_scheduler.cpp
        dev/logic/gimbal_logic.cpp

        # Hero shoot
        dev/scheduler/shoot_scheduler.cpp
        dev/logic/shoot_logic.cpp

        # Hero has super capacitor
        dev/interface/capacitor_interface.cpp
        dev/interface/lidar_interface.cpp

        # Main
        dev/vehicle/hero/CANBUS_MOTOR_CFG.cpp
        dev/vehicle/hero/inspector_hero.cpp
        dev/vehicle/hero/user_hero.cpp
        dev/vehicle/hero/main_hero.cpp)
target_include_directories(HERO PRIVATE dev/vehicle/hero)
target_compile_definitions(HERO PRIVATE HERO)
target_link_libraries(HERO ${AHRS_LIB})

add_executable(HAPTIC_DVC
        ${VEHICLE_COMMON_SRC}
        ${AHRS_SRC}
        dev/vehicle/haptic_device/Communicator.cpp
        dev/vehicle/haptic_device/CANBUS_MOTOR_CFG.cpp
        dev/vehicle/haptic_device/Inspector.cpp
        dev/module/controller_module.cpp
        dev/logic/haptic_logic.cpp
        dev/module/low_pass_filter.hpp
        dev/vehicle/haptic_device/main_haptic.cpp)
target_include_directories(HAPTIC_DVC PRIVATE dev/vehicle/haptic_device)
target_compile_definitions(HAPTIC_DVC PRIVATE HAPTIC_DVC)
target_link_libraries(HAPTIC_DVC ${AHRS_LIB})

# Main Program -- Sentry

# Main Program -- Engineer


# ======================================= Unit Tests Programs =======================================

# Includes Unit Tests and Param Adjust programs

# --------------------------------------- Unit Tests ---------------------------------------

include_directories(dev/debug/Unit_Tests/)
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
        dev/interface/can/can_interface.cpp
        dev/interface/ahrs/ahrs_ext.cpp
        dev/debug/Unit_Tests/ut_ahrs_ext.cpp)

# BuzzerSKD: inlcude BuzzerSKD interface and unit test.
add_executable(ut_buzzer
        dev/interface/buzzer_interface.cpp
        dev/scheduler/buzzer_scheduler.cpp
        dev/debug/Unit_Tests/ut_buzzer.cpp)

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
        dev/interface/capacitor_interface.cpp
        dev/interface/can/can_interface.cpp
        dev/debug/shell/shell.cpp
        dev/scheduler/buzzer_scheduler.cpp
        dev/interface/led.cpp
        dev/interface/referee_interface.cpp
        dev/common/CRC8.cpp
        dev/common/CRC16.cpp)

add_executable(ut_adc
        dev/debug/Unit_Tests/ut_adc.cpp
        dev/interface/dms_interface.cpp)

add_executable(ut_oled
        dev/debug/Unit_Tests/ut_oled.cpp
        dev/interface/oled_interface.cpp)

add_executable(ut_sensors_sound
        dev/debug/Unit_Tests/ut_sensors_sound.cpp
        )

add_executable(ut_chassis
        ${VEHICLE_COMMON_SRC}
        ${AHRS_SRC}
        dev/debug/Unit_Tests/ut_chassis/CANBUS_MOTOR_CFG.cpp
        dev/interface/can/can_motor_scheduler.cpp
        dev/scheduler/mecanum_chassis_scheduler.cpp
        dev/logic/chassis_logic.cpp
        dev/module/low_pass_filter.hpp
        dev/debug/Unit_Tests/ut_chassis/ut_chassis.cpp
        )
target_include_directories(ut_chassis PRIVATE dev/debug/Unit_Tests/ut_chassis)
target_compile_definitions(ut_chassis PRIVATE ut_chassis)
target_link_libraries(ut_chassis ${AHRS_LIB})

# --------------------------------------- Param Adjusts ---------------------------------------

add_executable(pa_hero_gimbal
        ${VEHICLE_COMMON_SRC}
        ${AHRS_SRC}

        # Hero gimbal
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
        dev/interface/can/can_interface.cpp
        dev/debug/param_adjusts/pa_hero_shoot.cpp)
target_compile_definitions(pa_hero_shoot PUBLIC HERO)

# Chassis
add_executable(pa_chassis
        dev/interface/can/can_interface.cpp
        dev/scheduler/mecanum_chassis_scheduler.cpp
        dev/scheduler/gimbal_scheduler.cpp
        dev/debug/param_adjusts/pa_chassis.cpp)
target_compile_definitions(pa_chassis PUBLIC INFANTRY)

# Shoot: include CANInterface, GimbalIF, GimbalController, GimbalFeedbackThread and adjustment modules.
add_executable(pa_infantry_shoot
        dev/common/CRC16.cpp
        dev/interface/sd_card_interface.cpp
        dev/interface/remote_interpreter.cpp
        ${AHRS_SRC}
        dev/interface/can/can_interface.cpp
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
        dev/interface/can/can_interface.cpp
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
        dev/interface/can/can_interface.cpp
        dev/module/pid_controller.hpp
        dev/scheduler/gimbal_scheduler.cpp
        dev/scheduler/shoot_scheduler.cpp
        dev/scheduler/buzzer_scheduler.cpp
        dev/interface/ahrs/ahrs.cpp
        dev/debug/param_adjusts/pa_new_gimbal.cpp)
target_link_libraries(pa_new_gimbal ${AHRS_LIB})

# Elevator: include CANInterface, ElevatorInterface, ElevatorController, and the unit test modules.
add_executable(pa_elevator
        dev/interface/can/can_interface.cpp
        dev/module/pid_controller.hpp
        dev/debug/param_adjusts/pa_engineer_elevator.cpp)

add_executable(pa_engineer_chassis
        dev/interface/can/can_interface.cpp
        dev/module/pid_controller.hpp
        dev/debug/param_adjusts/pa_engineer_chassis.cpp)

# Adjust Params of Skywalker-20A for friction wheels
add_executable(pa_skywalker
        dev/debug/param_adjusts/pa_skywalker.cpp
        dev/interface/remote_interpreter.cpp
        dev/scheduler/buzzer_scheduler.cpp)

# Adjust Params of Sentry Chassis
add_executable(pa_sentry_chassis
        dev/interface/can/can_interface.cpp
        dev/module/pid_controller.hpp
        dev/scheduler/sentry_chassis_scheduler.cpp
        dev/debug/param_adjusts/pa_sentry_chassis.cpp
        dev/interface/referee_interface.cpp
        dev/common/CRC8.cpp
        dev/common/CRC16.cpp)

add_executable(ca_ahrs_infantry
        dev/interface/ahrs/ahrs_math.hpp
        dev/interface/ahrs/imu_on_board.cpp
        dev/interface/ahrs/ahrs_lib_middleware.c
        dev/interface/ahrs/ahrs.cpp
        dev/common/CRC16.cpp
        dev/interface/sd_card_interface.cpp
        dev/debug/Unit_Tests/ca_ahrs.cpp)
target_link_libraries(ca_ahrs_infantry ${AHRS_LIB})
target_include_directories(ca_ahrs_infantry PRIVATE dev/vehicle/infantry)
target_compile_definitions(ca_ahrs_infantry PRIVATE INFANTRY)

add_executable(ca_ahrs_hero
        dev/interface/ahrs/ahrs_math.hpp
        dev/interface/ahrs/imu_on_board.cpp
        dev/interface/ahrs/ahrs_lib_middleware.c
        dev/interface/ahrs/ahrs.cpp
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


