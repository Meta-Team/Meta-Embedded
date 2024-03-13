# Created by liuzikai 2018-01-29

# =========== General Part ===========

cmake_minimum_required(VERSION 3.10)
set (BUILD_HERO TRUE)
set (BUILD_SENTRY TRUE)
set (BUILD_INFANTRY3 TRUE)
set (BUILD_INFANTRY4 TRUE)


include(config/toolchain.cmake)  # include cross compile configurations

set(CMAKE_CXX_STANDARD 14)

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

project(Meta_Embedded ASM C CXX)
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
        dev/interface/shell
        dev/interface
        dev/interface/ahrs
        dev/interface/buzzer
        dev/interface/can
        dev/interface/can_motor
        dev/interface/capacitor
        dev/interface/led
        dev/interface/lidar
        dev/interface/oled
        dev/interface/referee
        dev/interface/remote
        dev/interface/sd_card
        dev/interface/usb_com
        dev/interface/vision
        dev/module
        dev/scheduler
        dev/logic
        dev/application)

set(BASE_SRC  # common cpp sources, which are included to every target at post configuration below
        dev/interface/usb_com/usb_serial_interface.cpp
        dev/interface/shell/shell.cpp
        dev/interface/shell/shell_base.c
        dev/interface/shell/shell_base_cmd.c
        dev/interface/shell/shell_dbg_cmd.cpp
        dev/interface/shell/printf.c
        dev/interface/led/led.cpp
        dev/interface/buzzer/buzzer_interface.cpp
        dev/interface/buzzer/buzzer_scheduler.cpp)

set(CAN_SRC
        dev/module/pid_controller.hpp
        dev/interface/can/can_interface.cpp
        dev/interface/can_motor/can_motor_feedback.cpp
        dev/interface/can_motor/can_motor_interface.cpp
        dev/interface/can_motor/can_motor_controller.cpp
        )

set(VEHICLE_COMMON_SRC
        dev/module/pid_controller.hpp
        dev/interface/can/can_interface.cpp
        dev/interface/can_motor/can_motor_feedback.cpp
        dev/interface/can_motor/can_motor_interface.cpp
        dev/interface/can_motor/can_motor_controller.cpp
        dev/interface/remote/remote_interpreter.cpp
        dev/interface/oled/oled_interface.cpp
        dev/common/CRC8.cpp
        dev/common/CRC16.cpp
        dev/interface/sd_card/sd_card_interface.cpp)

set(AHRS_SRC
        dev/interface/ahrs/ahrs_math.hpp
        dev/interface/ahrs/ahrs.cpp
        dev/interface/ahrs/ahrs_lib_middleware.c
        dev/interface/ahrs/imu_on_board.cpp)

set(AHRS_EXT_SRC
        dev/interface/ahrs/ahrs_math.hpp
        dev/interface/ahrs/ahrs_ext.cpp
        dev/interface/ahrs/ahrs_lib_middleware.c
        dev/interface/ahrs/imu_on_board.cpp)

set(AHRS_LIB
        ${PROJECT_SOURCE_DIR}/dev/interface/ahrs/ahrs_lib.lib)

set(CHASSIS_SRC
        dev/scheduler/mecanum_chassis_scheduler.cpp
        dev/logic/chassis_logic.cpp)

set(VISION_SRC
        dev/interface/vision/vision_interface.cpp
        dev/scheduler/vision_scheduler.cpp
        )

set(REFEREE_SRC
        dev/interface/referee/referee_interface.cpp
        dev/scheduler/referee_UI_update_scheduler.cpp
        dev/logic/referee_UI_logic.cpp)

include_directories(dev/board_pin/${BOARD_NAME})  # include board configs

# Reuse ChibiOS object files (without linking) to avoid repeat compile
# They are included to every target at post configuration below
add_library(ChibiOS OBJECT ${CHIBIOS_XASM_SRC} ${CHIBIOS_C_SRC} ${CHIBIOS_CPP_SRC} dev/board_pin/${BOARD_NAME}/board.c)


# ========================== Files and Targets Configurations ==========================

# NOTICE: UPDATE README IF THE CONFIGURATIONS IS UPDATED

# ========================== Robot's Control Programs ===========================
if(BUILD_INFANTRY3 OR BUILD_INFANTRY4)
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
            dev/interface/capacitor/capacitor_interface.cpp

            # Main
            dev/application/vehicles/infantry/can_motor_config.cpp
            dev/application/vehicles/infantry/inspector_infantry.cpp
            dev/application/vehicles/infantry/user_infantry.cpp
            dev/application/vehicles/infantry/main_infantry.cpp)

    set(INFANTRY_DEFS
            INFANTRY
            INFANTRY_GIMBAL_ENABLE=1
            INFANTRY_CHASSIS_ENABLE=1
            INFANTRY_SUPER_CAPACITOR_ENABLE=1
            INFANTRY_VISION_ENABLE=1)

    set(INFANTRY_LIBS
            ${AHRS_LIB})
endif()
# Main Program -- Infantry #3
if(BUILD_INFANTRY3)
    add_executable(INFANTRY_THREE ${INFANTRY_SRC})
    target_include_directories(INFANTRY_THREE PRIVATE dev/application/vehicles/infantry)
    target_compile_definitions(INFANTRY_THREE PRIVATE ${INFANTRY_DEFS} INFANTRY_THREE)
    target_link_libraries(INFANTRY_THREE ${INFANTRY_LIBS})
endif()
## Main Program -- Infantry #4
if(BUILD_INFANTRY4)
    add_executable(INFANTRY_FOUR ${INFANTRY_SRC})
    target_include_directories(INFANTRY_FOUR PRIVATE dev/application/vehicles/infantry)
    target_compile_definitions(INFANTRY_FOUR PRIVATE ${INFANTRY_DEFS} INFANTRY_FOUR)
    target_link_libraries(INFANTRY_FOUR ${INFANTRY_LIBS})
endif()
# Main Program -- Hero
if(BUILD_HERO)
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
            dev/interface/capacitor/capacitor_interface.cpp
            dev/interface/lidar/lidar_interface.cpp

            # Main
            dev/application/vehicles/hero/can_motor_config.cpp
            dev/application/vehicles/hero/inspector_hero.cpp
            dev/application/vehicles/hero/user_hero.cpp
            dev/application/vehicles/hero/main_hero.cpp)
    target_include_directories(HERO PRIVATE dev/application/vehicles/hero)
    target_compile_definitions(HERO PRIVATE HERO)
    target_link_libraries(HERO ${AHRS_LIB})
endif()
#add_executable(HAPTIC_DVC
#        ${VEHICLE_COMMON_SRC}
#        ${AHRS_SRC}
#        dev/application/vehicles/haptic_device/Communicator.cpp
#        dev/application/vehicles/haptic_device/can_motor_config.cpp
#        dev/application/vehicles/haptic_device/Inspector.cpp
#        dev/logic/haptic_logic.cpp
#        dev/module/low_pass_filter.hpp
#        dev/application/vehicles/haptic_device/main_haptic.cpp)
#target_include_directories(HAPTIC_DVC PRIVATE dev/application/vehicles/haptic_device)
#target_compile_definitions(HAPTIC_DVC PRIVATE HAPTIC_DVC)
#target_link_libraries(HAPTIC_DVC ${AHRS_LIB})
if(BUILD_SENTRY)
    # Main Program -- Sentry
    add_executable(SENTRY
            ${VEHICLE_COMMON_SRC}
            #        ${AHRS_EXT_SRC}

            dev/interface/referee/referee_interface.cpp

            dev/scheduler/sentry_chassis_scheduler.cpp
            dev/scheduler/gimbal_scheduler.cpp
            dev/scheduler/shoot_scheduler.cpp

            dev/logic/sentry_chassis_logic.cpp
            dev/logic/gimbal_logic.cpp
            dev/logic/shoot_logic.cpp

            dev/application/vehicles/sentry/inspector_sentry.cpp
            dev/application/vehicles/sentry/can_motor_config.cpp
            dev/application/vehicles/sentry/user_sentry.cpp
            dev/application/vehicles/sentry/main_sentry.cpp
            )
    target_include_directories(SENTRY PRIVATE dev/application/vehicles/sentry)
    target_compile_definitions(SENTRY PRIVATE SENTRY)
    target_link_libraries(SENTRY ${AHRS_LIB})
    file(GLOB AUTO_SENTRY_SRC dev/application/vehicles/auto_sentry/*.cpp)
    add_executable(AUTO_SENTRY
            ${VEHICLE_COMMON_SRC}
            ${AUTO_SENTRY_SRC}
            dev/interface/can_motor/can_motor_controller.cpp
            dev/scheduler/mecanum_chassis_scheduler.cpp
            dev/logic/chassis_logic.cpp
            dev/module/low_pass_filter.hpp
            )
    target_include_directories(AUTO_SENTRY PRIVATE dev/application/vehicles/auto_sentry)
    target_compile_definitions(AUTO_SENTRY PRIVATE AUTO_SENTRY)
    target_link_libraries(AUTO_SENTRY ${AHRS_LIB})
    # TODO this defines the ut_chassis macro to use mecanum chassis, later we need to switch to omnichassis(WIP)
    target_compile_definitions(AUTO_SENTRY PRIVATE ut_chassis)
endif()
# Main Program -- Engineer

# Main Program -- Dart
add_executable(DART
        ${VEHICLE_COMMON_SRC}
        dev/interface/referee/referee_interface.cpp
        dev/interface/rudder/rudder_interface.cpp
        dev/application/vehicles/dart/can_motor_config.cpp
        dev/application/vehicles/dart/main_dart.cpp
        dev/application/vehicles/dart/user_dart.cpp

        dev/interface/damiao_motor/damiao_motor_feedback.cpp
        dev/interface/damiao_motor/damiao_motor_interface.cpp
        dev/interface/damiao_motor/damiao_motor_controller.cpp
        )
target_include_directories(DART PRIVATE dev/application/vehicles/dart
        dev/interface/damiao_motor)
target_compile_definitions(DART PRIVATE DART)

# ======================================= Unit Tests Programs =======================================

# Includes Unit Tests and Param Adjust programs

# --------------------------------------- Unit Tests ---------------------------------------

include_directories(dev/application/unit_tests/)
# Blink
add_executable(ut_blink
        dev/interface/button/button_monitor.cpp
        dev/application/unit_tests/ut_led/ut_led.cpp)
target_include_directories(ut_blink
        PRIVATE
        dev/interface/button
        dev/application/unit_tests/ut_led)

# Remote Interpreter: include remote_interpreter sources and the unit test modules.
add_executable(ut_remote_interpreter
        dev/interface/remote/remote_interpreter.cpp
        dev/application/unit_tests/ut_remote_if/ut_remoteIF.cpp)
target_include_directories(ut_remote_interpreter
        PRIVATE
        dev/application/unit_tests/ut_remote_if)

# AHRS
add_executable(ut_ahrs
        dev/interface/ahrs/ahrs_math.hpp
        dev/interface/ahrs/imu_on_board.cpp
        dev/interface/ahrs/ahrs_lib_middleware.c
        dev/interface/ahrs/ahrs.cpp
        dev/application/unit_tests/ut_ahrs/ut_ahrs.cpp)
target_include_directories(ut_ahrs PRIVATE dev/application/unit_tests/ut_ahrs)
target_link_libraries(ut_ahrs ${AHRS_LIB})

# AHRSExt
add_executable(ut_ahrs_ext
        dev/interface/can/can_interface.cpp
        dev/interface/ahrs/ahrs_ext.cpp
        dev/application/unit_tests/ut_ahrs/ut_ahrs_ext.cpp)
target_include_directories(ut_ahrs_ext PRIVATE dev/application/unit_tests/ut_ahrs)

# BuzzerSKD: inlcude BuzzerSKD interface and unit test.
add_executable(ut_buzzer
        dev/interface/buzzer/buzzer_interface.cpp
        dev/interface/buzzer/buzzer_scheduler.cpp
        dev/application/unit_tests/ut_buzzer/ut_buzzer.cpp)
target_include_directories(ut_buzzer PRIVATE dev/application/unit_tests/ut_buzzer)

# Referee
# temporarily disabled by Tony Zhang
# TODO re-enable this one, since referee is outdated
add_executable(ut_referee_interface
        dev/common/CRC8.cpp
        dev/common/CRC16.cpp
        dev/interface/referee/referee_interface.cpp
        dev/application/unit_tests/ut_referee_if/ut_referee_if.cpp)
target_include_directories(ut_referee_interface PRIVATE dev/application/unit_tests/ut_referee_if)

add_executable(ut_sd_card
        dev/interface/sd_card/sd_card_interface.cpp
        dev/common/CRC16.cpp
        dev/application/unit_tests/ut_sd_card/ut_sd_card.cpp)
target_include_directories(ut_sd_card PRIVATE dev/application/unit_tests/ut_sd_card)

add_executable(ut_oled
        dev/application/unit_tests/ut_oled/ut_oled.cpp
        dev/interface/oled/oled_interface.cpp)
target_include_directories(ut_oled PRIVATE dev/application/unit_tests/ut_oled)

add_executable(ut_rudder
        dev/application/unit_tests/ut_rudder/main.cpp
        dev/interface/rudder/rudder_interface.cpp)
target_include_directories(ut_rudder PRIVATE dev/interface/rudder
        dev/application/unit_tests/ut_rudder)

# Added by Tony Zhang
# Note, this Unit test is deprecated since advance of new sentry
#
#add_executable(ut_sentry_chassis
#        ${VEHICLE_COMMON_SRC}
#        dev/interface/buzzer/buzzer_interface.cpp
#        dev/interface/buzzer/buzzer_scheduler.cpp
#        dev/application/unit_tests/ut_sentry_chassis/can_motor_config.cpp
#        dev/application/unit_tests/ut_sentry_chassis/ut_sentry_chassis.cpp)
#target_include_directories(ut_sentry_chassis PRIVATE dev/application/unit_tests/ut_sentry_chassis)

add_executable(ut_chassis
        ${VEHICLE_COMMON_SRC}
        ${AHRS_SRC}
        dev/application/unit_tests/ut_chassis/can_motor_config.cpp
        dev/interface/can_motor/can_motor_controller.cpp
        dev/scheduler/mecanum_chassis_scheduler.cpp
        dev/logic/chassis_logic.cpp
        dev/module/low_pass_filter.hpp
        dev/application/unit_tests/ut_chassis/ut_chassis.cpp)
target_include_directories(ut_chassis PRIVATE dev/application/unit_tests/ut_chassis)
target_compile_definitions(ut_chassis PRIVATE ut_chassis)
target_link_libraries(ut_chassis ${AHRS_LIB})

add_executable(ut_steer_chassis
        ${VEHICLE_COMMON_SRC}
        ${AHRS_SRC}
        dev/application/unit_tests/ut_steer_chassis/can_motor_config.cpp
        dev/interface/can_motor/can_motor_controller.cpp
        dev/scheduler/steering_chassis_scheduler.cpp
        dev/logic/chassis_logic.cpp
        dev/module/low_pass_filter.hpp
        dev/application/unit_tests/ut_steer_chassis/ut_chassis.cpp)
target_include_directories(ut_steer_chassis PRIVATE dev/application/unit_tests/ut_steer_chassis)
target_compile_definitions(ut_steer_chassis PRIVATE ut_steer_chassis)
target_link_libraries(ut_steer_chassis ${AHRS_LIB})

add_executable(ut_damiao_motor
        dev/application/unit_tests/ut_damiao_motor/main.cpp
        dev/interface/damiao_motor/damiao_motor_interface.cpp
        dev/interface/damiao_motor/damiao_motor_controller.cpp
        dev/interface/damiao_motor/damiao_motor_feedback.cpp
        dev/interface/can/can_interface.cpp)
target_include_directories(ut_damiao_motor PRIVATE dev/application/unit_tests/ut_damiao_motor
        dev/interface/damiao_motor
        dev/interface/can)
target_compile_definitions(ut_damiao_motor PRIVATE ut_damiao_motor)

# --------------------------------------- Param Adjusts ---------------------------------------


# Gimbal: include CANInterface, GimbalInterface, GimbalController, GimbalFeedbackThread and adjustment modules.

set(pa_infantry_dir dev/application/param_adjusts/pa_infantry)
add_executable(pa_infantry
        ${COMMON_SRC}
        ${AHRS_SRC}
        ${CAN_SRC}
        dev/interface/remote/remote_interpreter.cpp
        ${pa_infantry_dir}/can_motor_config.cpp
        ${pa_infantry_dir}/pa_infantry.cpp)
target_link_libraries(pa_infantry ${AHRS_LIB})
target_include_directories(pa_infantry PRIVATE ${pa_infantry_dir})

set(pa_hero_dir dev/application/param_adjusts/pa_hero)
add_executable(pa_hero
        ${COMMON_SRC}
        ${AHRS_SRC}
        ${CAN_SRC}
        dev/interface/remote/remote_interpreter.cpp
        ${pa_hero_dir}/can_motor_config.cpp
        ${pa_hero_dir}/pa_hero.cpp)
target_link_libraries(pa_hero ${AHRS_LIB})
target_include_directories(pa_hero PRIVATE ${pa_hero_dir})

add_executable(ca_ahrs_infantry
        dev/interface/ahrs/ahrs_math.hpp
        dev/interface/ahrs/imu_on_board.cpp
        dev/interface/ahrs/ahrs_lib_middleware.c
        dev/interface/ahrs/ahrs.cpp
        dev/common/CRC16.cpp
        dev/interface/sd_card/sd_card_interface.cpp
        dev/application/unit_tests/ut_ahrs/ca_ahrs.cpp)
target_link_libraries(ca_ahrs_infantry ${AHRS_LIB})
target_include_directories(ca_ahrs_infantry PRIVATE dev/application/vehicles/infantry)
target_compile_definitions(ca_ahrs_infantry PRIVATE INFANTRY)

add_executable(ca_ahrs_hero
        dev/interface/ahrs/ahrs_math.hpp
        dev/interface/ahrs/imu_on_board.cpp
        dev/interface/ahrs/ahrs_lib_middleware.c
        dev/interface/ahrs/ahrs.cpp
        dev/common/CRC16.cpp
        dev/interface/sd_card/sd_card_interface.cpp
        dev/application/unit_tests/ut_ahrs/ca_ahrs.cpp)
target_link_libraries(ca_ahrs_hero ${AHRS_LIB})
target_include_directories(ca_ahrs_hero PRIVATE dev/application/vehicles/hero)
target_compile_definitions(ca_ahrs_hero PRIVATE HERO)

# ================================= Post Configurations for all targets =================================

# Make all targets compile to this filename to allow upload script to work properly
set(ELF_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build)

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
                COMMAND ${CMAKE_SIZE_UTILITY} ${ELF_OUTPUT_PATH}/${target}.elf
                )  # execute size utility to show size of executable file

        # Set unique output file
        set_target_properties(${target} PROPERTIES
                RUNTIME_OUTPUT_DIRECTORY ${ELF_OUTPUT_PATH}
                OUTPUT_NAME ${target}.elf)  #
    endif ()
endforeach (target)
