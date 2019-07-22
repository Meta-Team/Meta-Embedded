# Created by liuzikai 2018-01-29

# =========== General Part ===========

cmake_minimum_required(VERSION 3.10)

include(config/toolchain.cmake)  # include cross compile configurations

set(CMAKE_CXX_STANDARD 11)

# Check dev board defintion 
# NOTICE: ${BOARD_NAME} needs to be specific externally
# NOTICE: if you want to adapt this project to a new board, be sure to also set startup file in compile_options.cmake
if (NOT DEFINED BOARD_NAME)
    set(BOARD_NAME rm_board_2018_a)
    #    message(FATAL_ERROR "[ERROR] No specific dev board. Do you forget to add -DBOARD_NAMD=\"***\" to CMake options?")
endif ()

if ((${BOARD_NAME} STREQUAL rm_board_2017) OR (${BOARD_NAME} STREQUAL rm_board_2018_a))
    message("[Notice] Current profile is configurated for dev board \"${BOARD_NAME}\".")
else ()
    message(FATAL_ERROR "[ERROR] Dev board \"${BOARD_NAME}\" is not configuared.")
endif ()

project(Meta_Infantry ASM C CXX)
# NOTICE: project() must be in CMakeList.txt, rather than any cmake file. It must be after toolchain configs and before
#         flag configs for CMake to test compiler.

include(config/compile_options.cmake)  # include compile flags and startup file option


# =========== Options ===========

set(CMAKE_VERBOSE_MAKEFILE OFF)  # Enable this if you want to see verbose log


# =========== Common Files and Targets Configurations ===========

include(os/ch.cmake)  # add files and directories from ChibiOS

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
        # TODO: elimiate control directory someday
        dev/control)

set(COMMON_SRC  # common cpp sources, which are included to every tartget at post configuration below
        dev/debug/shell/shell_base.c
        dev/debug/shell/shell_base_cmd.c
        dev/debug/shell/printf.c
        dev/debug/shell/shell.cpp
        dev/debug/shell/shell_dbg_cmd.cpp
        dev/interface/led.cpp
        dev/interface/buzzer.cpp
#        dev/module/state_handler.cpp
        )

set(AHRS_SRC
        dev/interface/ahrs/ahrs_math.hpp
        dev/interface/ahrs/ahrs.cpp
        dev/interface/ahrs/ahrs_lib_middleware.c
        dev/interface/ahrs/ist8310.cpp
        dev/interface/ahrs/mpu6500.cpp)

set(INFANTRY_SHOOT_SRC
        dev/interface/gimbal_interface.cpp
        dev/scheduler/gimbal_scheduler.cpp
        dev/scheduler/shoot_scheduler.cpp
        dev/logic/gimbal_logic.cpp
        dev/logic/shoot_logic.cpp)

set(HERO_SHOOT_SRC
        dev/interface/gimbal_interface.cpp
        dev/scheduler/gimbal_scheduler.cpp
        dev/scheduler/shoot_scheduler.cpp
        dev/logic/gimbal_logic.cpp
        dev/logic/hero_shoot_logic.cpp)

set(CHASSIS_SRC
        dev/interface/chassis_interface.cpp
        dev/scheduler/chassis_scheduler.cpp
        dev/logic/chassis_logic.cpp)

include_directories(dev/board/${BOARD_NAME})  # include board configs

# Reuse ChibiOS object files (without linking) to avoid repeat compile
# They are included to every tartget at post configuration below
add_library(ChibiOS_OBJ OBJECT ${CHIBIOS_XASM_SRC} ${CHIBIOS_C_SRC} ${CHIBIOS_CPP_SRC} dev/board/${BOARD_NAME}/board.c)


# =========== Files and Targets Configurations ===========

# NOTICE: UPDATE README IF THE CONFIGURATIONS IS UPDATED

set(INFANTRY
        dev/module/pid_controller.hpp
        dev/interface/can_interface.cpp
        dev/interface/remote_interpreter.cpp
        ${AHRS_SRC}
        ${INFANTRY_SHOOT_SRC}
        ${CHASSIS_SRC}
        dev/interface/vision_port.cpp
        dev/interface/super_capacitor_port.cpp
        dev/interface/referee_interface.cpp
        dev/interface/sd_card_interface.cpp
        dev/common/CRC8.cpp
        dev/common/CRC16.cpp
        dev/vehicle/infantry/inspector_infantry.cpp
        dev/vehicle/infantry/user_infantry.cpp
        dev/vehicle/infantry/main_infantry.cpp
        dev/vehicle/infantry/settings_infantry.cpp)

# Main Program -- Infantry #3
add_executable(INFANTRY_THREE ${INFANTRY})
target_include_directories(INFANTRY_THREE PRIVATE dev/vehicle/infantry)
target_compile_definitions(INFANTRY_THREE PRIVATE INFANTRY INFANTRY_THREE)
target_link_libraries(INFANTRY_THREE ${PROJECT_SOURCE_DIR}/dev/interface/ahrs/ahrs_lib.lib)

# Main Program -- Infantry #4
add_executable(INFANTRY_FOUR ${INFANTRY})
target_include_directories(INFANTRY_FOUR PRIVATE dev/vehicle/infantry)
target_compile_definitions(INFANTRY_FOUR PRIVATE INFANTRY INFANTRY_FOUR)
target_link_libraries(INFANTRY_FOUR ${PROJECT_SOURCE_DIR}/dev/interface/ahrs/ahrs_lib.lib)

# Main Program -- Infantry #5
add_executable(INFANTRY_FIVE ${INFANTRY})
target_include_directories(INFANTRY_FIVE PRIVATE dev/vehicle/infantry)
target_compile_definitions(INFANTRY_FIVE PRIVATE INFANTRY INFANTRY_FIVE)
target_link_libraries(INFANTRY_FIVE ${PROJECT_SOURCE_DIR}/dev/interface/ahrs/ahrs_lib.lib)

# Main Program -- Hero
add_executable(HERO
        dev/module/pid_controller.hpp
        dev/interface/can_interface.cpp
        dev/interface/remote_interpreter.cpp
        ${AHRS_SRC}
        ${HERO_SHOOT_SRC}
        ${CHASSIS_SRC}
        dev/interface/vision_port.cpp
        dev/interface/super_capacitor_port.cpp
        dev/interface/referee_interface.cpp
        dev/interface/sd_card_interface.cpp
        dev/common/CRC8.cpp
        dev/common/CRC16.cpp
        dev/vehicle/hero/inspector_hero.cpp
        dev/vehicle/hero/user_hero.cpp
        dev/vehicle/hero/main_hero.cpp
        dev/vehicle/hero/settings_hero.cpp)
target_include_directories(HERO PRIVATE dev/vehicle/hero)
target_compile_definitions(HERO PRIVATE HERO)
target_link_libraries(HERO ${PROJECT_SOURCE_DIR}/dev/interface/ahrs/ahrs_lib.lib)

# Main Program -- Sentry
add_executable(SENTRY
        dev/interface/can_interface.cpp
        dev/interface/remote_interpreter.cpp
        dev/interface/referee_interface.cpp
        dev/interface/sd_card_interface.cpp
        dev/interface/vision_port.cpp
        dev/interface/ahrs/ahrs_ext.cpp
        dev/common/CRC8.cpp
        dev/common/CRC16.cpp

        dev/interface/sentry_chassis_interface.cpp
        dev/scheduler/sentry_chassis_scheduler.cpp
        dev/logic/sentry_chassis_logic.cpp

        dev/interface/gimbal_interface.cpp
        dev/scheduler/gimbal_scheduler.cpp
        dev/logic/sentry_gimbal_logic.cpp

        dev/scheduler/shoot_scheduler.cpp
        dev/logic/shoot_logic.cpp

        dev/vehicle/sentry/main_sentry.cpp
        dev/vehicle/sentry/inspector_sentry.cpp
        dev/vehicle/sentry/settings_sentry.cpp
        dev/vehicle/sentry/user_sentry.cpp

        )
target_include_directories(SENTRY PRIVATE dev/vehicle/sentry)
target_compile_definitions(SENTRY PRIVATE SENTRY)

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

#        dev/control/robotic_arm_skd.cpp
#        dev/interface/robotic_arm_interface.cpp
        )
target_include_directories(ENGINEER PRIVATE dev/vehicle/engineer)
target_compile_definitions(ENGINEER PRIVATE ENGINEER)

add_executable(ut_engineer
        dev/debug/unit_tests/engineer_unit_test.cpp
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
        dev/debug/unit_tests/ut_led.cpp)

# Remote Interpreter: include remote_interpreter sources and the unit test modules.
add_executable(ut_remote_interpreter
        dev/interface/remote_interpreter.cpp
        dev/debug/unit_tests/ut_remoteIF.cpp)

# Hero Shoot
add_executable(pa_hero_shoot
        dev/interface/can_interface.cpp
        dev/interface/gimbal_interface.cpp
        dev/debug/unit_tests/pa_hero_shoot.cpp)
target_compile_definitions(pa_hero_shoot PUBLIC HERO)

# Shoot: include CANInterface, GimbalIF, GimbalController, GimbalFeedbackThread and adjustment modules.
add_executable(pa_infantry_shoot
        dev/interface/can_interface.cpp
        dev/interface/gimbal_interface.cpp
        dev/scheduler/shoot_scheduler.cpp
        dev/debug/unit_tests/pa_infantry_shoot.cpp
        dev/interface/remote_interpreter.cpp)

# Gimbal: include CANInterface, GimbalInterface, GimbalController, GimbalFeedbackThread and adjustment modules.
add_executable(pa_gimbal
        dev/interface/can_interface.cpp
        dev/interface/gimbal_interface.cpp
        dev/module/pid_controller.hpp
        dev/scheduler/gimbal_scheduler.cpp
        dev/scheduler/shoot_scheduler.cpp
        dev/interface/ahrs/mpu6500.cpp
        dev/debug/unit_tests/pa_gimbal.cpp)
target_compile_definitions(pa_gimbal PRIVATE SHELL_NO_ECHO_MODE)

# MPU6500: include MP6500 interface and unit test.
add_executable(ut_mpu6500
        dev/interface/ahrs/mpu6500.cpp
        dev/debug/unit_tests/ut_mpu6500.cpp)

# IST8310
add_executable(ut_ist8310
        dev/interface/ahrs/mpu6500.cpp
        dev/interface/ahrs/ist8310.cpp
        dev/debug/unit_tests/ut_ist8310.cpp)

# IMU: include IMU interface and unit test.
add_executable(ut_imu
        dev/interface/ahrs/ahrs_math.hpp
        dev/interface/ahrs/mpu6500.cpp
        dev/interface/ahrs/ist8310.cpp
        dev/interface/ahrs/ahrs.cpp
        dev/debug/unit_tests/ut_ahrs.cpp)

# AHRS
add_executable(ut_ahrs
        dev/interface/ahrs/ahrs_math.hpp
        dev/interface/ahrs/mpu6500.cpp
        dev/interface/ahrs/ist8310.cpp
        dev/interface/ahrs/ahrs_lib_middleware.c
        dev/interface/ahrs/ahrs.cpp
        dev/debug/unit_tests/ut_ahrs.cpp)
target_link_libraries(ut_ahrs ${PROJECT_SOURCE_DIR}/dev/interface/ahrs/ahrs_lib.lib)

# AHRSExt
add_executable(ut_ahrs_ext
        dev/interface/can_interface.cpp
        dev/interface/ahrs/ahrs_ext.cpp
        dev/debug/unit_tests/ut_ahrs_ext.cpp)

# Buzzer: inlcude Buzzer interface and unit test.
add_executable(ut_buzzer
        dev/interface/buzzer.cpp
        dev/debug/unit_tests/ut_buzzer.cpp)

# Chassis: include chassis interface, chassis controller and the unit test modules.
add_executable(pa_chassis
        dev/module/pid_controller.hpp
        dev/interface/can_interface.cpp
        dev/interface/remote_interpreter.cpp
        ${AHRS_SRC}
        ${HERO_SHOOT_SRC}
        ${CHASSIS_SRC}
        dev/debug/unit_tests/pa_chassis/pa_chassis.cpp
        dev/debug/unit_tests/pa_chassis/inspector.cpp
        dev/debug/unit_tests/pa_chassis/command.cpp)
target_include_directories(pa_chassis PRIVATE dev/debug/unit_tests/pa_chassis)
target_compile_definitions(pa_chassis PRIVATE pa_chassis ENABLE_STATE_HANDLE)
target_link_libraries(pa_chassis ${PROJECT_SOURCE_DIR}/dev/interface/ahrs/ahrs_lib.lib)

# Elevator Interface: include CANInterface, ElevatorInterface, and the unit test modules.
add_executable(ut_elevator_interface
        dev/interface/can_interface.cpp
        dev/debug/unit_tests/ut_elevator_IF.cpp)

add_executable(ut_engineer_gimbal
        dev/interface/engineer_gimbal.cpp
        dev/interface/remote_interpreter.cpp
        dev/debug/unit_tests/engineer_gimbal_unit_test.cpp)

# Elevator: include CANInterface, ElevatorInterface, ElevatorController, and the unit test modules.
add_executable(pa_elevator
        dev/interface/can_interface.cpp
        dev/module/pid_controller.hpp
        dev/logic/engineer_elevator_logic.cpp
        dev/scheduler/engineer_elevator_skd.cpp
        dev/interface/engineer_elevator_interface.cpp
        dev/debug/unit_tests/engineer_elevator_param_adjust.cpp)

add_executable(pa_engineer_chassis
        dev/interface/can_interface.cpp
        dev/module/pid_controller.hpp
        dev/scheduler/engineer_chassis_skd.cpp
        dev/debug/unit_tests/engineer_chassis_param_adjust.cpp)

# Robotic Arm
add_executable(ut_robotic_arm
        dev/debug/unit_tests/engineer_robotic_arm_unit_test.cpp
        dev/interface/can_interface.cpp
        dev/interface/robotic_arm_interface.cpp
        dev/scheduler/robotic_arm_skd.cpp)

# Adjust Params of Skywalker-20A for friction wheels
add_executable(pa_skywalker
        dev/debug/unit_tests/pa_skywalker.cpp
        dev/interface/remote_interpreter.cpp
        dev/interface/buzzer.cpp)


# Adjust Params of Sentry Chassis
add_executable(pa_sentry_chassis
        dev/interface/can_interface.cpp
        dev/module/pid_controller.hpp
        dev/interface/sentry_chassis_interface.cpp
        dev/scheduler/sentry_chassis_scheduler.cpp
        dev/debug/unit_tests/sentry_chassis_param_adjust.cpp
        dev/interface/referee_interface.cpp
        dev/common/CRC8.cpp
        dev/common/CRC16.cpp)

add_executable(ut_referee_interface
        dev/common/CRC8.cpp
        dev/common/CRC16.cpp
        dev/interface/referee_interface.cpp
        dev/debug/unit_tests/ut_refereeIF.cpp)

add_executable(ut_vision_port
        dev/common/CRC8.cpp
        dev/common/CRC16.cpp
        dev/interface/vision_port.cpp
        dev/debug/unit_tests/ut_vision_port.cpp)

add_executable(ut_sd_card
        dev/interface/sd_card_interface.cpp
        dev/common/CRC16.cpp
        dev/debug/unit_tests/ut_sd_card.cpp)

add_executable(ut_adc
        dev/debug/unit_tests/ut_adc.cpp
        dev/interface/dms_interface.cpp)
add_executable(ca_ahrs_infantry
        dev/interface/ahrs/ahrs_math.hpp
        dev/interface/ahrs/mpu6500.cpp
        dev/interface/ahrs/ist8310.cpp
        dev/interface/ahrs/ahrs_lib_middleware.c
        dev/interface/ahrs/ahrs.cpp
        dev/common/CRC16.cpp
        dev/interface/sd_card_interface.cpp
        dev/debug/unit_tests/ca_ahrs.cpp)
target_link_libraries(ca_ahrs_infantry ${PROJECT_SOURCE_DIR}/dev/interface/ahrs/ahrs_lib.lib)
target_include_directories(ca_ahrs_infantry PRIVATE dev/vehicle/infantry)
target_compile_definitions(ca_ahrs_infantry PRIVATE INFANTRY)

add_executable(ca_ahrs_hero
        dev/interface/ahrs/ahrs_math.hpp
        dev/interface/ahrs/mpu6500.cpp
        dev/interface/ahrs/ist8310.cpp
        dev/interface/ahrs/ahrs_lib_middleware.c
        dev/interface/ahrs/ahrs.cpp
        dev/common/CRC16.cpp
        dev/interface/sd_card_interface.cpp
        dev/debug/unit_tests/ca_ahrs.cpp)
target_link_libraries(ca_ahrs_hero ${PROJECT_SOURCE_DIR}/dev/interface/ahrs/ahrs_lib.lib)
target_include_directories(ca_ahrs_hero PRIVATE dev/vehicle/hero)
target_compile_definitions(ca_ahrs_hero PRIVATE HERO)

# =========== Post Configurations for all targets ===========

# Make all targets compile to this filename to allow upload script to work properly
set(ELF_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build)
set(ELF_OUTPUT_NAME meta.elf)
message("[Notice] All executables are built to ${ELF_OUTPUT_PATH}/${ELF_OUTPUT_NAME}")

add_definitions(-DBUILD_TARGET_NAME="$<TARGET_PROPERTY:NAME>")

get_directory_property(TARGER_LIST BUILDSYSTEM_TARGETS)
foreach (target ${TARGER_LIST})
    if (NOT ${target} STREQUAL "ChibiOS_OBJ")  # OBJECT library doesn't have PRE_BUILD or POST_BUILD
        # Add ChibiOS files and commom sources
        target_sources(${target} PRIVATE $<TARGET_OBJECTS:ChibiOS_OBJ> ${COMMON_SRC})

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


