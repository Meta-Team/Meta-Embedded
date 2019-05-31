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

set(COMMON_CPP_SRC  # common cpp sources, which are included to every tartget at post configuration below
        dev/debug/shell/shell_base.c
        dev/debug/shell/shell_base_cmd.c
        dev/debug/shell/printf.c
        dev/debug/shell/shell.cpp
        dev/debug/shell/shell_dbg_cmd.cpp
        dev/interface/led.cpp
        dev/interface/buzzer.cpp
        dev/control/state_handler.cpp)

include_directories(  # common include directories
        dev
        dev/common
        dev/debug
        dev/debug/shell
        dev/interface
        dev/interface/imu
        dev/module
        dev/control)

include_directories(dev/board/${BOARD_NAME})  # include board configs

# Reuse ChibiOS object files (without linking) to avoid repeat compile
# They are included to every tartget at post configuration below
add_library(ChibiOS_OBJ OBJECT ${CHIBIOS_XASM_SRC} ${CHIBIOS_C_SRC} ${CHIBIOS_CPP_SRC} dev/board/${BOARD_NAME}/board.c)


# =========== Files and Targets Configurations ===========

# NOTICE: UPDATE README IF THE CONFIGURATIONS IS UPDATED

set(INFANTRY
        dev/module/pid_controller.hpp
        dev/module/can_interface.cpp
        dev/interface/imu/mpu6500.cpp
        dev/interface/gimbal_interface.cpp
        dev/interface/chassis_interface.cpp
        dev/interface/remote_interpreter.cpp
        dev/control/gimbal.cpp
        dev/control/shoot.cpp
        dev/control/chassis.cpp
        dev/vehicle/infantry/main_infantry.cpp)

# Main Program -- Infantry #1
add_executable(INFANTRY_THREE ${INFANTRY})
target_include_directories(INFANTRY_THREE PRIVATE dev/vehicle/infantry)
target_compile_definitions(INFANTRY_THREE PRIVATE INFANTRY INFANTRY_THREE ENABLE_STATE_HANDLE)

# Main Program -- Infantry #4
add_executable(INFANTRY_FOUR ${INFANTRY})
target_include_directories(INFANTRY_FOUR PRIVATE dev/vehicle/infantry)
target_compile_definitions(INFANTRY_FOUR PRIVATE INFANTRY INFANTRY_FOUR ENABLE_STATE_HANDLE)

# Main Program -- Infantry #5
add_executable(INFANTRY_FIVE ${INFANTRY} dev/control/sentry_chassis_calculator.cpp dev/control/sentry_chassis_calculator.h)
target_include_directories(INFANTRY_FIVE PRIVATE dev/vehicle/infantry)
target_compile_definitions(INFANTRY_FIVE PRIVATE INFANTRY INFANTRY_FIVE ENABLE_STATE_HANDLE)

# Main Program -- Engineer
add_executable(ENGINEER
        dev/module/pid_controller.hpp
        dev/module/can_interface.cpp
        dev/interface/chassis_interface.cpp
        dev/interface/remote_interpreter.cpp
        dev/interface/elevator_interface.cpp
        dev/interface/robotic_arm.cpp
        dev/control/chassis.cpp
        dev/vehicle/engineer/elevator_thread.cpp
        dev/vehicle/engineer/robotic_arm_thread.cpp
        dev/vehicle/engineer/main_engineer.cpp)
target_include_directories(ENGINEER PRIVATE dev/vehicle/engineer)
target_compile_definitions(ENGINEER PRIVATE ENGINEER ENABLE_STATE_HANDLE)

# Main Program -- Hero
add_executable(HERO
        dev/module/pid_controller.hpp
        dev/module/can_interface.cpp
        dev/interface/mpu6500.cpp
        dev/interface/chassis_interface.cpp
        dev/interface/gimbal_interface.cpp
        dev/interface/remote_interpreter.cpp
        dev/interface/elevator_interface.cpp
        dev/interface/robotic_arm.cpp
        dev/control/chassis.cpp
        dev/control/gimbal.cpp
        dev/control/shoot.cpp
        dev/vehicle/hero/main_hero.cpp)
target_include_directories(HERO PRIVATE dev/vehicle/hero)
target_compile_definitions(HERO PUBLIC HERO ENABLE_STATE_HANDLE)

# Blink
add_executable(ut_blink
        dev/module/button_monitor.cpp
        dev/debug/unit_tests/led_unit_test.cpp)

# Remote Interpreter: include remote_interpreter sources and the unit test modules.
add_executable(ut_remote_interpreter
        dev/interface/remote_interpreter.cpp
        dev/debug/unit_tests/remote_interpreter_unit_test.cpp)

# Gimbal Interface: include CANInterface, GimbalInterface, GimbalFeedbackThread and the unit test modules.
add_executable(ut_gimbal_interface
        dev/module/can_interface.cpp
        dev/interface/imu/mpu6500.cpp
        dev/interface/gimbal_interface.cpp
        dev/control/shoot.cpp
        dev/debug/unit_tests/gimbal_interface_unit_test.cpp)
# Shoot
add_executable(ut_shoot
        dev/module/can_interface.cpp
        dev/interface/mpu6500.cpp
        dev/interface/gimbal_interface.cpp
        dev/control/shoot.cpp
        dev/debug/unit_tests/hero_shoot_unit_test.cpp)

# Shoot
add_executable(pa_hero_shoot
        dev/module/can_interface.cpp
        dev/interface/gimbal_interface.cpp
        dev/control/shoot.cpp
        dev/debug/unit_tests/hero_shoot_param_adjust.cpp)

# Gimbal: include CANInterface, GimbalInterface, GimbalController, GimbalFeedbackThread and adjustment modules.
add_executable(pa_gimbal
        dev/module/can_interface.cpp
        dev/interface/gimbal_interface.cpp
        dev/module/pid_controller.hpp
        dev/control/gimbal.cpp
        dev/control/shoot.cpp
        dev/interface/imu/mpu6500.cpp
        dev/debug/unit_tests/gimbal_pid_adjustment_program.cpp)
target_compile_definitions(pa_gimbal PRIVATE SHELL_NO_ECHO_MODE)

# MPU6500: include MP6500 interface and unit test.
add_executable(ut_mpu6500
        dev/interface/imu/mpu6500.cpp
        dev/debug/unit_tests//mpu6500_unit_test.cpp)

# IMU: include IMU interface and unit test.
add_executable(ut_imu
        dev/interface/imu/imu_math.hpp
        dev/interface/imu/mpu6500.cpp
#        dev/interface/imu/ist8310.cpp
        dev/interface/imu/ahrs.cpp
        dev/debug/unit_tests/imu_unit_test.cpp)

# Buzzer: inlcude Buzzer interface and unit test.
add_executable(ut_buzzer
        dev/interface/buzzer.cpp
        dev/debug/unit_tests/buzzer_unit_test.cpp)

# Chassis Interface: include chassis interface sources and the unit test modules.
add_executable(ut_chassis_interface
        dev/module/can_interface.cpp
        dev/interface/chassis_interface.cpp
        dev/debug/unit_tests/chassis_interface_unit_test.cpp)

# Chassis: include chassis interface, chassis controller and the unit test modules.
add_executable(ut_chassis
        dev/module/can_interface.cpp
        dev/interface/chassis_interface.cpp
        dev/module/pid_controller.hpp
        dev/control/chassis.cpp
        dev/debug/unit_tests/chassis_unit_test.cpp)
target_include_directories(ut_chassis PRIVATE dev/vehicle/infantry)

# Elevator Interface: include CANInterface, ElevatorInterface, and the unit test modules.
add_executable(ut_elevator_interface
        dev/module/can_interface.cpp
        dev/interface/elevator_interface.cpp
        dev/debug/unit_tests/elevator_interface_unit_test.cpp)

# Elevator: include CANInterface, ElevatorInterface, ElevatorController, and the unit test modules.
add_executable(pa_elevator
        dev/module/can_interface.cpp
        dev/interface/elevator_interface.cpp
        dev/interface/chassis_interface.cpp
        dev/control/chassis.cpp
        dev/module/pid_controller.hpp
        dev/debug/unit_tests/engineer_chassis_elevator_unit_test.cpp
        dev/vehicle/engineer/elevator_thread.cpp)

# Robotic Arm
add_executable(ut_robotic_arm
        dev/module/can_interface.cpp
        dev/interface/robotic_arm.cpp
        dev/vehicle/engineer/robotic_arm_thread.cpp
        dev/debug/unit_tests/engineer_robotic_arm_unit_test.cpp)

# Adjust Params of Skywalker-20A for friction wheels
add_executable(pa_skywalker
        dev/debug/unit_tests/skywalker_param_adjust.cpp
        dev/interface/remote_interpreter.cpp
        dev/interface/buzzer.cpp)

# Sentry Chassis
add_executable(ut_sentry_chassis
        dev/module/can_interface.cpp
        dev/interface/sentry_chassis_interface.cpp
        dev/debug/unit_tests/sentry_chassis_unit_test.cpp)

add_executable(ut_referee_interface
        dev/module/CRC8.cpp
        dev/module/CRC16.cpp
        dev/interface/referee_interface.cpp
        dev/debug/unit_tests/referee_interface_unit_test.cpp)

# Template Unit Test
add_executable(ut_template
        dev/debug/unit_tests/unit_test_template.cpp)

# =========== Post Configurations for all targets ===========

# Make all targets compile to this filename to allow upload script to work properly
set(ELF_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build)
set(ELF_OUTPUT_NAME meta.elf)
message("[Notice] All executables are built to ${ELF_OUTPUT_PATH}/${ELF_OUTPUT_NAME}")

get_directory_property(TARGER_LIST BUILDSYSTEM_TARGETS)
foreach (target ${TARGER_LIST})
    if (NOT ${target} STREQUAL "ChibiOS_OBJ")  # OBJECT library doesn't have PRE_BUILD or POST_BUILD
        # Add ChibiOS files and commom sources
        target_sources(${target} PRIVATE $<TARGET_OBJECTS:ChibiOS_OBJ> ${COMMON_CPP_SRC})

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


