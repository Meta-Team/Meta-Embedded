# This file is configuration for CMake build system.
# liuzikai 2018-01-29

cmake_minimum_required(VERSION 3.10)

include(config/toolchain.cmake)  # include cross compile configurations

set(CMAKE_CXX_STANDARD 11)
project(Meta_Infantry ASM C CXX) # NOTICE: project() must be in CMakeList.txt, rather than any cmake file. It must be
                                 # after toolchain configs and before flag configs for CMake to test compiler.

include(config/compile_options.cmake)  # include compile flags

set(CMAKE_VERBOSE_MAKEFILE OFF)  # Enable this if you want to see verbose log


# ---------- Common Files and Targets Configurations ----------

include(os/ch.cmake)  # add files and directories from ChibiOS

set(COMMON_CPP_SRC  # common cpp sources
        dev/common/port_to_string.cpp
        dev/debug/serial_shell.cpp
        dev/debug/shell_debug_commands.cpp
        dev/interfaces/led.cpp
        dev/interfaces/buzzer.cpp)

include_directories(  # common include directories
        dev
        dev/common
        dev/debug
        dev/interfaces
        dev/module
        dev/control
        dev/vehicle_configs)

include_directories(dev/board/${BOARD_NAME})  # include board configs

# Reuse ChibiOS object files (without linking) to avoid repeat compile
add_library(ChibiOS_OBJ OBJECT ${CHIBIOS_XASM_SRC} ${CHIBIOS_C_SRC} ${CHIBIOS_CPP_SRC} dev/board/${BOARD_NAME}/board.c)

# NOTICE: if you want to adapt this project to a new board, be sure to check startup file congif in compile_options.cmake

# ---------- Files and Targets Configurations ----------

# Main Program -- Infantry #1
add_executable(Infantry_One $<TARGET_OBJECTS:ChibiOS_OBJ> ${COMMON_CPP_SRC}
        dev/main.cpp)
target_compile_definitions(Infantry_One PRIVATE VEHICLE=INFANTRY_ONE)

# Main Program -- Engineer
add_executable(Engineer $<TARGET_OBJECTS:ChibiOS_OBJ> ${COMMON_CPP_SRC}
        dev/main.cpp)
target_compile_definitions(Engineer PRIVATE VEHICLE=ENGINEER)

# Blink
add_executable(ut_blink $<TARGET_OBJECTS:ChibiOS_OBJ> ${COMMON_CPP_SRC}
        dev/module/button_monitor.cpp
        dev/debug/main_blink.cpp)

# Remote Interpreter: include remote_interpreter sources and the unit test modules.
add_executable(ut_remote_interpreter $<TARGET_OBJECTS:ChibiOS_OBJ> ${COMMON_CPP_SRC}
        dev/interfaces/remote_interpreter.cpp
        dev/interfaces/remote_interpreter_unit_test.cpp)

# Gimbal Interface: include CANInterface, GimbalInterface, GimbalFeedbackModule and the unit test modules.
add_executable(ut_gimbal_interface $<TARGET_OBJECTS:ChibiOS_OBJ> ${COMMON_CPP_SRC}
        dev/interfaces/can_interface.cpp
        dev/interfaces/gimbal_interface.cpp
        dev/interfaces/gimbal_interface_unit_test.cpp
        dev/debug/gimbal_feedback_module.cpp)

# Gimbal: include CANInterface, GimbalInterface, GimbalController, GimbalFeedbackModule and test & adjustment modules.
add_executable(ut_gimbal $<TARGET_OBJECTS:ChibiOS_OBJ> ${COMMON_CPP_SRC}
        dev/interfaces/can_interface.cpp
        dev/interfaces/gimbal_interface.cpp
        dev/module/pid_controller.cpp
        dev/control/gimbal_controller.cpp
        dev/debug/gimbal_feedback_module.cpp
        dev/control/gimbal_controller_unit_test.cpp
        dev/interfaces/mpu6500.cpp)


# MPU6500: include MP6500 interface and unit test.
add_executable(ut_mpu6500 $<TARGET_OBJECTS:ChibiOS_OBJ> ${COMMON_CPP_SRC}
        dev/interfaces/mpu6500.cpp
        dev/interfaces/mpu6500_unit_test.cpp)

# Elevator Interface: include CANInterface, ElevatorInterface, and the unit test modules.
add_executable(ut_elevator $<TARGET_OBJECTS:ChibiOS_OBJ> ${COMMON_CPP_SRC}
        dev/interfaces/can_interface.cpp
        dev/interfaces/elevator_interface.cpp
        dev/interfaces/elevator_interface_unit_test.cpp)

# Buzzer: inlcude Buzzer interface and unit test.
add_executable(ut_buzzer $<TARGET_OBJECTS:ChibiOS_OBJ> ${COMMON_CPP_SRC}
        dev/interfaces/buzzer.cpp
        dev/interfaces/buzzer_unit_test.cpp)


# ---------- Post Configurations for all targets ----------

# Make all targets compile to this filename to allow upload script to work properly
set(ELF_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build)
set(ELF_OUTPUT_NAME meta.elf)
message("[Notice] All executables are built to ${ELF_OUTPUT_PATH}/${ELF_OUTPUT_NAME}")

get_directory_property(TARGER_LIST BUILDSYSTEM_TARGETS)
foreach (target ${TARGER_LIST})
    if (NOT ${target} STREQUAL "ChibiOS_OBJ")  # OBJECT library doesn't have PRE_BUILD or POST_BUILD
        add_custom_command(TARGET ${target} PRE_BUILD
                COMMENT "Build for ${BOARD_NAME}")  # echo board type
        add_custom_command(TARGET ${target} POST_BUILD
                COMMAND echo
                COMMAND ${CMAKE_SIZE_UTILITY} ${ELF_OUTPUT_PATH}/${ELF_OUTPUT_NAME}
                COMMAND echo)  # execute size utility to show size of executable file
        set_target_properties(${target} PROPERTIES
                RUNTIME_OUTPUT_DIRECTORY ${ELF_OUTPUT_PATH}
                OUTPUT_NAME ${ELF_OUTPUT_NAME})  #
    endif ()
endforeach (target)


