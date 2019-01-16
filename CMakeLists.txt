set(CMAKE_CXX_STANDARD 11)

cmake_minimum_required(VERSION 3.10)
project(Meta_Infantry)


IF (CMAKE_SYSTEM_NAME MATCHES "Darwin")
    MESSAGE("Use make to compile.")
    set(MAKE_EXECUTABLE make)
ELSE()
    MESSAGE("Use mingw32-make to compile.")
    set(MAKE_EXECUTABLE mingw32-make)
ENDIF()

# Actual compile command
add_custom_target(MAIN COMMAND ${MAKE_EXECUTABLE} -j -C ${Meta_Infantry_SOURCE_DIR}/dev DEV_MODULE=MAIN)
add_custom_target(UT_REMOTE_INTERPRETER COMMAND ${MAKE_EXECUTABLE} -j -C ${Meta_Infantry_SOURCE_DIR}/dev DEV_MODULE=REMOTE_INTERPRETER)
add_custom_target(UT_GIMBAL_INTERFACE COMMAND ${MAKE_EXECUTABLE} -j -C ${Meta_Infantry_SOURCE_DIR}/dev DEV_MODULE=GIMBAL_INTERFACE)
add_custom_target(UT_GIMBAL COMMAND ${MAKE_EXECUTABLE} -j -C ${Meta_Infantry_SOURCE_DIR}/dev DEV_MODULE=GIMBAL)

# make clean
add_custom_target(MAKE_CLEAN COMMAND ${MAKE_EXECUTABLE} clean -C ${Meta_Infantry_SOURCE_DIR}/dev)

# Our source files
set(DEVCPPSRC
        dev/common/port_to_string.cpp
        dev/module/button_monitor.cpp
        dev/debug/serial_shell.cpp
        dev/debug/shell_debug_commands.cpp
        dev/main.cpp
        dev/debug/led.cpp
        dev/interfaces/remote_interpreter.cpp
        dev/interfaces/remote_interpreter_unit_test.cpp
        dev/interfaces/gimbal_interface_unit_test.cpp
        dev/interfaces/gimbal_interface.cpp dev/interfaces/gimbal_interface.h dev/interfaces/can_interface.cpp dev/interfaces/can_interface.h dev/debug/gimbal_feedback_module.cpp dev/debug/gimbal_feedback_module.h dev/module/pid_controller.cpp dev/module/pid_controller.h dev/control/gimbal_controller.cpp dev/control/gimbal_controller.h dev/control/gimbal_controller_unit_test.cpp dev/interfaces/shoot_interface.cpp dev/interfaces/shoot_interface.h dev/interfaces/elevator_interface.cpp dev/interfaces/elevator_interface.h)

# Our include directories
include_directories(dev
        dev/common
        dev/debug
        dev/interfaces
        dev/module
        dev/control)


# This target is used to make CLion IDE work properly
# There is no need to edit them
include(os/ch.cmake)
add_library(useless_cmake_target ${DEVCPPSRC} ${ALLCSRC})

add_definitions("-DSHELL_CONFIG_FILE")