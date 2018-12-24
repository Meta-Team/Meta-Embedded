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
add_custom_target(Main COMMAND ${MAKE_EXECUTABLE} -j -C ${Meta_Infantry_SOURCE_DIR}/dev UNIT_TEST=NONE)
add_custom_target(UnitTest_Remote_Interpreter COMMAND ${MAKE_EXECUTABLE} -j -C ${Meta_Infantry_SOURCE_DIR}/dev UNIT_TEST=REMOTE_INTERPRETER)

# make clean
add_custom_target(clean_make COMMAND ${MAKE_EXECUTABLE} clean -C ${Meta_Infantry_SOURCE_DIR}/dev)

# Our source files
set(DEVCPPSRC
        dev/common/port_to_string.cpp
        dev/debug/button_monitor.cpp
        dev/debug/serial_shell.cpp
        dev/debug/serial_shell_commands.cpp
        dev/main.cpp dev/debug/led.cpp dev/remote_interpreter/remote_interpreter.cpp dev/remote_interpreter/remote_interpreter.hpp dev/remote_interpreter/remote_interpreter_unit_test.cpp dev/debug/unit_test_common.cpp dev/debug/unit_test_common.hpp dev/send_currents_functions.cpp dev/send_currents_functions.h)
# Our include directories
include_directories(dev
        dev/common
        dev/debug
        dev/remote_interpreter)


# This target is used to make CLion IDE work properly
# There is no need to edit them
include(os/ch.cmake)
add_library(useless_cmake_target ${DEVCPPSRC} ${ALLCSRC})

add_definitions("-DSHELL_CONFIG_FILE")