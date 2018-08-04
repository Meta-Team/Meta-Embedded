set(CMAKE_CXX_STANDARD 11)

cmake_minimum_required(VERSION 3.10)
project(Insouled_ChibiOS_CPP)

# Actual compile command
add_custom_target(ch.elf COMMAND make -j -C ${Insouled_ChibiOS_CPP_SOURCE_DIR}/dev)

# make clean
add_custom_target(clean_make COMMAND make clean -C ${Insouled_ChibiOS_CPP_SOURCE_DIR}/dev)

# Our source files
set(DEVCPPSRC
        dev/common/port_to_string.cpp
        dev/debug/button_monitor.cpp
        dev/debug/serial_shell.cpp
        dev/debug/serial_shell_commands.cpp
        dev/main.cpp dev/debug/led.cpp)

# Our include directories
include_directories(dev
        dev/common
        dev/debug)


# This target is used to make CLion IDE work properly
# There is no need to edit them
include(os/ch.cmake)
add_library(cmake ${DEVCPPSRC} ${ALLCSRC})

add_definitions("-DSHELL_CONFIG_FILE")