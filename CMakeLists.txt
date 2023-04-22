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


