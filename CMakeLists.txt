# This file is configuration for CMake build system.

# By liuzikai 2018-01-29

cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 11)

# CMake system configs. These configs affects the way to pass flags.
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
set(CMAKE_SYSTEM_PROCESSOR arm)

project(Meta_Infantry ASM C CXX)

# ---------- Toolchain Configurations ----------

# The following lines set the toolchain. No we use CLion Preference to config toolchain, so they have no use.

set(TOOLCHAIN_PREFIX arm-none-eabi-)
#set(CMAKE_C_COMPILER   ${TOOLCHAIN_PREFIX}gcc    )
#set(CMAKE_ASM_COMPILER ${CMAKE_C_COMPLIER}       )
#set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}g++    )
#set(CMAKE_OBJCOPY      ${TOOLCHAIN_PREFIX}objcopy)
set(CMAKE_SIZE_UTILITY  ${TOOLCHAIN_PREFIX}size   )
#set(CMAKE_EXE_LINKER_FLAGS "--specs=nosys.specs")


# The following lines create target to let CMake call Make directly. Now they are deprecated.

#if (CMAKE_HOST_SYSTEM_NAME MATCHES "Darwin")
#    message("CMAKE_HOST_SYSTEM_NAME = ${CMAKE_HOST_SYSTEM_NAME}, use `make` to compile")
#    set(MAKE_EXECUTABLE make)
#else()
#    message("CMAKE_HOST_SYSTEM_NAME = ${CMAKE_HOST_SYSTEM_NAME}, use `mingw32-make` to compile")
#    set(MAKE_EXECUTABLE mingw32-make)
#endif()
#add_custom_target(Main_Make COMMAND ${MAKE_EXECUTABLE} -j -C ${Meta_Infantry_SOURCE_DIR}/dev UNIT_TEST=NONE)



# ---------- Flags Configurations ----------

# Compiler optimization flags, for ASM, C and CPP
#   -O and -g flag is auto set by CMake depending on debug mode (-g for Debug, -O3 -DNDEBUG for Release)
set(OPT_FLAGS "-fomit-frame-pointer -falign-functions=16")

# USE_LINK_GC: Enable this if you want the linker to remove unused code and data
set(OPT_FLAGS "${OPT_FLAGS} -ffunction-sections -fdata-sections -fno-common")

# USE_LTO: Enable this if you want link time optimizations (LTO)
set(OPT_FLAGS "${OPT_FLAGS} -flto")

# USE_FPU_OPT
set(OPT_FLAGS "${OPT_FLAGS} -mfloat-abi=hard -mfpu=fpv4-sp-d16")

# Linker optimization flags
set(LD_OPT_FLAGS ",--gc-sections,--defsym=__process_stack_size__=0x400,--defsym=__main_stack_size__=0x400")

# Flags for C sources only
set(C_OPT_FLAGS "")

# Flags for CPP sources only
set(CPP_OPT_FLAGS "-std=gnu++11 -fno-exceptions -fno-rtti")

# Warning flags for C sources only
set(C_WARN_FLAGS "-Wall -Wextra -Wundef -Wstrict-prototypes")

# Warning flags for CPP sources only
set(CPP_WARN_FLAGS "-Wall -Wextra -Wundef")

# Defintions for C and CPP sources
set(DEFS "-DCORTEX_USE_FPU=TRUE -DSHELL_CONFIG_FILE")

# Defintions for ASM sources
set(ASM_DEFS "-DCORTEX_USE_FPU=TRUE")

# MCU config flags
set(MCU_FLAGS "-mcpu=cortex-m4")

# Flags for objdump
#set(ODFLAGS "-x --syms")

# Flags for thumb compile mode.
set(THUMB_FLAGS "-mthumb -DTHUMB -DTHUMB_PRESENT -mno-thumb-interwork -DTHUMB_NO_INTERWORKING")
# Some defintions are not needed for ARM or linker, but it doesn't matter so they are all included :)
# Omitted flags from Makefiles: -MD -MP -MF .build/dep/(filename).d (For .d files generation)

# Startup LD directory and LD file
set(STARTUP_LD_DICT ${PROJECT_SOURCE_DIR}/os/common/startup/ARMCMx/compilers/GCC/ld)
set(STARTUP_LD_FILE ${STARTUP_LD_DICT}/STM32F429xI.ld)


# SET FLAGS TO CMAKE

set(CMAKE_ASM_FLAGS "-x assembler-with-cpp ${MCU_FLAGS} ${OPT_FLAGS} ${ASM_DEFS} ${THUMB_FLAGS}" CACHE INTERNAL "asm compiler flags" FORCE)
set(CMAKE_C_FLAGS "${MCU_FLAGS} ${OPT_FLAGS} ${C_OPT_FLAGS} ${C_WARN_FLAGS} ${DEFS} ${THUMB_FLAGS}" CACHE INTERNAL "c compiler flags" FORCE)
set(CMAKE_CXX_FLAGS "${MCU_FLAGS} ${OPT_FLAGS} ${CPP_OPT_FLAGS} ${CPP_WARN_FLAGS} ${DEFS} ${THUMB_FLAGS}" CACHE INTERNAL "cxx compiler flags" FORCE)
set(CMAKE_EXE_LINKER_FLAGS "-nostartfiles -L${STARTUP_LD_DICT} -Wl,--no-warn-mismatch,--library-path=${STARTUP_LD_DICT},--script=${STARTUP_LD_FILE}${LD_OPT_FLAGS}" CACHE INTERNAL "linker flags" FORCE)
# Omitted flags from Makefiles:
#   -Wl,-Map=${PROJECT_SOURCE_DIR}/build/ch.map (for .map files generation)
#   -Wl,--cref (output Cross Reference Table)


# Enable this if you want to see verbose log
set(CMAKE_VERBOSE_MAKEFILE OFF)




# ---------- Files and Targets Configurations ----------

include(os/ch.cmake)  # add files and directories from ChibiOS

set(COMMON_CPP_SRC
        dev/common/port_to_string.cpp
        dev/debug/serial_shell.cpp
        dev/debug/shell_debug_commands.cpp
        dev/debug/led.cpp)

include_directories(dev
        dev/board
        dev/common
        dev/debug
        dev/interfaces
        dev/module
        dev/control)

add_library(ChibiOS_OBJ OBJECT ${CHIBIOS_XASM_SRC} ${CHIBIOS_C_SRC} ${CHIBIOS_CPP_SRC})

add_executable(${PROJECT_NAME}.elf $<TARGET_OBJECTS:ChibiOS_OBJ> ${COMMON_CPP_SRC}
        dev/main.cpp)

# ===========================================================================
# REMOTE_INTERPRETER MODULES
#
#	Include remote_interpreter sources and the unit test modules.
#
# ===========================================================================
add_executable(ut_remote_interpreter.elf $<TARGET_OBJECTS:ChibiOS_OBJ> ${COMMON_CPP_SRC}
    dev/interfaces/remote_interpreter.cpp
    dev/interfaces/remote_interpreter_unit_test.cpp)

# ===========================================================================
# GIMBAL_INTERFACE MODULES
#
#	Include CANInterface, GimbalInterface, GimbalFeedbackModule and the unit
#   test modules.
#
# ===========================================================================
add_executable(ut_gimbal_interface.elf $<TARGET_OBJECTS:ChibiOS_OBJ> ${COMMON_CPP_SRC}
        dev/interfaces/can_interface.cpp
        dev/interfaces/gimbal_interface.cpp
        dev/interfaces/gimbal_interface_unit_test.cpp
        dev/debug/gimbal_feedback_module.cpp)

# ===========================================================================
# GIMBAL MODULES
#
#	Include CANInterface, GimbalInterface, GimbalController,
# 	GimbalFeedbackModule and the unit test and adjustment modules.
#
# ===========================================================================

add_executable(ut_gimbal.elf $<TARGET_OBJECTS:ChibiOS_OBJ> ${COMMON_CPP_SRC}
        dev/interfaces/can_interface.cpp
        dev/interfaces/gimbal_interface.cpp
        dev/module/pid_controller.cpp
        dev/control/gimbal_controller.cpp
        dev/debug/gimbal_feedback_module.cpp
        dev/control/gimbal_controller_unit_test.cpp
        dev/interfaces/mpu6500.cpp)


# ===========================================================================
# MPU6500 MODULES
#
#	Include MP6500 interface and unit test.
#
# ===========================================================================

add_executable(ut_mpu6500.elf $<TARGET_OBJECTS:ChibiOS_OBJ> ${COMMON_CPP_SRC}
        dev/interfaces/mpu6500.cpp
        dev/interfaces/mpu6500_unit_test.cpp)


# ===========================================================================
# ELEVATOR_INTERFACE MODULES
#
#	Include CANInterface, ElevatorInterface, and the unit test modules.
#
# ===========================================================================
add_executable(ut_elevator.elf $<TARGET_OBJECTS:ChibiOS_OBJ> ${COMMON_CPP_SRC}
        dev/interfaces/can_interface.cpp
        dev/interfaces/elevator_interface.cpp
        dev/interfaces/elevator_interface_unit_test.cpp)


# Run size utility after build
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND echo "\n"
        COMMAND ${CMAKE_SIZE_UTILITY} ${PROJECT_NAME}.elf
        COMMAND echo "\n")
