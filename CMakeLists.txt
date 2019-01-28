set(CMAKE_CXX_STANDARD 11)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
set(CMAKE_SYSTEM_PROCESSOR arm)

cmake_minimum_required(VERSION 3.10)
project(Meta_Infantry C ASM CXX)

#set(TOOLCHAIN_PREFIX arm-none-eabi-)
#
#set(CMAKE_C_COMPILER   ${TOOLCHAIN_PREFIX}gcc    )
#set(CMAKE_ASM_COMPILER ${CMAKE_C_COMPLIER}       )
#set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}g++    )
#set(CMAKE_OBJCOPY      ${TOOLCHAIN_PREFIX}objcopy)
#set(CMAKE_SIZE_UTIL    ${TOOLCHAIN_PREFIX}size   )

#set(CMAKE_EXE_LINKER_FLAGS "--specs=nosys.specs")

#set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM BOTH)
#set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
#set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
#set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

set(CMAKE_CXX_IMPLICIT_LINK_LIBRARIES "")
set(CMAKE_CXX_IMPLICIT_LINK_DIRECTORIES "")
set(CMAKE_C_IMPLICIT_LINK_LIBRARIES "")
set(CMAKE_C_IMPLICIT_LINK_DIRECTORIES "")
set(CMAKE_ASM_IMPLICIT_LINK_LIBRARIES "")
set(CMAKE_ASM_IMPLICIT_LINK_DIRECTORIES "")

if (CMAKE_HOST_SYSTEM_NAME MATCHES "Darwin")
    message("CMAKE_HOST_SYSTEM_NAME = ${CMAKE_HOST_SYSTEM_NAME}, use `make` to compile")
    set(MAKE_EXECUTABLE make)
else()
    message("CMAKE_HOST_SYSTEM_NAME = ${CMAKE_HOST_SYSTEM_NAME}, use `mingw32-make` to compile")
    set(MAKE_EXECUTABLE mingw32-make)
endif()

set(MAKE_EXECUTABLE make)

# TODO: Change unit test command to CMake
# Actual compile command
add_custom_target(Main_Make COMMAND ${MAKE_EXECUTABLE} -j -C ${Meta_Infantry_SOURCE_DIR}/dev UNIT_TEST=NONE)
add_custom_target(UnitTest_Remote_Interpreter COMMAND ${MAKE_EXECUTABLE} -j -C ${Meta_Infantry_SOURCE_DIR}/dev UNIT_TEST=REMOTE_INTERPRETER)

# make clean

include(os/ch.cmake)

# Our source files
set(ALLCPPSRC ${ALLCPPSRC}
        dev/common/port_to_string.cpp
        dev/debug/button_monitor.cpp
        dev/debug/serial_shell.cpp
        dev/debug/serial_shell_commands.cpp
        dev/main.cpp
        dev/debug/led.cpp)

# Our include directories
include_directories(dev
        dev/common
        dev/debug
        dev/remote_interpreter)


set(OPT "-O0 -ggdb -fomit-frame-pointer -falign-functions=16 -DSHELL_CONFIG_FILE")
set(OPT "${OPT} -ffunction-sections -fdata-sections -fno-common") # USE_LINK_GC
set(OPT "${OPT} -flto") # USE_LTO
set(OPT "${OPT} -mfloat-abi=hard -mfpu=fpv4-sp-d16") # USE_FPU_OPT

set(LDOPT ",--gc-sections,--defsym=__process_stack_size__=0x400,--defsym=__main_stack_size__=0x400")
set(COPT "")
set(CPPOPT "-std=gnu++11 -fno-exceptions -fno-rtti")

set(CWARN "-Wall -Wextra -Wundef -Wstrict-prototypes")
set(CPPWARN "-Wall -Wextra -Wundef")

set(DEFS "-DCORTEX_USE_FPU=TRUE")
set(ADEFS "-DCORTEX_USE_FPU=TRUE")

set(MCFLAGS "-mcpu=cortex-m4")
set(ODFLAGS "-x --syms")


set(CMAKE_VERBOSE_MAKEFILE OFF)


set(THUMBFLAGS "-mthumb -DTHUMB -DTHUMB_PRESENT -mno-thumb-interwork -DTHUMB_NO_INTERWORKING")
# Omitted flags: -MD -MP -MF .build/dep/(filename).d (For .d files generation)

# TODO: look into CACHE
# TODO: extract startup script
set(CMAKE_ASM_FLAGS " -x assembler-with-cpp ${MCFLAGS} ${OPT} ${ADEFS} ${THUMBFLAGS}" CACHE INTERNAL "asm compiler flags" FORCE)
set(CMAKE_C_FLAGS "${MCFLAGS} ${OPT} ${COPT} ${CWARN} ${DEFS} ${THUMBFLAGS}" CACHE INTERNAL "c compiler flags" FORCE)
set(CMAKE_CXX_FLAGS "${MCFLAGS} ${OPT} ${CPPOPT} ${CPPWARN} ${DEFS} ${THUMBFLAGS}" CACHE INTERNAL "cxx compiler flags" FORCE)
set(CMAKE_EXE_LINKER_FLAGS "-nostartfiles -L${PROJECT_SOURCE_DIR}/os/common/startup/ARMCMx/compilers/GCC/ld/ -Wl,--no-warn-mismatch,--library-path=${PROJECT_SOURCE_DIR}/os/common/startup/ARMCMx/compilers/GCC/ld,--script=${PROJECT_SOURCE_DIR}/os/common/startup/ARMCMx/compilers/GCC/ld/STM32F429xI.ld${LDOPT}" CACHE INTERNAL "linker flags" FORCE)
# Omitted flags:
#   -Wl,-Map=${PROJECT_SOURCE_DIR}/build/ch.map (for .map files generation)
#   -Wl,--cref (output Cross Reference Table)

add_executable(Infantry.elf ${ALLCPPSRC} ${ALLCSRC} ${ALLXASMSRC})

# TODO: add size display after build