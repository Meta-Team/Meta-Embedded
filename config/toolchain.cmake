# This file includes configurations for ARM embedded development and toolchain.
# liuzikai 2018-02-09

# CMake system configs. These configs affects the way to pass flags.
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
set(CMAKE_SYSTEM_PROCESSOR arm)

# ---------- Toolchain Configurations ----------

# The following lines set the toolchain. Setting toolchain in CLion preference is also valid.
# Add support for compiling from CLI
if(WIN32)
# For windows system
set(EXTENSION ".exe")
else()
# For the UNIX like systems
set(EXTENSION "")
endif()
# To retain a clear environment by not directly adding it to system PATH
if(SDK_CLEAR_ENV)
    set(ARM_TOOLCHAIN_DIR "C:/Program Files (x86)/GNU Arm Embedded Toolchain/10 2021.10/bin/")
endif()
set(TOOLCHAIN_PREFIX "${ARM_TOOLCHAIN_DIR}arm-none-eabi-")
set(CMAKE_C_COMPILER   ${TOOLCHAIN_PREFIX}gcc${EXTENSION}    )
set(CMAKE_ASM_COMPILER ${TOOLCHAIN_PREFIX}gcc${EXTENSION}    )
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}g++${EXTENSION}   )
set(CMAKE_OBJCOPY      ${TOOLCHAIN_PREFIX}objcopy${EXTENSION})
set(CMAKE_SIZE_UTILITY ${TOOLCHAIN_PREFIX}size${EXTENSION})
set(CMAKE_EXE_LINKER_FLAGS "--specs=nosys.specs")


# The following lines create target to let CMake call Make directly. Now they are deprecated.

#if (CMAKE_HOST_SYSTEM_NAME MATCHES "Darwin")
#    message("CMAKE_HOST_SYSTEM_NAME = ${CMAKE_HOST_SYSTEM_NAME}, use `make` to compile")
#    set(MAKE_EXECUTABLE make)
#else()
#    message("CMAKE_HOST_SYSTEM_NAME = ${CMAKE_HOST_SYSTEM_NAME}, use `mingw32-make` to compile")
#    set(MAKE_EXECUTABLE mingw32-make)
#endif()
#add_custom_target(Main_Make COMMAND ${MAKE_EXECUTABLE} -j -C ${Meta_Infantry_SOURCE_DIR}/dev UNIT_TEST=NONE)