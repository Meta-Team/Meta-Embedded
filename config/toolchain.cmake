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
    if(DEFINED ENV{ARM_NONE_EABI_TOOLCHAIN_DIR})
	CMAKE_PATH(SET TOOLCHAIN_DIR NORMALIZE $ENV{ARM_NONE_EABI_TOOLCHAIN_DIR})
	MESSAGE("[Notice] Your arm-none-eabi toolchain dir is ${TOOLCHAIN_DIR}")
    else()
        MESSAGE(FATAL_ERROR "Please set ARM_NONE_EABI_TOOLCHAIN_DIR environment variable to the toolchain dir")
    endif()
endif()
set(TOOLCHAIN_PREFIX "${TOOLCHAIN_DIR}/arm-none-eabi-")
MESSAGE(${TOOLCHAIN_PREFIX})
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
