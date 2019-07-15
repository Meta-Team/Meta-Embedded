# This file includes configurations for build flags.
# liuzikai 2018-02-09

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
set(LD_OPT_FLAGS ",--gc-sections,--defsym=__process_stack_size__=0x800,--defsym=__main_stack_size__=0x800")

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

if (${BOARD_NAME} STREQUAL rm_board_2017)
    set(STARTUP_LD_DICT ${PROJECT_SOURCE_DIR}/os/common/startup/ARMCMx/compilers/GCC/ld)
    set(STARTUP_LD_FILE ${STARTUP_LD_DICT}/STM32F429xI.ld)
elseif (${BOARD_NAME} STREQUAL rm_board_2018_a)
    set(STARTUP_LD_DICT ${PROJECT_SOURCE_DIR}/os/common/startup/ARMCMx/compilers/GCC/ld)
    set(STARTUP_LD_FILE ${STARTUP_LD_DICT}/STM32F429xI.ld)
else ()
    message(FATAL_ERROR "[ERROR] Dev board \"${BOARD_NAME}\" is not configuared." )
endif ()


# SET FLAGS TO CMAKE

set(CMAKE_ASM_FLAGS "-x assembler-with-cpp ${MCU_FLAGS} ${OPT_FLAGS} ${ASM_DEFS} ${THUMB_FLAGS}" CACHE INTERNAL "asm compiler flags" FORCE)
set(CMAKE_C_FLAGS "${MCU_FLAGS} ${OPT_FLAGS} ${C_OPT_FLAGS} ${C_WARN_FLAGS} ${DEFS} ${THUMB_FLAGS}" CACHE INTERNAL "c compiler flags" FORCE)
set(CMAKE_CXX_FLAGS "${MCU_FLAGS} ${OPT_FLAGS} ${CPP_OPT_FLAGS} ${CPP_WARN_FLAGS} ${DEFS} ${THUMB_FLAGS}" CACHE INTERNAL "cxx compiler flags" FORCE)
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -nostartfiles -L\"${STARTUP_LD_DICT}\" -Wl,--no-warn-mismatch,--library-path=\"${STARTUP_LD_DICT}\",--script=\"${STARTUP_LD_FILE}\"${LD_OPT_FLAGS}" CACHE INTERNAL "linker flags" FORCE)
# CMake will pass CXX_FLAGS to linker since it's a CXX linker, so some flags are excluded to avoid duplicate
# Omitted flags from Makefiles:
#   -Wl,-Map=${PROJECT_SOURCE_DIR}/build/ch.map (for .map files generation)
#   -Wl,--cref (output Cross Reference Table)