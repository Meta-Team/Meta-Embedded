# Extract from Makefiles and .mk files from ChibiOS.
# By liuzikai 2018-01-29


# license/license.mk
set(CHIBIOS_INCLUDE_DIRS ${CHIBIOS_INCLUDE_DIRS}
        license)

# common/startup/ARMCMx/compilers/GCC/mk/startup_stm32f4xx.mk
set(CHIBIOS_C_SRC ${CHIBIOS_C_SRC}
        common/startup/ARMCMx/compilers/GCC/crt1.c)
set(CHIBIOS_XASM_SRC ${CHIBIOS_XASM_SRC}
        common/startup/ARMCMx/compilers/GCC/crt0_v7m.S
        common/startup/ARMCMx/compilers/GCC/vectors.S)
set(CHIBIOS_INCLUDE_DIRS ${CHIBIOS_INCLUDE_DIRS}
        common/portability/GCC
        common/startup/ARMCMx/compilers/GCC
        common/startup/ARMCMx/devices/STM32F4xx
        common/ext/ARM/CMSIS/Core/Include
        common/ext/ST/STM32F4xx)


# hal/hal.mk
set(CHIBIOS_C_SRC ${CHIBIOS_C_SRC}
        hal/src/hal.c
        hal/src/hal_buffers.c
        hal/src/hal_queues.c
        hal/src/hal_mmcsd.c
        hal/src/hal_adc.c
        hal/src/hal_can.c
        hal/src/hal_crypto.c
        hal/src/hal_dac.c
        hal/src/hal_ext.c
        hal/src/hal_gpt.c
        hal/src/hal_i2c.c
        hal/src/hal_i2s.c
        hal/src/hal_icu.c
        hal/src/hal_mac.c
        hal/src/hal_mmc_spi.c
        hal/src/hal_pal.c
        hal/src/hal_pwm.c
        hal/src/hal_qspi.c
        hal/src/hal_rtc.c
        hal/src/hal_sdc.c
        hal/src/hal_serial.c
        hal/src/hal_serial_usb.c
        hal/src/hal_spi.c
        hal/src/hal_st.c
        hal/src/hal_uart.c
        hal/src/hal_usb.c
        hal/src/hal_wdg.c)
set(CHIBIOS_INCLUDE_DIRS ${CHIBIOS_INCLUDE_DIRS}
        hal/include)


# hal/ports/STM32/STM32F4xx/platform.mk
set(CHIBIOS_C_SRC ${CHIBIOS_C_SRC}
        hal/ports/common/ARMCMx/nvic.c
        hal/ports/STM32/STM32F4xx/stm32_isr.c
        hal/ports/STM32/STM32F4xx/hal_lld.c)
set(CHIBIOS_C_SRC ${CHIBIOS_C_SRC}
        hal/ports/STM32/LLD/ADCv2/hal_adc_lld.c
        hal/ports/STM32/LLD/CANv1/hal_can_lld.c
        hal/ports/STM32/LLD/DACv1/hal_dac_lld.c
        hal/ports/STM32/LLD/DMAv2/stm32_dma.c
        hal/ports/STM32/LLD/EXTIv1/hal_ext_lld.c
        hal/ports/STM32/LLD/GPIOv2/hal_pal_lld.c
        #        hal/lib/fallback/I2C/hal_i2c_lld.c
        hal/ports/STM32/LLD/I2Cv1/hal_i2c_lld.c
        hal/ports/STM32/LLD/MACv1/hal_mac_lld.c
        hal/ports/STM32/LLD/OTGv1/hal_usb_lld.c
        hal/ports/STM32/LLD/QUADSPIv1/hal_qspi_lld.c
        hal/ports/STM32/LLD/RTCv2/hal_rtc_lld.c
        hal/ports/STM32/LLD/SPIv1/hal_i2s_lld.c
        hal/ports/STM32/LLD/SPIv1/hal_spi_lld.c
        hal/ports/STM32/LLD/SDIOv1/hal_sdc_lld.c
        hal/ports/STM32/LLD/TIMv1/hal_gpt_lld.c
        hal/ports/STM32/LLD/TIMv1/hal_icu_lld.c
        hal/ports/STM32/LLD/TIMv1/hal_pwm_lld.c
        hal/ports/STM32/LLD/USARTv1/hal_serial_lld.c
        hal/ports/STM32/LLD/USARTv1/hal_uart_lld.c
        hal/ports/STM32/LLD/xWDGv1/hal_wdg_lld.c)
set(CHIBIOS_INCLUDE_DIRS ${CHIBIOS_INCLUDE_DIRS}
        hal/ports/common/ARMCMx
        hal/ports/STM32/STM32F4xx)
set(CHIBIOS_INCLUDE_DIRS ${CHIBIOS_INCLUDE_DIRS}
        hal/ports/STM32/LLD/ADCv2
        hal/ports/STM32/LLD/CANv1
        hal/ports/STM32/LLD/DACv1
        hal/ports/STM32/LLD/DMAv2
        hal/ports/STM32/LLD/EXTIv1
        hal/ports/STM32/LLD/GPIOv2
        hal/ports/STM32/LLD/I2Cv1
        hal/ports/STM32/LLD/MACv1
        hal/ports/STM32/LLD/OTGv1
        hal/ports/STM32/LLD/QUADSPIv1
        hal/ports/STM32/LLD/RTCv2
        hal/ports/STM32/LLD/SPIv1
        hal/ports/STM32/LLD/SDIOv1
        hal/ports/STM32/LLD/TIMv1
        hal/ports/STM32/LLD/USARTv1
        hal/ports/STM32/LLD/xWDGv1)

# hal/ports/STM32/LLD/TIMv1/driver.mk
set(CHIBIOS_C_SRC ${CHIBIOS_C_SRC}
        hal/ports/STM32/LLD/TIMv1/hal_st_lld.c
        hal/ports/STM32/LLD/TIMv1/hal_gpt_lld.c
        hal/ports/STM32/LLD/TIMv1/hal_icu_lld.c
        hal/ports/STM32/LLD/TIMv1/hal_pwm_lld.c)
set(CHIBIOS_INCLUDE_DIRS ${CHIBIOS_INCLUDE_DIRS}
        hal/ports/STM32/LLD/TIMv1)

## $(CHIBIOS)/dev/board/board.mk
#set(CHIBIOS_C_SRC ${CHIBIOS_C_SRC}
#        ${CHIBIOS}/dev/board/board.c)

set(CHIBIOS_INCLUDE_DIRS ${CHIBIOS_INCLUDE_DIRS}
        ${CHIBIOS}/dev/board)


# hal/osal/rt/osal.mk
set(CHIBIOS_C_SRC ${CHIBIOS_C_SRC}
        hal/osal/rt/osal.c)
set(CHIBIOS_INCLUDE_DIRS ${CHIBIOS_INCLUDE_DIRS}
        hal/osal/rt)


# rt/rt.mk
set(CHIBIOS_C_SRC ${CHIBIOS_C_SRC}
        rt/src/chsys.c
        rt/src/chdebug.c
        rt/src/chtrace.c
        rt/src/chvt.c
        rt/src/chschd.c
        rt/src/chthreads.c
        rt/src/chtm.c
        rt/src/chstats.c
        rt/src/chregistry.c
        rt/src/chsem.c
        rt/src/chmtx.c
        rt/src/chcond.c
        rt/src/chevents.c
        rt/src/chmsg.c
        rt/src/chdynamic.c
        common/oslib/src/chmboxes.c
        common/oslib/src/chmemcore.c
        common/oslib/src/chheap.c
        common/oslib/src/chmempools.c
        common/oslib/src/chfactory.c)
set(CHIBIOS_INCLUDE_DIRS ${CHIBIOS_INCLUDE_DIRS}
        rt/include
        common/oslib/include)


# common/ports/ARMCMx/compilers/GCC/mk/port_v7m.mk
set(CHIBIOS_C_SRC ${CHIBIOS_C_SRC}
        common/ports/ARMCMx/chcore.c
        common/ports/ARMCMx/chcore_v7m.c)
set(CHIBIOS_XASM_SRC ${CHIBIOS_XASM_SRC}
        common/ports/ARMCMx/compilers/GCC/chcoreasm_v7m.S)
set(CHIBIOS_INCLUDE_DIRS ${CHIBIOS_INCLUDE_DIRS}
        common/ports/ARMCMx
        common/ports/ARMCMx/compilers/GCC)


# various/cpp_wrappers/chcpp.mk
set(CHIBIOS_CPP_SRC ${CHIBIOS_CPP_SRC}
        various/cpp_wrappers/ch.cpp
        various/cpp_wrappers/syscalls_cpp.cpp)
set(CHIBIOS_INCLUDE_DIRS ${CHIBIOS_INCLUDE_DIRS}
        various/cpp_wrappers)


# hal/lib/streams/streams.mk
set(CHIBIOS_C_SRC ${CHIBIOS_C_SRC}
        #        hal/lib/streams/chprintf.c
        hal/lib/streams/memstreams.c
        hal/lib/streams/nullstreams.c)
set(CHIBIOS_INCLUDE_DIRS ${CHIBIOS_INCLUDE_DIRS}
        hal/lib/streams)


# various/shell/shell.mk
# Replaced by our shell

# Reuse ChibiOS object files to avoid repeat compile (not linked since the program is not yet completed)
add_library(ChibiOS OBJECT ${CHIBIOS_XASM_SRC} ${CHIBIOS_C_SRC} ${CHIBIOS_CPP_SRC})
target_include_directories(ChibiOS PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CHIBIOS_INCLUDE_DIRS})