if (DEFINED BOARD_NAME)
    add_subdirectory(cmsis)
endif ()

if (DEFINED BOARD_NAME)
    set(gtest_disable_pthreads ON CACHE BOOL "Disable pthreads as STM32 doesn't support it")
endif ()
add_subdirectory(gtest)

add_subdirectory(crc)