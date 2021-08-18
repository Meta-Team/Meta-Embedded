if (DEFINED BOARD_NAME)
    add_subdirectory(cmsis)
endif ()

#if (DEFINED BOARD_NAME)
#    set(gtest_disable_pthreads ON CACHE BOOL "Disable pthreads as STM32 doesn't support it")
#endif ()
#add_subdirectory(gtest)

set(STD_CPP OFF CACHE BOOL "Disable C++ standard library")
add_subdirectory(cpputest)

add_subdirectory(crc)