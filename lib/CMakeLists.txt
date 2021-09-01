if (DEFINED BOARD_NAME)
    add_subdirectory(cmsis)
endif ()

set(STD_CPP OFF CACHE BOOL "Disable C++ standard library")
set(COVERAGE ON CACHE BOOL "Disable C++ standard library")
set(TESTS OFF CACHE BOOL "Disable C++ standard library")
add_subdirectory(cpputest)

add_subdirectory(crc)