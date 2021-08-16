add_library(CRC OBJECT CRC8.cpp CRC16.cpp)
target_include_directories(CRC PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})