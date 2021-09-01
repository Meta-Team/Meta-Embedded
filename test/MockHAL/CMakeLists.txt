add_library(MockHAL
        mock_hal_pal.cpp)
target_include_directories(MockHAL BEFORE PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(MockHAL PRIVATE ${TEST_FRAMEWORK})