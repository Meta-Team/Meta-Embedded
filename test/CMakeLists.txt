set(TEST_FRAMEWORK gtest_main)
include(GoogleTest)

add_executable(
        HelloTest
        HelloTest.cpp
)
target_link_libraries(
        HelloTest
        ${TEST_FRAMEWORK}
)
gtest_discover_tests(HelloTest)