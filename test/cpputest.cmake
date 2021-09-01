set(TEST_FRAMEWORK CppUTest CppUTestExt)
set(TEST_MAIN_TARGET CTests)

# Reference: https://youtrack.jetbrains.com/issue/CPP-2470
add_executable(${TEST_MAIN_TARGET} ${PROJECT_SOURCE_DIR}/extra/null.cpp)

function(add_unit_test_to_test_framework)
    if (NOT ${ARGC} EQUAL 1)
        message(FATAL_ERROR "Usage: add_unit_test_to_test_framework(<test name>)")
    endif ()
    set(test_name ${ARGV0})

    target_link_libraries(${test_name} ${TEST_FRAMEWORK})
    add_test(${test_name} ${test_name})
    add_dependencies(${TEST_MAIN_TARGET} ${test_name})
endfunction()