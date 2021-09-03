if (NOT DEFINED BOARD_NAME)
    set(TEST_COVERAGE_ENABLED TRUE)
endif ()

include(cpputest.cmake)  # set ${TEST_FRAMEWORK}, ${TEST_MAIN_TARGET} and add_unit_test_to_test_framework()

if (TEST_COVERAGE_ENABLED)
    # The following two options works for both gcc and clang
    add_compile_options("--coverage")
    add_link_options("--coverage")

    include(${PROJECT_SOURCE_DIR}/config/CodeCoverage.cmake)
endif ()


# Add a unit test with one or more source files. The filename (extension removed) is used as the name of the test.
# add_unit_test(MainTestFile.cpp
#               OtherSourceFile.cpp
#               ALSO_ON_TARGET         # enable this test for the target board (BOARD_NAME defined)
#               WITH_HAL               # link against HAL (on target) or mock_hal (on development)
#)
function(add_unit_test)
    if (${ARGC} LESS 1)
        message(FATAL_ERROR "Usage: add_unit_test(<src> [<more src>...]), where the test is named as <src> (without .cpp)")
    endif ()

    set(options ALSO_ON_TARGET WITH_HAL)
    cmake_parse_arguments(Test "${options}" "" "" ${ARGN})
    set(src_list ${Test_UNPARSED_ARGUMENTS})

    if (DEFINED BOARD_NAME AND NOT Test_ALSO_ON_TARGET)
        if (VERBOSE)
            string(REPLACE ";" ", " src_list_output "${src_list}")
            message("Unit test ${test_name} is not added for target device")
        endif ()
        return()
    endif ()

    list(GET src_list 0 main_src)
    get_filename_component(test_name "${main_src}" NAME_WE)

    add_executable(${test_name} ${src_list} test_runner.cpp)
    target_include_directories(${test_name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})   # prioritize testing headers
    add_unit_test_to_test_framework(${test_name})

    if (VERBOSE)
        string(REPLACE ";" ", " src_list_output "${src_list}")
        message("Unit test ${test_name}: ${src_list_output}")
    endif ()

    if (Test_WITH_HAL)
        if (DEFINED BOARD_NAME)
            # TODO: [liuzikai] link real HAL?
        else ()
            target_link_libraries(${test_name} MockHAL)
        endif ()
    endif ()
endfunction()

if (NOT DEFINED BOARD_NAME)
    add_subdirectory(mock_hal)
endif ()

include_directories(${PROJECT_SOURCE_DIR}/include)

add_unit_test(hello_test.cpp
        ALSO_ON_TARGET)
add_unit_test(led_test.cpp
        ${PROJECT_SOURCE_DIR}/dev/interface/led.cpp
        WITH_HAL ALSO_ON_TARGET)


if (TEST_COVERAGE_ENABLED)
    setup_target_for_coverage_lcov(
            NAME ctest_coverage
            EXECUTABLE ctest -j ${PROCESSOR_COUNT}
            DEPENDENCIES ${TEST_MAIN_TARGET}
            EXCLUDE "${PROJECT_SOURCE_DIR}/lib/*"
    )
endif ()