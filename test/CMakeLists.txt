set(TEST_FRAMEWORK gtest_main)
include(GoogleTest)

function(add_unit_test)
    # Add a unit test with one or more source files
    # Arguments: list of src files, where the basename of the first file is used as the name of the test
    # Option: ON_TARGET (OFF by default) - compile the test on the target device

    if (${ARGC} LESS 1)
        message(FATAL_ERROR "Usage: add_unit_test(<src> [<more src>...]), where the test is named as <src> (without .cpp)")
    endif ()

    set(options ON_TARGET HAL)
    cmake_parse_arguments(TEST "${options}" "" "" ${ARGN})
    set(src_list ${TEST_UNPARSED_ARGUMENTS})

    list(GET src_list 0 main_src)
    get_filename_component(test_name "${main_src}" NAME_WE)

    if (NOT DEFINED BOARD_NAME OR TEST_ON_TARGET)
        add_executable(${test_name} ${src_list})
        target_link_libraries(${test_name} ${TEST_FRAMEWORK})
        gtest_discover_tests(${test_name})
        if (VERBOSE)
            string(REPLACE ";" ", " src_list_output "${src_list}")
            message("Unit test ${test_name}: ${src_list_output}")
        endif ()
    else ()
        if (VERBOSE)
            string(REPLACE ";" ", " src_list_output "${src_list}")
            message("Unit test ${test_name} is not added for target device")
        endif ()
    endif ()

    if (TEST_HAL)
        if (DEFINED BOARD_NAME)
            # TODO: [liuzikai] link real HAL?
        else ()
            target_link_libraries(${test_name} HALStubs)
        endif ()
    endif ()
endfunction()

include_directories(${PROJECT_SOURCE_DIR}/include)

if (NOT DEFINED BOARD_NAME)
    add_library(HALStubs
            HALStubs/hal_pal_stub.cpp)
    target_include_directories(HALStubs
            PUBLIC HALStubs)
    add_unit_test(HALStubs/hal_pal_stub_test.cpp HAL)
endif ()

add_unit_test(HelloTest.cpp ON_TARGET)
add_unit_test(LEDTest.cpp HAL ON_TARGET
        ${PROJECT_SOURCE_DIR}/src/LED.cpp)