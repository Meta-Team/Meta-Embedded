file(GLOB COMMON_FILES "dsp/src/CommonTables/*.c")
file(GLOB MATRIX_FP32_FILES "dsp/src/MatrixFunctions/*f32.c")
file(GLOB FAST_MATH_FP32_FILES "dsp/src/FastMathFunctions/*f32.c")
add_library(CMSIS_DSP OBJECT ${COMMON_FILES} ${MATRIX_FP32_FILES} ${FAST_MATH_FP32_FILES})
target_include_directories(CMSIS_DSP PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/dsp/include)
target_compile_definitions(CMSIS_DSP PUBLIC
        ARM_MATH_CM4
        ARM_MATH_MATRIX_CHECK
        ARM_MATH_ROUNDING
        __FPU_PRESENT=1U
        )
set_target_properties(CMSIS_DSP PROPERTIES LINKER_LANGUAGE C)