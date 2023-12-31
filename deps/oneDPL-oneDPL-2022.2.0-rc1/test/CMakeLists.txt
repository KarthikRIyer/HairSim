##===-- CMakeLists.txt ----------------------------------------------------===##
#
# Copyright (C) Intel Corporation
#
# SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
#
# This file incorporates work covered by the following copyright and permission
# notice:
#
# Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
# See https://llvm.org/LICENSE.txt for license information.
#
##===----------------------------------------------------------------------===##
# rng_tests
set (ranlux_24_48_test.pass_timeout_debug "900") # 15min
set (ranlux_24_48_test.pass_timeout_release "720") # 12min

# disable fast math for ALL TESTS 
# icpx and dpcpp accept -fno-fast-math, but icx-cl and dpcpp-cl only accept /clang:-fno-fast-math
foreach(_fno_fast_math_flag -fno-fast-math /clang:-fno-fast-math)
    string(MAKE_C_IDENTIFIER ${_fno_fast_math_flag} FLAG_DISPLAY_NAME)
    check_cxx_compiler_flag(${_fno_fast_math_flag} ${FLAG_DISPLAY_NAME}_option)
    if (${FLAG_DISPLAY_NAME}_option)
        target_compile_options(oneDPL INTERFACE ${_fno_fast_math_flag})
        set(_fno_fast_math_option ${_fno_fast_math_flag})
        break()
    endif()
endforeach()
if (_fno_fast_math_option)
    add_compile_options(${_fno_fast_math_option})
else()
    message(STATUS "oneDPL: -fno-fast-math is not supported by current compiler")
endif()

# usage of Kernel names in tests
set(ONEDPL_TEST_EXPLICIT_KERNEL_NAMES AUTO CACHE STRING "Use explicit Kernel names in tests")
set_property(CACHE ONEDPL_TEST_EXPLICIT_KERNEL_NAMES PROPERTY STRINGS AUTO ALWAYS)
message(STATUS "Usage of Kernel names in tests: ONEDPL_TEST_EXPLICIT_KERNEL_NAMES=${ONEDPL_TEST_EXPLICIT_KERNEL_NAMES}")
if (${ONEDPL_TEST_EXPLICIT_KERNEL_NAMES} STREQUAL AUTO)
    message(STATUS "    Usage of Kernel names in tests depend on unnamed lambda option")
    # State of TEST_EXPLICIT_KERNEL_NAMES will be definel later in C++ code
elseif(${ONEDPL_TEST_EXPLICIT_KERNEL_NAMES} STREQUAL ALWAYS)
    add_definitions(-DTEST_EXPLICIT_KERNEL_NAMES=1)
    message(STATUS "    Use Kernel names in tests :")
    message(STATUS "        #define TEST_EXPLICIT_KERNEL_NAMES 1")
else()
    message(FATAL_ERROR "Unsupported explicit Kernel names usage in tests: ${ONEDPL_TEST_EXPLICIT_KERNEL_NAMES}.\n"
            "Select one of the following values: AUTO, ALWAYS")
endif()

add_custom_target(build-onedpl-tests
    COMMENT "Build all oneDPL tests")

add_custom_target(run-onedpl-tests
    COMMAND "${CMAKE_CTEST_COMMAND}" --output-on-failure
    USES_TERMINAL
    DEPENDS build-onedpl-tests
    COMMENT "Build and run all oneDPL tests")

macro(onedpl_add_test test_source_file switch_off_checked_iterators)

    set(_build_type_in_lower "unknown")
    if (CMAKE_BUILD_TYPE)
        string(TOLOWER "${CMAKE_BUILD_TYPE}" _build_type_in_lower)
    endif()

    get_filename_component(_test_name ${test_source_file} NAME)
    string(REPLACE "\.cpp" "" _test_name ${_test_name})

    add_executable(${_test_name} EXCLUDE_FROM_ALL "${test_source_file}")
    target_compile_definitions(${_test_name} PRIVATE _PSTL_TEST_SUCCESSFUL_KEYWORD=1)
    if (MSVC)
        target_compile_options(${_test_name} PRIVATE /bigobj)
    endif()

    # Disable checked iterators on Windows for debug mode
    # For details please see
    #    https://learn.microsoft.com/en-us/cpp/standard-library/iterator-debug-level?view=msvc-170
    #    https://learn.microsoft.com/en-us/cpp/build/reference/md-mt-ld-use-run-time-library?view=msvc-170
    #    https://stackoverflow.com/questions/51494506/replacing-md-with-mt-in-cmake-is-not-possible-in-release-mode
    if (WIN32 AND ${_build_type_in_lower} STREQUAL "debug" AND ${switch_off_checked_iterators})
        target_compile_definitions(${_test_name} PRIVATE _ITERATOR_DEBUG_LEVEL=0)
        target_compile_options(${_test_name} PRIVATE "/MD$<$<CONFIG:Debug>:>")
    endif()

    # oneDPL test harness may initialize a C++ iterator using iterator with different type
    # that may break code when using Intel(R) C++ Compiler Classic with -O3 flag on Linux
    if (CMAKE_SYSTEM_NAME MATCHES "Linux" AND CMAKE_CXX_COMPILER_ID STREQUAL Intel)
        target_compile_options(${_test_name} PRIVATE $<$<CONFIG:Release>:-fno-strict-aliasing>)
    endif()

    target_include_directories(${_test_name} PRIVATE "${CMAKE_CURRENT_LIST_DIR}")
    target_link_libraries(${_test_name} PRIVATE oneDPL)
    set_target_properties(${_test_name} PROPERTIES CXX_EXTENSIONS NO)

    add_test(NAME ${_test_name} COMMAND ${CMAKE_CURRENT_BINARY_DIR}/${_test_name})

    if (DEFINED ${_test_name}_timeout_${_build_type_in_lower})
        message(STATUS
        "Timeout for ${_test_name} is set to ${${_test_name}_timeout_${_build_type_in_lower}}, "
        "it overrides user-defined timeout for CTest")
        set_tests_properties(${_test_name} PROPERTIES TIMEOUT ${${_test_name}_timeout_${_build_type_in_lower}})
    elseif (DEFINED ${_test_name}_timeout_release)
        # Apply default (release) timeout
        message(STATUS
        "Timeout for ${_test_name} is set to ${${_test_name}_timeout_release}, "
        "it overrides user-defined timeout for CTest")
        set_tests_properties(${_test_name} PROPERTIES TIMEOUT ${${_test_name}_timeout_release})
    endif()

    set_tests_properties(${_test_name} PROPERTIES SKIP_RETURN_CODE 77)

    if (DEFINED DEVICE_SELECTION_LINE)
        set_tests_properties(${_test_name} PROPERTIES ENVIRONMENT ${DEVICE_SELECTION_LINE})
    endif()

    add_custom_target(run-${_test_name}
        COMMAND "${CMAKE_CTEST_COMMAND}" -R ^${_test_name}$$ --output-on-failure --no-label-summary
        USES_TERMINAL
        DEPENDS ${_test_name}
        COMMENT "Build and run test ${_test_name}")

    # Add labels and group targets
    file(RELATIVE_PATH _test_rel_path ${CMAKE_CURRENT_SOURCE_DIR} ${test_source_file})
    get_filename_component(_test_rel_path ${_test_rel_path} DIRECTORY)
    if (_test_rel_path)
        string(REPLACE "/" ";" _test_labels ${_test_rel_path})
        set_tests_properties(${_test_name} PROPERTIES LABELS "${_test_labels}")
    endif()

    foreach (_label ${_test_labels})
        if (NOT TARGET build-onedpl-${_label}-tests)
            add_custom_target(build-onedpl-${_label}-tests COMMENT "Build tests with label ${_label}")

            add_custom_target(run-onedpl-${_label}-tests
                COMMAND "${CMAKE_CTEST_COMMAND}" -L ^${_label}$$ --output-on-failure --no-label-summary
                USES_TERMINAL
                DEPENDS build-onedpl-${_label}-tests
                COMMENT "Build and run tests with label ${_label}")
        endif()
        add_dependencies(build-onedpl-${_label}-tests ${_test_name})
    endforeach()
    add_dependencies(build-onedpl-tests ${_test_name})
endmacro()

set(_regexp_dpcpp_backend_required "(xpu_api/algorithms|xpu_api/functional|xpu_api/numerics)")
file(GLOB_RECURSE UNIT_TESTS "*.pass.cpp")
foreach (_file IN LISTS UNIT_TESTS)

    if (_file MATCHES "${_regexp_dpcpp_backend_required}")
        if (ONEDPL_BACKEND MATCHES "^(dpcpp|dpcpp_only)$")
            # Switch off checked iterators
            onedpl_add_test(${_file} true)
        else()
            message(STATUS "Skip test ${_file} as it requires DPC++ backend")
        endif()
    else()
        # Do not switch off checked iterators
        onedpl_add_test(${_file} false)
    endif()

endforeach()
