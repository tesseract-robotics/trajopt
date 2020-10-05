#
# @file trajopt_macros.cmake
# @brief Common TrajOpt CMake Macros
#
# @author Levi Armstrong
# @date November 30, 2019
# @version TODO
# @bug No known bugs
#
# @copyright Copyright (c) 2019, Southwest Research Institute
#
# @par License
# Software License Agreement (Apache License)
# @par
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# http://www.apache.org/licenses/LICENSE-2.0
# @par
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


# This macro will add tesseract standard compiler option to a target
# Usage: trajopt_target_compile_options(target <INTERFACE|PUBLIC|PRIVATE>)
#    * c++14
#    * Warning (-Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
#    * disable avx due to eigen issues
#    * Add Clang Tidy
macro(trajopt_target_compile_options target)
  cmake_parse_arguments(ARG "INTERFACE;PUBLIC;PRIVATE" "" "" ${ARGN})

  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "trajopt_target_compile_options() called with unused arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif()

  list(FIND CMAKE_CXX_COMPILE_FEATURES cxx_std_14 CXX_FEATURE_FOUND)
  if (NOT TRAJOPT_ENABLE_TESTING)
    set(warning_flags -Wall -Wextra -Wconversion -Wsign-conversion -Wno-sign-compare)
  else()
    set(warning_flags -Werror=all -Werror=extra -Werror=conversion -Werror=sign-conversion -Wno-sign-compare)
  endif()

  if (ARG_INTERFACE)
    if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
      if(CXX_FEATURE_FOUND EQUAL "-1")
        target_compile_options("${target}" INTERFACE -std=c++14 -mno-avx ${warning_flags})
      else()
        target_compile_features("${target}" INTERFACE cxx_std_14)
        target_compile_options("${target}" INTERFACE -mno-avx ${warning_flags})
      endif()
    else()
      message(WARNING "Non-GNU compiler detected. If using AVX instructions, Eigen alignment issues may result.")
    endif()
  elseif(ARG_PUBLIC)
    target_compile_options("${target}" PRIVATE ${warning_flags})

    if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
      if(CXX_FEATURE_FOUND EQUAL "-1")
        target_compile_options("${target}" PUBLIC -std=c++14 -mno-avx)
      else()
        target_compile_features("${target}" PUBLIC cxx_std_14)
        target_compile_options("${target}" PUBLIC -mno-avx)
      endif()
    else()
      message(WARNING "Non-GNU compiler detected. If using AVX instructions, Eigen alignment issues may result.")
    endif()
  elseif(ARG_PRIVATE)
    if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
      if(CXX_FEATURE_FOUND EQUAL "-1")
        target_compile_options("${target}" PRIVATE -std=c++14 -mno-avx ${warning_flags})
      else()
        target_compile_features("${target}" PRIVATE cxx_std_14)
        target_compile_options("${target}" PRIVATE -mno-avx ${warning_flags})
      endif()
    else()
      message(WARNING "Non-GNU compiler detected. If using AVX instructions, Eigen alignment issues may result.")
    endif()
  endif()
endmacro()

# Add clang-tidy to a target if TRAJOPT_ENABLE_CLANG_TIDY or TRAJOPT_ENABLE_TESTING is enabled
# Usage: tesseract_clang_tidy(Target) or tesseract_clang_tidy(Target true) or tesseract_clang_tidy(Target false)
#    * tesseract_clang_tidy(Target) adds clang tidy with warnings as errors
#    * tesseract_clang_tidy(Target true) adds clang tidy with warnings as errors
#    * tesseract_clang_tidy(Target false) adds clang tidy with warnings as warnings
macro(trajopt_clang_tidy target)
  cmake_parse_arguments(ARG "true;false" "" "" ${ARGN})

  get_target_property(${target}_type ${target} TYPE)

  # Add clang tidy
  if (NOT ${${target}_type} STREQUAL "INTERFACE_LIBRARY")
    if (TRAJOPT_ENABLE_CLANG_TIDY)
      find_program(CLANG_TIDY_EXE NAMES "clang-tidy" DOC "Path to clang-tidy executable")
      if(NOT CLANG_TIDY_EXE)
        message(WARNING "clang-tidy not found.")
      else()
        message(STATUS "clang-tidy found: ${CLANG_TIDY_EXE}")
        if(ARG_false)
          set(DO_CLANG_TIDY "${CLANG_TIDY_EXE}" "-header-filter=.*" "-checks=-*,bugprone-*,cppcoreguidelines-avoid-goto,cppcoreguidelines-c-copy-assignment-signature,cppcoreguidelines-interfaces-global-init,cppcoreguidelines-narrowing-conversions,cppcoreguidelines-no-malloc,cppcoreguidelines-slicing,cppcoreguidelines-special-member-functions,misc-*,-misc-non-private-member-variables-in-classes*,modernize-*,-modernize-use-trailing-return-type*,performance-*,readability-avoid-const-params-in-decls,readability-container-size-empty,readability-delete-null-pointer,readability-deleted-default,readability-else-after-return,readability-function-size,readability-identifier-naming,readability-inconsistent-declaration-parameter-name,readability-misleading-indentation,readability-misplaced-array-index,readability-non-const-parameter,readability-redundant-*,readability-simplify-*,readability-static-*,readability-string-compare,readability-uniqueptr-delete-release,readability-rary-objects")
        else()
          set(DO_CLANG_TIDY "${CLANG_TIDY_EXE}" "-header-filter=.*" "-checks=-*,bugprone-*,cppcoreguidelines-avoid-goto,cppcoreguidelines-c-copy-assignment-signature,cppcoreguidelines-interfaces-global-init,cppcoreguidelines-narrowing-conversions,cppcoreguidelines-no-malloc,cppcoreguidelines-slicing,cppcoreguidelines-special-member-functions,misc-*,-misc-non-private-member-variables-in-classes*,modernize-*,-modernize-use-trailing-return-type*,performance-*,readability-avoid-const-params-in-decls,readability-container-size-empty,readability-delete-null-pointer,readability-deleted-default,readability-else-after-return,readability-function-size,readability-identifier-naming,readability-inconsistent-declaration-parameter-name,readability-misleading-indentation,readability-misplaced-array-index,readability-non-const-parameter,readability-redundant-*,readability-simplify-*,readability-static-*,readability-string-compare,readability-uniqueptr-delete-release,readability-rary-objects" "-warnings-as-errors=-*,bugprone-*,cppcoreguidelines-avoid-goto,cppcoreguidelines-c-copy-assignment-signature,cppcoreguidelines-interfaces-global-init,cppcoreguidelines-narrowing-conversions,cppcoreguidelines-no-malloc,cppcoreguidelines-slicing,cppcoreguidelines-special-member-functions,misc-*,modernize-*,performance-*,readability-avoid-const-params-in-decls,readability-container-size-empty,readability-delete-null-pointer,readability-deleted-default,readability-else-after-return,readability-function-size,readability-identifier-naming,readability-inconsistent-declaration-parameter-name,readability-misleading-indentation,readability-misplaced-array-index,readability-non-const-parameter,readability-redundant-*,readability-simplify-*,readability-static-*,readability-string-compare,readability-uniqueptr-delete-release,readability-rary-objects")
        endif()
        set_target_properties("${target}" PROPERTIES CXX_CLANG_TIDY "${DO_CLANG_TIDY}")
      endif()
    endif()
  endif()
endmacro()

# Performs multiple operation so other packages may find a package
# Usage: trajopt_configure_package(targetA targetb)
#   * It installs the provided targets
#   * It exports the provided targets under the namespace tesseract::
#   * It installs the package.xml file
#   * It create and install the ${PROJECT_NAME}-config.cmake and ${PROJECT_NAME}-config-version.cmake
macro(trajopt_configure_package)
  install(TARGETS ${ARGV}
          EXPORT ${PROJECT_NAME}-targets
          RUNTIME DESTINATION bin
          LIBRARY DESTINATION lib
          ARCHIVE DESTINATION lib)
  install(EXPORT ${PROJECT_NAME}-targets NAMESPACE trajopt:: DESTINATION lib/cmake/${PROJECT_NAME})

  install(FILES package.xml DESTINATION share/${PROJECT_NAME})

  # Create cmake config files
  include(CMakePackageConfigHelpers)
  configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/cmake/${PROJECT_NAME}-config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
    INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
    NO_CHECK_REQUIRED_COMPONENTS_MACRO)

  write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
    VERSION ${PROJECT_VERSION} COMPATIBILITY ExactVersion)

  install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
    DESTINATION lib/cmake/${PROJECT_NAME})

  export(EXPORT ${PROJECT_NAME}-targets NAMESPACE trajopt:: FILE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-targets.cmake)

  # Allows Colcon to find non-Ament packages when using workspace underlays
  file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/share/ament_index/resource_index/packages/${PROJECT_NAME} "")
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/share/ament_index/resource_index/packages/${PROJECT_NAME} DESTINATION share/ament_index/resource_index/packages)
  file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/ament_prefix_path.dsv "prepend-non-duplicate;AMENT_PREFIX_PATH;")
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/ament_prefix_path.dsv DESTINATION share/${PROJECT_NAME}/hook)
  file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/ros_package_path.dsv "prepend-non-duplicate;ROS_PACKAGE_PATH;")
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/ros_package_path.dsv DESTINATION share/${PROJECT_NAME}/hook)
endmacro()

# This macro call the appropriate gtest function to add a test based on the cmake version
# Usage: tesseract_gtest_discover_tests(target)
macro(trajopt_gtest_discover_tests target)
  if(${CMAKE_VERSION} VERSION_LESS "3.10.0")
    gtest_add_tests(${target} "" AUTO)
  else()
    gtest_discover_tests(${target})
  endif()
endmacro()

# This macro add a custom target that will run the tests after they are finished building.
# This is added to allow ability do disable the running of tests as part of the build for CI which calls make test
macro(trajopt_add_run_tests_target)
  if(TRAJOPT_ENABLE_RUN_TESTING)
    add_custom_target(run_tests ALL
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        COMMAND ${CMAKE_CTEST_COMMAND} -V -O "/tmp/${PROJECT_NAME}_ctest.log" -C $<CONFIGURATION>)
  else()
    add_custom_target(run_tests)
  endif()
endmacro()

# This macro add a custom target that will run the benchmarks after they are finished building.
# Usage: trajopt_add_run_benchmark_target(benchmark_name)
# Results are saved to /test/benchmarks/${benchmark_name}_results.json in the build directory
macro(trajopt_add_run_benchmark_target benchmark_name)
  if(TRAJOPT_ENABLE_RUN_BENCHMARKING)
    add_custom_target(run_benchmark_${benchmark_name} ALL
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        COMMAND ./test/benchmarks/${benchmark_name} --benchmark_out_format=json --benchmark_out="./test/benchmarks/${benchmark_name}_results.json")
  else()
    add_custom_target(run_benchmark_${benchmark_name})
  endif()
  add_dependencies(run_benchmark_${benchmark_name} ${benchmark_name})
endmacro()
