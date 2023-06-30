#
# @file trajopt_macros.cmake @brief Common TrajOpt CMake Macros
#
# @author Levi Armstrong @date November 30, 2019 @version TODO @bug No known bugs
#
# @copyright Copyright (c) 2019, Southwest Research Institute
#
# @par License Software License Agreement (Apache License) @par Licensed under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the License. You may obtain a copy of the License at
# http://www.apache.org/licenses/LICENSE-2.0 @par Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
# express or implied. See the License for the specific language governing permissions and limitations under the License.

macro(trajopt_variables)
  if(NOT DEFINED BUILD_SHARED_LIBS)
    set(BUILD_SHARED_LIBS ON)
  endif()

  if(NOT DEFINED TRAJOPT_ENABLE_CLANG_TIDY)
    set(TRAJOPT_ENABLE_CLANG_TIDY OFF)
  endif()

  if(NOT DEFINED TRAJOPT_ENABLE_TESTING)
    set(TRAJOPT_ENABLE_TESTING OFF)
  endif()

  if(NOT DEFINED TRAJOPT_ENABLE_RUN_TESTING)
    set(TRAJOPT_ENABLE_RUN_TESTING OFF)
  endif()

  if(TRAJOPT_ENABLE_TESTING_ALL)
    set(TRAJOPT_ENABLE_TESTING ON)
    set(TRAJOPT_ENABLE_CLANG_TIDY ON)
  endif()

  set(TRAJOPT_COMPILE_DEFINITIONS "")
  set(TRAJOPT_COMPILE_OPTIONS_PRIVATE "")
  set(TRAJOPT_COMPILE_OPTIONS_PUBLIC "")
  if(NOT TRAJOPT_ENABLE_TESTING AND NOT TRAJOPT_ENABLE_TESTING_ALL)
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      set(TRAJOPT_COMPILE_OPTIONS_PRIVATE
          -Wall
          -Wextra
          -Wconversion
          -Wsign-conversion
          -Wno-sign-compare)
      set(TRAJOPT_COMPILE_OPTIONS_PUBLIC -mno-avx)
    elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
      set(TRAJOPT_COMPILE_OPTIONS_PRIVATE
          -Wall
          -Wextra
          -Wconversion
          -Wsign-conversion)
      message(WARNING "Non-GNU compiler detected. If using AVX instructions, Eigen alignment issues may result.")
    elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
      set(TRAJOPT_COMPILE_DEFINITIONS "_USE_MATH_DEFINES=ON")
      message(WARNING "Non-GNU compiler detected. If using AVX instructions, Eigen alignment issues may result.")
    else()
      message(WARNING "${CMAKE_CXX_COMPILER_ID} Unsupported compiler detected.")
    endif()
  else()
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      set(TRAJOPT_COMPILE_OPTIONS_PRIVATE
          -Werror=all
          -Werror=extra
          -Werror=conversion
          -Werror=sign-conversion
          -Wno-sign-compare)
      set(TRAJOPT_COMPILE_OPTIONS_PUBLIC -mno-avx)
    elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
      set(TRAJOPT_COMPILE_OPTIONS_PRIVATE
          -Werror=all
          -Werror=extra
          -Werror=conversion
          -Werror=sign-conversion)
      message(WARNING "Non-GNU compiler detected. If using AVX instructions, Eigen alignment issues may result.")
    elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
      set(TRAJOPT_COMPILE_DEFINITIONS "_USE_MATH_DEFINES=ON")
      message(WARNING "Non-GNU compiler detected. If using AVX instructions, Eigen alignment issues may result.")
    else()
      message(WARNING "${CMAKE_CXX_COMPILER_ID} Unsupported compiler detected.")
    endif()
  endif()

  set(TRAJOPT_CXX_VERSION 14)
endmacro()
