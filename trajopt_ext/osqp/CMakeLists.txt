cmake_minimum_required(VERSION 3.5.0)
project(osqp)

find_package(osqp QUIET)

if(NOT ${osqp_FOUND})
  message(WARNING "No valid OSQP version found. Cloning OSQP 0.6.0 into build directory")

  include(ExternalProject)

  ExternalProject_Add(
    osqp
    GIT_REPOSITORY https://github.com/oxfordcontrol/osqp
    GIT_TAG v0.6.2
    SOURCE_DIR ${CMAKE_BINARY_DIR}/../trajopt_ext/osqp-src
    BINARY_DIR ${CMAKE_BINARY_DIR}/../trajopt_ext/osqp-build
    PATCH_COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_SOURCE_DIR}/patch_osqp.cmake
    CMAKE_CACHE_ARGS
      -DCMAKE_INSTALL_PREFIX:STRING=${CMAKE_INSTALL_PREFIX}
      -DCMAKE_BUILD_TYPE:STRING=Release
      -DCMAKE_WINDOWS_EXPORT_ALL_SYMBOLS:BOOL=ON
      -DCMAKE_TOOLCHAIN_FILE:STRING=${CMAKE_TOOLCHAIN_FILE}
      -DVCPKG_TARGET_TRIPLET:STRING=${VCPKG_TARGET_TRIPLET})
endif()

install(FILES package.xml DESTINATION share/osqp)
