cmake_minimum_required(VERSION 3.5.0)
project(osqp)

find_package(osqp QUIET)

if (NOT ${osqp_FOUND})
  message(WARNING "No valid OSQP version found. Cloning OSQP 0.6.0 into build directory")

  include(ExternalProject)

  ExternalProject_Add(osqp
    GIT_REPOSITORY    https://github.com/oxfordcontrol/osqp
    GIT_TAG           v0.6.2
    SOURCE_DIR        ${CMAKE_BINARY_DIR}/../trajopt_ext/osqp-src
    BINARY_DIR        ${CMAKE_BINARY_DIR}/../trajopt_ext/osqp-build
    CMAKE_CACHE_ARGS
            -DCMAKE_INSTALL_PREFIX:STRING=${CMAKE_INSTALL_PREFIX}
            -DCMAKE_BUILD_TYPE:STRING=Release
  )
endif()

install(FILES package.xml DESTINATION share/osqp)
