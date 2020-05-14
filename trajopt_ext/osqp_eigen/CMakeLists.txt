cmake_minimum_required(VERSION 3.5.0)
project(OsqpEigen)

find_package(OsqpEigen QUIET)

if (NOT ${OsqpEigen_FOUND})
  message(WARNING "No valid OsqpEigen version found. Cloning into build directory")

  include(ExternalProject)

  ExternalProject_Add(osqp
    GIT_REPOSITORY    https://github.com/mpowelson/osqp-eigen
    GIT_TAG           feature/expose_workspace
    SOURCE_DIR        ${CMAKE_BINARY_DIR}/../trajopt_ext/osqp_eigen-src
    BINARY_DIR        ${CMAKE_BINARY_DIR}/../trajopt_ext/osqp_eigen-build
    CMAKE_CACHE_ARGS
            -DCMAKE_INSTALL_PREFIX:STRING=${CMAKE_INSTALL_PREFIX}
            -DCMAKE_BUILD_TYPE:STRING=Release
  )
endif()

install(FILES package.xml DESTINATION share/osqp_eigen)
