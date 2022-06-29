cmake_minimum_required(VERSION 3.5.0)
find_package(ros_industrial_cmake_boilerplate REQUIRED)

extract_package_metadata(pkg)
project(OsqpEigen VERSION ${pkg_extracted_version} LANGUAGES CXX)

find_package(OsqpEigen QUIET)

if(NOT ${OsqpEigen_FOUND})
  message(WARNING "No valid OsqpEigen version found. Cloning into build directory")

  include(ExternalProject)

  ExternalProject_Add(
    ${PROJECT_NAME}
    GIT_REPOSITORY https://github.com/robotology/osqp-eigen
    GIT_TAG v0.6.3
    SOURCE_DIR ${CMAKE_BINARY_DIR}-src
    BINARY_DIR ${CMAKE_BINARY_DIR}-build
    CMAKE_CACHE_ARGS
      -DCMAKE_INSTALL_PREFIX:STRING=${CMAKE_INSTALL_PREFIX}
      -DCMAKE_BUILD_TYPE:STRING=Release
      -DCMAKE_TOOLCHAIN_FILE:STRING=${CMAKE_TOOLCHAIN_FILE}
      -DVCPKG_TARGET_TRIPLET:STRING=${VCPKG_TARGET_TRIPLET})

  if(TRAJOPT_PACKAGE)
    find_package(ros_industrial_cmake_boilerplate REQUIRED)
    set(CPACK_BUILD_SOURCE_DIRS "${CMAKE_BINARY_DIR}-src;${CMAKE_BINARY_DIR}-build")
    set(CPACK_INSTALL_CMAKE_PROJECTS "${CMAKE_BINARY_DIR}-build;${PROJECT_NAME};ALL;/")
    cpack(
      VERSION
      ${pkg_extracted_version}
      MAINTAINER
      <https://github.com/tesseract-robotics/trajopt>
      VENDOR
      "ROS-Industrial"
      DESCRIPTION
      ${pkg_extracted_description}
      PACKAGE_PREFIX
      ${TRAJOPT_PACKAGE_PREFIX}
      LINUX_DEPENDS
      "libeigen3-dev"
      WINDOWS_DEPENDS
      "Eigen3")
  endif()
endif()

install(FILES package.xml DESTINATION share/${PROJECT_NAME})
