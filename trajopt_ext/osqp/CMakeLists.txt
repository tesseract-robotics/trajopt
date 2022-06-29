cmake_minimum_required(VERSION 3.5.0)
find_package(ros_industrial_cmake_boilerplate REQUIRED)

extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

find_package(osqp QUIET)

if(NOT ${osqp_FOUND})
  message(WARNING "No valid OSQP version found. Cloning OSQP 0.6.0 into build directory")

  include(ExternalProject)

  ExternalProject_Add(
    osqp
    GIT_REPOSITORY https://github.com/oxfordcontrol/osqp
    GIT_TAG v0.6.2
    SOURCE_DIR ${CMAKE_BINARY_DIR}-src
    BINARY_DIR ${CMAKE_BINARY_DIR}-build
    PATCH_COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_SOURCE_DIR}/patch_osqp.cmake
    CMAKE_CACHE_ARGS
      -DCMAKE_INSTALL_PREFIX:STRING=${CMAKE_INSTALL_PREFIX}
      -DCMAKE_BUILD_TYPE:STRING=Release
      -DCMAKE_WINDOWS_EXPORT_ALL_SYMBOLS:BOOL=ON
      -DCMAKE_TOOLCHAIN_FILE:STRING=${CMAKE_TOOLCHAIN_FILE}
      -DVCPKG_TARGET_TRIPLET:STRING=${VCPKG_TARGET_TRIPLET})

  if(TRAJOPT_PACKAGE)
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
      ${TRAJOPT_PACKAGE_PREFIX})
  endif()
endif()

install(FILES package.xml DESTINATION share/${PROJECT_NAME})
