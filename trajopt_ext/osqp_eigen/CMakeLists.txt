cmake_minimum_required(VERSION 3.5.0)
find_package(ros_industrial_cmake_boilerplate REQUIRED)

extract_package_metadata(pkg)
project(OsqpEigen_Ext VERSION ${pkg_extracted_version} LANGUAGES CXX)

set(TRAJOPT_OSQPEIGEN_VERSION "0.11.0")
# Remove patch version for find_package
string(
  REGEX
  REPLACE "\\.[0-9]+$"
          ""
          TRAJOPT_OSQPEIGEN_VERSION_MAJORMINOR
          ${TRAJOPT_OSQPEIGEN_VERSION})

find_package(OsqpEigen ${TRAJOPT_OSQPEIGEN_VERSION_MAJORMINOR} QUIET)

if(NOT ${OsqpEigen_FOUND} AND NOT TRAJOPT_OSQP_DISABLED)
  message(
    WARNING "No valid OsqpEigen version found. Cloning OsqpEigen ${TRAJOPT_OSQPEIGEN_VERSION} into build directory")

  include(ExternalProject)

  ExternalProject_Add(
    ${PROJECT_NAME}
    GIT_REPOSITORY https://github.com/robotology/osqp-eigen
    GIT_TAG v${TRAJOPT_OSQPEIGEN_VERSION}
    SOURCE_DIR ${CMAKE_BINARY_DIR}-src
    BINARY_DIR ${CMAKE_BINARY_DIR}-build
    CMAKE_CACHE_ARGS
      -DCMAKE_BUILD_TYPE:STRING=Release
      -DCMAKE_INSTALL_PREFIX:STRING=${CMAKE_INSTALL_PREFIX}
      -DCMAKE_TOOLCHAIN_FILE:STRING=${CMAKE_TOOLCHAIN_FILE}
      -DVCPKG_TARGET_TRIPLET:STRING=${VCPKG_TARGET_TRIPLET}
      -DOSQP_EIGEN_DEBUG_OUTPUT=OFF)

  if(TRAJOPT_PACKAGE)
    find_package(ros_industrial_cmake_boilerplate REQUIRED)
    set(CPACK_BUILD_SOURCE_DIRS "${CMAKE_BINARY_DIR}-src;${CMAKE_BINARY_DIR}-build")
    set(CPACK_INSTALL_CMAKE_PROJECTS "${CMAKE_BINARY_DIR}-build;${PROJECT_NAME};ALL;/")
    cpack(
      VERSION ${pkg_extracted_version} MAINTAINER <https://github.com/tesseract-robotics/trajopt>
      VENDOR "ROS-Industrial"
      DESCRIPTION ${pkg_extracted_description}
      PACKAGE_PREFIX ${TRAJOPT_PACKAGE_PREFIX}
      LINUX_DEPENDS "libeigen3-dev"
      WINDOWS_DEPENDS "Eigen3")
  endif()
endif()

install(FILES package.xml DESTINATION share/${PROJECT_NAME})
