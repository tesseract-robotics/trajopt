cmake_minimum_required(VERSION 3.5.0)
find_package(ros_industrial_cmake_boilerplate REQUIRED)

extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

set(TRAJOPT_PIQP_VERSION "0.6.4")

# PIQP's package version file only accepts an exact match, so do not request a version
find_package(piqp QUIET)

if(NOT piqp_FOUND AND NOT TRAJOPT_PIQP_DISABLED)
  message(WARNING "No valid PIQP version found. Cloning PIQP ${TRAJOPT_PIQP_VERSION} into build directory")

  # PIQP does not install its DLL, so link it statically on Windows
  if(WIN32)
    set(PIQP_BUILD_SHARED_LIBS OFF)
  else()
    set(PIQP_BUILD_SHARED_LIBS ON)
  endif()

  include(ExternalProject)

  ExternalProject_Add(
    piqp
    GIT_REPOSITORY https://github.com/PREDICT-EPFL/piqp
    GIT_TAG v${TRAJOPT_PIQP_VERSION}
    GIT_SHALLOW TRUE
    SOURCE_DIR ${CMAKE_BINARY_DIR}-src
    BINARY_DIR ${CMAKE_BINARY_DIR}-build
    CMAKE_CACHE_ARGS
      -DCMAKE_INSTALL_PREFIX:STRING=${CMAKE_INSTALL_PREFIX}
      -DCMAKE_BUILD_TYPE:STRING=Release
      -DCMAKE_TOOLCHAIN_FILE:STRING=${CMAKE_TOOLCHAIN_FILE}
      -DVCPKG_TARGET_TRIPLET:STRING=${VCPKG_TARGET_TRIPLET}
      -DBUILD_SHARED_LIBS:BOOL=${PIQP_BUILD_SHARED_LIBS}
      # PIQP's tests and examples need Matio and fetch googletest
      -DBUILD_TESTS:BOOL=OFF
      -DBUILD_EXAMPLES:BOOL=OFF
      # trajopt links only the C++ library
      -DBUILD_C_INTERFACE:BOOL=OFF)

  if(TRAJOPT_PACKAGE)
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
