cmake_minimum_required(VERSION 3.2.1...3.99)
find_package(ros_industrial_cmake_boilerplate REQUIRED)

extract_package_metadata(pkg)
project(qpoases VERSION ${pkg_extracted_version} LANGUAGES CXX)

find_package(qpOASES QUIET)

if(NOT qpOASES_FOUND AND NOT TRAJOPT_QPOASES_DISABLED)
  message(WARNING "No valid qpOASES version found. Cloning into build directory")

  if(MSVC)
    set(QPOASES_BUILD_SHARED_LIBS OFF CACHE BOOL "Build qpOASES shared libs")
  else()
    set(QPOASES_BUILD_SHARED_LIBS ON CACHE BOOL "Build qpOASES shared libs")
  endif()

  include(ExternalProject)

  ExternalProject_Add(
    ${PROJECT_NAME}
    GIT_REPOSITORY "https://github.com/coin-or/qpOASES.git"
    GIT_TAG f23588b59762f45ca2fef4b5fc861363f86391e5
    SOURCE_DIR ${CMAKE_BINARY_DIR}-src
    BINARY_DIR ${CMAKE_BINARY_DIR}-build
    CMAKE_CACHE_ARGS
      -DCMAKE_INSTALL_PREFIX:STRING=${CMAKE_INSTALL_PREFIX}
      -DCMAKE_BUILD_TYPE:STRING=Release
      -DCMAKE_TOOLCHAIN_FILE:STRING=${CMAKE_TOOLCHAIN_FILE}
      -DVCPKG_TARGET_TRIPLET:STRING=${VCPKG_TARGET_TRIPLET}
      -DBUILD_SHARED_LIBS:BOOL=${QPOASES_BUILD_SHARED_LIBS}
      -DQPOASES_AVOID_LA_NAMING_CONFLICTS:BOOL=ON
      -DQPOASES_BUILD_EXAMPLES:BOOL=OFF)

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
