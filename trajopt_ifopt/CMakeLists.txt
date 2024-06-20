cmake_minimum_required(VERSION 3.10.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

if(WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

# Load variable for clang tidy args, compiler options and cxx version
find_package(trajopt_common REQUIRED)
trajopt_variables()

# ######################################################################################################################
# Build ##
# ######################################################################################################################

set(SUPPORTED_COMPONENTS ifopt)
add_subdirectory(ifopt)

# Simple
list(APPEND SUPPORTED_COMPONENTS trajopt)
add_subdirectory(trajopt)

# Package configuration
configure_package(
  COMPONENT
  ifopt
  SUPPORTED_COMPONENTS
  ${SUPPORTED_COMPONENTS})

if(TRAJOPT_PACKAGE)
  cpack_component_package(
    VERSION
    ${pkg_extracted_version}
    MAINTAINER_NAME
    ${pkg_extracted_maintainer_name}
    MAINTAINER_EMAIL
    ${pkg_extracted_maintainer_email}
    DESCRIPTION
    ${pkg_extracted_description}
    LICENSE_FILE
    ${CMAKE_CURRENT_LIST_DIR}/../LICENSE
    README_FILE
    ${CMAKE_CURRENT_LIST_DIR}/../README.md
    COMPONENT_DEPENDS
    ${SUPPORTED_COMPONENTS})
endif()
