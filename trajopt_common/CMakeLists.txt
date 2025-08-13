cmake_minimum_required(VERSION 3.10.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

include(cmake/trajopt_macros.cmake)

if(WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS program_options serialization REQUIRED)
find_package(ros_industrial_cmake_boilerplate REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(tesseract_collision COMPONENTS core REQUIRED)
find_package(tesseract_kinematics COMPONENTS core REQUIRED)

# Load variable for clang tidy args, compiler options and cxx version
trajopt_variables()

set(UTILS_SOURCE_FILES
    src/stl_to_string.cpp
    src/clock.cpp
    src/collision_types.cpp
    src/collision_utils.cpp
    src/logging.cpp
    src/utils.cpp)

add_library(${PROJECT_NAME} ${UTILS_SOURCE_FILES})
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC Eigen3::Eigen
         Boost::program_options
         Boost::serialization
         tesseract::tesseract_common
         tesseract::tesseract_collision_core
         tesseract::tesseract_kinematics_core)
target_compile_options(${PROJECT_NAME} PRIVATE ${TRAJOPT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME} PUBLIC ${TRAJOPT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME} PUBLIC ${TRAJOPT_COMPILE_DEFINITIONS}
                                                  TRAJOPT_DATA_DIR="${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/data")
target_cxx_version(${PROJECT_NAME} PUBLIC VERSION ${TRAJOPT_CXX_VERSION})
target_clang_tidy(${PROJECT_NAME} ENABLE ${TRAJOPT_ENABLE_CLANG_TIDY})
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                  "$<INSTALL_INTERFACE:include>")

configure_package(NAMESPACE trajopt TARGETS ${PROJECT_NAME})

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE)

if(TRAJOPT_ENABLE_TESTING)
  enable_testing()
  add_custom_target(run_tests)
  add_subdirectory(test)
endif()

install(FILES "${CMAKE_CURRENT_LIST_DIR}/cmake/trajopt_macros.cmake" DESTINATION lib/cmake/${PROJECT_NAME})

foreach(dir data)
  install(DIRECTORY ${dir}/ DESTINATION share/${PROJECT_NAME}/${dir})
endforeach()

if(TRAJOPT_PACKAGE)
  cpack(
    VERSION ${pkg_extracted_version} MAINTAINER <https://github.com/tesseract-robotics/trajopt>
    VENDOR "ROS-Industrial"
    DESCRIPTION ${pkg_extracted_description}
    LICENSE_FILE ""
    README_FILE ${CMAKE_CURRENT_LIST_DIR}/../README.md
    PACKAGE_PREFIX ${TRAJOPT_PACKAGE_PREFIX}
    LINUX_DEPENDS "libeigen3-dev" "libboost-program-options-dev" "libboost-serialization-dev"
    WINDOWS_DEPENDS "Eigen3" "boost-program-options" "boost_serialization")
endif()
