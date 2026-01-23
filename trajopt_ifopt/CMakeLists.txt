cmake_minimum_required(VERSION 3.10.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES C CXX)

if(WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

find_package(trajopt_common REQUIRED)
find_package(console_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tesseract_environment REQUIRED)
find_package(Boost REQUIRED)

# Load variable for clang tidy args, compiler options and cxx version
trajopt_variables()

# ######################################################################################################################
# Build ##
# ######################################################################################################################

set(TRAJOPT_IFOPT_SOURCE_FILES
    src/core/bounds.cpp
    src/core/component.cpp
    src/core/composite.cpp
    src/core/constraint_set.cpp
    src/core/cost_term.cpp
    src/core/problem.cpp
    src/costs/absolute_cost.cpp
    src/costs/squared_cost.cpp
    src/constraints/cartesian_position_constraint.cpp
    src/constraints/inverse_kinematics_constraint.cpp
    src/constraints/joint_acceleration_constraint.cpp
    src/constraints/joint_jerk_constraint.cpp
    src/constraints/joint_position_constraint.cpp
    src/constraints/joint_velocity_constraint.cpp
    src/constraints/cartesian_line_constraint.cpp
    src/constraints/collision/discrete_collision_evaluators.cpp
    src/constraints/collision/continuous_collision_evaluators.cpp
    src/constraints/collision/weighted_average_methods.cpp
    src/constraints/collision/discrete_collision_constraint.cpp
    src/constraints/collision/discrete_collision_numerical_constraint.cpp
    src/constraints/collision/continuous_collision_constraint.cpp
    src/constraints/collision/continuous_collision_numerical_constraint.cpp
    src/utils/ifopt_utils.cpp
    src/utils/numeric_differentiation.cpp
    src/utils/trajopt_utils.cpp
    src/variable_sets/nodes_variables.cpp
    src/variable_sets/nodes_observer.cpp
    src/variable_sets/node.cpp
    src/variable_sets/var.cpp)

add_library(${PROJECT_NAME} ${TRAJOPT_IFOPT_SOURCE_FILES})
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC console_bridge::console_bridge
         trajopt::trajopt_common
         tesseract::tesseract_environment
         Eigen3::Eigen
         Boost::boost)
target_compile_options(${PROJECT_NAME} PRIVATE ${TRAJOPT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME} PUBLIC ${TRAJOPT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME} PUBLIC ${TRAJOPT_COMPILE_DEFINITIONS})
target_cxx_version(${PROJECT_NAME} PUBLIC VERSION ${TRAJOPT_CXX_VERSION})
target_clang_tidy(${PROJECT_NAME} ENABLE ${TRAJOPT_ENABLE_CLANG_TIDY})
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                  "$<INSTALL_INTERFACE:include>")

# ######################################################################################################################
# Install ##
# ######################################################################################################################

configure_package(
  NAMESPACE trajopt
  TARGETS ${PROJECT_NAME}
  DEPENDENCIES
    trajopt_common
    console_bridge
    Eigen3
    tesseract_environment
    Boost)

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE)

# ######################################################################################################################
# Testing ##
# ######################################################################################################################

if(TRAJOPT_ENABLE_TESTING)
  enable_testing()
  add_custom_target(run_tests)
  add_subdirectory(test)
endif()

if(TRAJOPT_PACKAGE)
  cpack(
    VERSION ${pkg_extracted_version} MAINTAINER <https://github.com/tesseract-robotics/trajopt>
    VENDOR "ROS-Industrial"
    DESCRIPTION ${pkg_extracted_description}
    README_FILE ${CMAKE_CURRENT_LIST_DIR}/../README.md
    PACKAGE_PREFIX ${TRAJOPT_PACKAGE_PREFIX}
    LINUX_DEPENDS
      "libeigen3-dev"
      "libboost-dev"
      "libconsole-bridge-dev"
      "${TRAJOPT_PACKAGE_PREFIX}trajopt-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-environment"
    WINDOWS_DEPENDS
      "Eigen3"
      "boost"
      "console-bridge"
      "${TRAJOPT_PACKAGE_PREFIX}trajopt-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-environment")
endif()
