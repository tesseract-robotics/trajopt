find_package(console_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ifopt REQUIRED)
find_package(tesseract_environment REQUIRED)
find_package(ros_industrial_cmake_boilerplate REQUIRED)
find_package(Boost REQUIRED)

if(NOT TARGET console_bridge::console_bridge)
  add_library(console_bridge::console_bridge INTERFACE IMPORTED)
  set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
                                                                  ${console_bridge_INCLUDE_DIRS})
  set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_LINK_LIBRARIES ${console_bridge_LIBRARIES})
else()
  get_target_property(CHECK_INCLUDE_DIRECTORIES console_bridge::console_bridge INTERFACE_INCLUDE_DIRECTORIES)
  if(NOT ${CHECK_INCLUDE_DIRECTORIES})
    set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
                                                                    ${console_bridge_INCLUDE_DIRS})
  endif()
endif()

# ######################################################################################################################
# Build ##
# ######################################################################################################################

set(TRAJOPT_IFOPT_SOURCE_FILES
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
    src/variable_sets/joint_position_variable.cpp
    src/variable_sets/node.cpp
    src/variable_sets/nodes_variables.cpp
    src/variable_sets/var.cpp)

add_library(${PROJECT_NAME}_trajopt ${TRAJOPT_IFOPT_SOURCE_FILES})
target_link_libraries(
  ${PROJECT_NAME}_trajopt
  PUBLIC console_bridge::console_bridge
         ifopt::ifopt_core
         trajopt::trajopt_common
         tesseract::tesseract_environment
         Eigen3::Eigen
         Boost::boost)
target_compile_options(${PROJECT_NAME}_trajopt PRIVATE ${TRAJOPT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_trajopt PUBLIC ${TRAJOPT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_trajopt PUBLIC ${TRAJOPT_COMPILE_DEFINITIONS})
target_cxx_version(${PROJECT_NAME}_trajopt PUBLIC VERSION ${TRAJOPT_CXX_VERSION})
target_clang_tidy(${PROJECT_NAME}_trajopt ENABLE ${TRAJOPT_ENABLE_CLANG_TIDY})
target_include_directories(${PROJECT_NAME}_trajopt PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                          "$<INSTALL_INTERFACE:include>")

# Configure Components
configure_component(
  COMPONENT
  trajopt
  NAMESPACE
  trajopt
  TARGETS
  ${PROJECT_NAME}_trajopt
  DEPENDENCIES
  trajopt_common
  console_bridge
  Eigen3
  ifopt
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
  cpack_component(
    COMPONENT
    trajopt
    VERSION
    ${pkg_extracted_version}
    DESCRIPTION
    "TrajOpt Ifopt trajopt components"
    # COMPONENT_DEPENDS ifopt
    LINUX_DEPENDS
    "libeigen3-dev"
    "libboost-dev"
    "libconsole-bridge-dev"
    "ifopt"
    "${TRAJOPT_PACKAGE_PREFIX}trajopt-utils"
    "${TESSERACT_PACKAGE_PREFIX}tesseract-environment"
    WINDOWS_DEPENDS
    "Eigen3"
    "boost"
    "console-bridge"
    "ifopt"
    "${TRAJOPT_PACKAGE_PREFIX}trajopt-utils"
    "${TESSERACT_PACKAGE_PREFIX}tesseract-environment")
endif()
