cmake_minimum_required(VERSION 3.5.0)
project(trajopt_ifopt VERSION 0.1.0 LANGUAGES CXX)

find_package(trajopt_utils)
find_package(trajopt)
find_package(console_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ifopt REQUIRED)
find_package(tesseract REQUIRED)

list(FIND CMAKE_CXX_COMPILE_FEATURES cxx_std_14 CXX_FEATURE_FOUND)


###########
## Build ##
###########

set(TRAJOPT_IFOPT_SOURCE_FILES
    src/cartesian_position_constraint.cpp
    src/collision_constraint.cpp
    src/inverse_kinematics_constraint.cpp
    src/joint_position_constraint.cpp
    src/joint_velocity_constraint.cpp
    src/squared_cost.cpp
)

add_library(${PROJECT_NAME} SHARED ${TRAJOPT_IFOPT_SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} PUBLIC
  console_bridge
  ifopt::ifopt_core
  trajopt::trajopt_utils
  trajopt::trajopt
  tesseract::tesseract)
trajopt_target_compile_options(${PROJECT_NAME} PUBLIC)
trajopt_clang_tidy(${PROJECT_NAME})
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIRS})
list (APPEND PACKAGE_LIBRARIES ${PROJECT_NAME})

#############
## Install ##
#############

trajopt_configure_package(${PACKAGE_LIBRARIES})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
 )

#############
## Testing ##
#############

if (TRAJOPT_ENABLE_TESTING)
  enable_testing()
  add_custom_target(run_tests ALL
      WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
      COMMAND ${CMAKE_CTEST_COMMAND} -C $<CONFIGURATION> -V)

  add_subdirectory(test)
endif()
