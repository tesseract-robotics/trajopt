cmake_minimum_required(VERSION 2.8.3)
project(tesseract_ros_planning)

add_compile_options(-std=c++11 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tesseract_ros
  trajopt
)

find_package(OMPL REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
    ${OMPL_INCLUDE_DIRS}
  LIBRARIES
    tesseract_ros_trajopt_planner
  CATKIN_DEPENDS
    roscpp
    tesseract_ros
    trajopt
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  SYSTEM ${OMPL_INCLUDE_DIRS}
)

# Trajopt Planner
add_library(tesseract_ros_trajopt_planner
  src/trajopt/trajopt_planner.cpp
)

target_link_libraries(tesseract_ros_trajopt_planner
  ${catkin_LIBRARIES}
)

# OMPL Planning Interface
add_library(${PROJECT_NAME}_ompl
  src/ompl/chain_ompl_interface.cpp
  src/ompl/conversions.cpp
  src/ompl/continuous_motion_validator.cpp
)

add_dependencies(${PROJECT_NAME}_ompl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_ompl
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
)

# OMPL Planning Test/Example Program
add_executable(${PROJECT_NAME}_ompl_test
  src/ompl_planner_tests.cpp
)

add_dependencies(${PROJECT_NAME}_ompl_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_ompl_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_ompl
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_ompl
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
