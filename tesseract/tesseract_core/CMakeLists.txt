cmake_minimum_required(VERSION 2.8.3)
project(tesseract_core)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES
  #CATKIN_DEPENDS
  DEPENDS
    EIGEN3
    Boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
