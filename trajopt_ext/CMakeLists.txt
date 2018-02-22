cmake_minimum_required(VERSION 2.8.3)
project(trajopt_ext)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS)

catkin_package(
  INCLUDE_DIRS ext/bullet/src
  LIBRARIES BulletCollision LinearMath
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

add_subdirectory(ext)
