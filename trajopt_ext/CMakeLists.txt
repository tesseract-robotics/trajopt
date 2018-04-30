cmake_minimum_required(VERSION 2.8.3)
project(trajopt_ext)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS)

catkin_package(
  INCLUDE_DIRS bullet3/src bullet3/Extras vhacd/public
  LIBRARIES BulletCollision BulletDynamics LinearMath ConvexDecomposition HACD vhacd
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

add_subdirectory(bullet3)

set(CMAKE_COMMON_INC "${CMAKE_SOURCE_DIR}/vhacd/scripts/cmake_common.cmake")
add_subdirectory(vhacd)
