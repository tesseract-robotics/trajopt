cmake_minimum_required(VERSION 2.8.3)
project(trajopt_ext)

add_compile_options(-std=c++11 -w)

find_package(catkin REQUIRED COMPONENTS)

set(BUILD_PYBULLET, ON)
set(BUILD_PYBULLET_NUMPY, OFF)
set(USE_DOUBLE_PRECISION, ON)

catkin_package(
  INCLUDE_DIRS bullet3/src bullet3/Extras vhacd/public
  LIBRARIES BulletCollision BulletDynamics LinearMath ConvexDecomposition HACD vhacd
)

add_subdirectory(bullet3)

set(CMAKE_COMMON_INC "${CMAKE_SOURCE_DIR}/vhacd/scripts/cmake_common.cmake")
add_subdirectory(vhacd)
