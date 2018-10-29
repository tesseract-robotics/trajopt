cmake_minimum_required(VERSION 2.8.3)
project(trajopt_test_support)

find_package(catkin REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES trajopt_test_support
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

install(
  DIRECTORY config launch meshes urdf srdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
