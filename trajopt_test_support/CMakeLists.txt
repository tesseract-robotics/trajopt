cmake_minimum_required(VERSION 2.8.3)
project(trajopt_test_support)

find_package(catkin REQUIRED)

catkin_package()

include_directories()

install(
  DIRECTORY config launch meshes urdf srdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
