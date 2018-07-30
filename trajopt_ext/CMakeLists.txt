cmake_minimum_required(VERSION 2.8.3)
project(trajopt_ext)

add_compile_options(-std=c++11 -w)

find_package(catkin REQUIRED COMPONENTS)

catkin_package(
  INCLUDE_DIRS vhacd/public
  LIBRARIES HACD vhacd
)

set(CMAKE_COMMON_INC "${CMAKE_CURRENT_SOURCE_DIR}/vhacd/scripts/cmake_common.cmake")
add_subdirectory(vhacd)
