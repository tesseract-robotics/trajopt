cmake_minimum_required(VERSION 2.8.3)
project(trajopt_tools)

add_compile_options(-std=c++11 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  trajopt_ext
  bullet3_ros
)

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

catkin_package(
#  INCLUDE_DIRS
#  LIBRARIES
  CATKIN_DEPENDS roscpp trajopt_ext bullet3_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(convex_decomposition_hacd_node src/convex_decomposition_hacd.cpp)
target_link_libraries(convex_decomposition_hacd_node ${catkin_LIBRARIES})

add_executable(convex_decomposition_vhacd_node src/convex_decomposition_vhacd.cpp)
target_link_libraries(convex_decomposition_vhacd_node ${catkin_LIBRARIES})
