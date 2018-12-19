cmake_minimum_required(VERSION 2.8.3)
project(trajopt_tools)

add_compile_options(-std=c++11 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  vhacd_ros
  bullet3_ros
  trajopt_utils
)

catkin_package(
#  INCLUDE_DIRS
#  LIBRARIES
  CATKIN_DEPENDS roscpp vhacd_ros bullet3_ros trajopt_utils
#  DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(convex_decomposition_hacd_node src/convex_decomposition_hacd.cpp)
target_link_libraries(convex_decomposition_hacd_node ${catkin_LIBRARIES})
target_compile_options(convex_decomposition_hacd_node PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

add_executable(convex_decomposition_vhacd_node src/convex_decomposition_vhacd.cpp)
target_link_libraries(convex_decomposition_vhacd_node ${catkin_LIBRARIES})
target_compile_options(convex_decomposition_vhacd_node PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

# Mark executables and/or libraries for installation
install(TARGETS convex_decomposition_hacd_node convex_decomposition_vhacd_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
