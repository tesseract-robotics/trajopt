cmake_minimum_required(VERSION 2.8.3)
project(tesseract_ros)

add_compile_options(-std=c++11 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  tesseract_core
  tesseract_msgs
  tesseract_collision
  roscpp
  urdf
  srdfdom
  geometric_shapes
  kdl_parser
  eigen_conversions
  pluginlib
)

find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system thread REQUIRED)
find_package(octomap REQUIRED)
find_package(orocos_kdl REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    ${PROJECT_NAME}_kdl
    ${OCTOMAP_LIBRARIES}
    ${orocos_kdl_LIBRARIES}
  CATKIN_DEPENDS
    tesseract_core
    tesseract_msgs
    tesseract_collision
    roscpp
    urdf
    srdfdom
    geometric_shapes
    kdl_parser
    eigen_conversions
    pluginlib
  DEPENDS
    EIGEN3
    Boost
    OCTOMAP
    orocos_kdl
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  SYSTEM ${EIGEN3_INCLUDE_DIRS}
  SYSTEM ${Boost_INCLUDE_DIRS}
  SYSTEM ${OCTOMAP_INCLUDE_DIRS}
  SYSTEM ${orocos_kdl_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_kdl
  src/kdl/kdl_chain_kin.cpp
  src/kdl/kdl_env.cpp
)

target_link_libraries(${PROJECT_NAME}_kdl ${catkin_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${orocos_kdl_LIBRARIES})

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_kdl
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
 )
