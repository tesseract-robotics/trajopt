cmake_minimum_required(VERSION 2.8.3)
project(tesseract_ros)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  tesseract_core
  tesseract_msgs
  trajopt_ext
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

set(BULLET_LIBRARIES BulletCollision LinearMath)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    ${PROJECT_NAME}_bullet
    ${PROJECT_NAME}_kdl
    ${OCTOMAP_LIBRARIES}
    ${orocos_kdl_LIBRARIES}
  CATKIN_DEPENDS
    tesseract_core
    tesseract_msgs
    trajopt_ext
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
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_kdl
  src/kdl/kdl_chain_kin.cpp
)

target_link_libraries(${PROJECT_NAME}_kdl ${catkin_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${Boost_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_library(${PROJECT_NAME}_bullet
  src/bullet/bullet_env.cpp
  src/bullet/bullet_env_singleton.cpp
  src/bullet/bullet_utils.cpp
)

target_link_libraries(${PROJECT_NAME}_bullet ${PROJECT_NAME}_kdl ${catkin_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_library(${PROJECT_NAME}_bullet_plugin src/plugins/bullet_env_plugin.cpp)
target_link_libraries(${PROJECT_NAME}_bullet_plugin ${PROJECT_NAME}_bullet ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}_bullet_singleton_plugin src/plugins/bullet_env_singleton_plugin.cpp)
target_link_libraries(${PROJECT_NAME}_bullet_singleton_plugin ${PROJECT_NAME}_bullet ${catkin_LIBRARIES})

# Tesseract ROS Nodes
add_executable(${PROJECT_NAME}_contact_monitoring_node src/contact_monitoring.cpp)
target_link_libraries(${PROJECT_NAME}_contact_monitoring_node ${catkin_LIBRARIES})

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_bullet ${PROJECT_NAME}_kdl
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

#if (CATKIN_ENABLE_TESTING)

#  add_definitions(-DDATA_DIR="${CMAKE_SOURCE_DIR}/data")
#  find_package(rostest REQUIRED)
#  find_package(catkin REQUIRED COMPONENTS octomap_ros)
#  find_package(PCL REQUIRED COMPONENTS core features filters io segmentation surface)

#  include_directories(
#    test
#    ${PCL_INCLUDE_DIRS}
#  )

#  add_rostest_gtest(${PROJECT_NAME}_planning_unit test/planning_unit.launch test/planning_unit.cpp)
#  target_link_libraries(${PROJECT_NAME}_planning_unit ${PROJECT_NAME} ${Boost_SYSTEM_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${catkin_LIBRARIES})

#  add_rostest_gtest(${PROJECT_NAME}_cast_cost_unit test/cast_cost_unit.launch test/cast_cost_unit.cpp)
#  target_link_libraries(${PROJECT_NAME}_cast_cost_unit ${PROJECT_NAME} ${Boost_SYSTEM_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${catkin_LIBRARIES})

#  add_rostest_gtest(${PROJECT_NAME}_cast_cost_world_unit test/cast_cost_world_unit.launch test/cast_cost_world_unit.cpp)
#  target_link_libraries(${PROJECT_NAME}_cast_cost_world_unit ${PROJECT_NAME} ${Boost_SYSTEM_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${catkin_LIBRARIES})

#  add_rostest_gtest(${PROJECT_NAME}_cast_cost_attached_unit test/cast_cost_attached_unit.launch test/cast_cost_attached_unit.cpp)
#  target_link_libraries(${PROJECT_NAME}_cast_cost_attached_unit ${PROJECT_NAME} ${Boost_SYSTEM_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${catkin_LIBRARIES})

#  add_rostest_gtest(${PROJECT_NAME}_cast_cost_octomap_unit test/cast_cost_octomap_unit.launch test/cast_cost_octomap_unit.cpp)
#  target_link_libraries(${PROJECT_NAME}_cast_cost_octomap_unit ${PROJECT_NAME} ${Boost_SYSTEM_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${PCL_LIBRARIES} ${catkin_LIBRARIES})
#endif()

