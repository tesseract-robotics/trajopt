cmake_minimum_required(VERSION 2.8.3)
project(trajopt_moveit)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  trajopt
  tesseract_ext
  urdf
  moveit_core
  moveit_ros_planning
  pluginlib
  tesseract_ros
)

find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system thread REQUIRED)
find_package(octomap REQUIRED)

set(BULLET_LIBRARIES BulletCollision LinearMath)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}
    ${OCTOMAP_LIBRARIES}
    moveit_collision_detection_bullet
  CATKIN_DEPENDS
    roscpp
    trajopt
    tesseract_ext
    urdf
    moveit_core
    moveit_ros_planning
    pluginlib
    tesseract_ros
  DEPENDS
    EIGEN3
    Boost
    OCTOMAP
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  SYSTEM ${EIGEN3_INCLUDE_DIRS}
  SYSTEM ${Boost_INCLUDE_DIRS}
  SYSTEM ${OCTOMAP_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/trajopt_moveit_env.cpp
)

target_link_libraries(${PROJECT_NAME} ${OCTOMAP_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${catkin_LIBRARIES})


add_library(moveit_collision_detection_bullet
  src/collision_common.cpp
  src/collision_robot_bullet.cpp
  src/collision_world_bullet.cpp
)

target_link_libraries(moveit_collision_detection_bullet ${catkin_LIBRARIES} ${console_bridge_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_library(moveit_collision_detection_bullet_plugin src/collision_detector_allocator_bullet.cpp)
target_link_libraries(moveit_collision_detection_bullet_plugin moveit_collision_detection_bullet ${catkin_LIBRARIES})

if (CATKIN_ENABLE_TESTING)

  find_package(rostest REQUIRED)
  find_package(catkin REQUIRED COMPONENTS octomap_ros trajopt_test_support roslib)
  find_package(PCL REQUIRED COMPONENTS core features filters io segmentation surface)

  include_directories(
    test
    ${PCL_INCLUDE_DIRS}
  )

  add_rostest_gtest(${PROJECT_NAME}_planning_unit test/planning_unit.launch test/planning_unit.cpp)
  target_link_libraries(${PROJECT_NAME}_planning_unit ${PROJECT_NAME} ${Boost_SYSTEM_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${catkin_LIBRARIES})

  add_rostest_gtest(${PROJECT_NAME}_cast_cost_unit test/cast_cost_unit.launch test/cast_cost_unit.cpp)
  target_link_libraries(${PROJECT_NAME}_cast_cost_unit ${PROJECT_NAME} ${Boost_SYSTEM_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${catkin_LIBRARIES})

  add_rostest_gtest(${PROJECT_NAME}_cast_cost_world_unit test/cast_cost_world_unit.launch test/cast_cost_world_unit.cpp)
  target_link_libraries(${PROJECT_NAME}_cast_cost_world_unit ${PROJECT_NAME} ${Boost_SYSTEM_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${catkin_LIBRARIES})

  add_rostest_gtest(${PROJECT_NAME}_cast_cost_attached_unit test/cast_cost_attached_unit.launch test/cast_cost_attached_unit.cpp)
  target_link_libraries(${PROJECT_NAME}_cast_cost_attached_unit ${PROJECT_NAME} ${Boost_SYSTEM_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${catkin_LIBRARIES})

  add_rostest_gtest(${PROJECT_NAME}_cast_cost_octomap_unit test/cast_cost_octomap_unit.launch test/cast_cost_octomap_unit.cpp)
  target_link_libraries(${PROJECT_NAME}_cast_cost_octomap_unit ${PROJECT_NAME} ${Boost_SYSTEM_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${PCL_LIBRARIES} ${catkin_LIBRARIES})
endif()
