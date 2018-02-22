cmake_minimum_required(VERSION 2.8.3)
project(trajopt)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  trajopt_sco
  trajopt_utils
  trajopt_ext
  roscpp
  urdf
  moveit_core
  moveit_ros_planning
  pluginlib
)

find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system python thread program_options REQUIRED)
find_package(octomap REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)

link_directories(
    /usr/local/lib
    /usr/local/lib64
)

set(BULLET_LIBRARIES BulletCollision LinearMath)

set(TRAJOPT_SOURCE_FILES
    src/trajectory_costs.cpp
    src/kinematic_terms.cpp
    src/collision_terms.cpp
    src/json_marshal.cpp
    src/problem_description.cpp
    src/utils.cpp
    src/plot_callback.cpp
    src/ros_kin.cpp
    src/ros_env.cpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    ${PROJECT_NAME}
    ${OCTOMAP_LIBRARIES}
    ${JSONCPP_LIBRARIES}
  CATKIN_DEPENDS
    trajopt_sco
    trajopt_utils
    trajopt_ext
    roscpp
    urdf
    moveit_core
    moveit_ros_planning
    pluginlib
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} ${TRAJOPT_SOURCE_FILES})
#target_link_libraries(trajopt ${Boost_SYSTEM_LIBRARY} sco utils json osgviewer)
target_link_libraries(${PROJECT_NAME} ${OCTOMAP_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${JSONCPP_LIBRARIES} ${catkin_LIBRARIES})

add_library(moveit_collision_detection_bullet
  src/moveit/collision_common.cpp
  src/moveit/collision_robot_bullet.cpp
  src/moveit/collision_world_bullet.cpp
)

target_link_libraries(moveit_collision_detection_bullet ${catkin_LIBRARIES} ${console_bridge_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_library(moveit_collision_detection_bullet_plugin src/moveit/collision_detector_allocator_bullet.cpp)
target_link_libraries(moveit_collision_detection_bullet_plugin moveit_collision_detection_bullet ${catkin_LIBRARIES})

if (CATKIN_ENABLE_TESTING)

  add_definitions(-DDATA_DIR="${CMAKE_SOURCE_DIR}/data")
  find_package(rostest REQUIRED)
  find_package(catkin REQUIRED COMPONENTS octomap_ros)
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
