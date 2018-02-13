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
find_package(OpenRAVE REQUIRED)

link_directories(
    /usr/local/lib
    /usr/local/lib64
)

set(OpenRAVE_BOTH_LIBRARIES ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES})
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
  LIBRARIES ${PROJECT_NAME}
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
  ${OpenRAVE_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} ${TRAJOPT_SOURCE_FILES})
#target_link_libraries(trajopt ${Boost_SYSTEM_LIBRARY} sco utils json osgviewer)
target_link_libraries(${PROJECT_NAME} ${OpenRAVE_BOTH_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${catkin_LIBRARIES})

add_library(moveit_collision_detection_bullet
  src/moveit/collision_common.cpp
  src/moveit/collision_robot_bullet.cpp
  src/moveit/collision_world_bullet.cpp
)

target_link_libraries(moveit_collision_detection_bullet ${catkin_LIBRARIES} ${console_bridge_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${Boost_LIBRARIES})

add_library(moveit_collision_detection_bullet_plugin src/moveit/collision_detector_allocator_bullet.cpp)
target_link_libraries(moveit_collision_detection_bullet_plugin moveit_collision_detection_bullet ${catkin_LIBRARIES})

if (CATKIN_ENABLE_TESTING)
  include_directories(test)
  add_definitions(-DDATA_DIR="${CMAKE_SOURCE_DIR}/data")
  find_package(rostest REQUIRED)

  add_rostest_gtest(${PROJECT_NAME}_planning_unit test/planning_unit.launch test/planning_unit.cpp)
  target_link_libraries(${PROJECT_NAME}_planning_unit ${PROJECT_NAME} ${Boost_SYSTEM_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${catkin_LIBRARIES})
endif()

#if (CATKIN_ENABLE_TESTING)
#  include_directories(test)
#  add_definitions(-DDATA_DIR="${CMAKE_SOURCE_DIR}/data")

#  catkin_add_gtest(${PROJECT_NAME}-collision-checker-unit test/collision-checker-unit.cpp)
#  target_link_libraries(${PROJECT_NAME}-collision-checker-unit ${PROJECT_NAME} ${Boost_SYSTEM_LIBRARY} ${catkin_LIBRARIES})

#  catkin_add_gtest(${PROJECT_NAME}-planning-unit test/planning-unit.cpp)
#  target_link_libraries(${PROJECT_NAME}-planning-unit ${PROJECT_NAME} ${Boost_SYSTEM_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${catkin_LIBRARIES})

#  catkin_add_gtest(${PROJECT_NAME}-render-collision-test test/render_collision_test.cpp)
#  target_link_libraries(${PROJECT_NAME}-render-collision-test ${PROJECT_NAME} ${Boost_SYSTEM_LIBRARY} ${catkin_LIBRARIES})

#  catkin_add_gtest(${PROJECT_NAME}-cast-cost-unit test/cast-cost-unit.cpp)
#  target_link_libraries(${PROJECT_NAME}-cast-cost-unit ${PROJECT_NAME} ${Boost_SYSTEM_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${catkin_LIBRARIES})
#endif()
