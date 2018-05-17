cmake_minimum_required(VERSION 2.8.3)
project(trajopt)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -Wall -Wextra -Wno-unused-parameter)

find_package(catkin REQUIRED COMPONENTS
  trajopt_sco
  trajopt_utils
  tesseract_core
  roscpp
)

find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system python thread program_options REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)

link_directories(
    /usr/local/lib
    /usr/local/lib64
)

set(TRAJOPT_SOURCE_FILES
    src/trajectory_costs.cpp
    src/kinematic_terms.cpp
    src/collision_terms.cpp
    src/json_marshal.cpp
    src/problem_description.cpp
    src/utils.cpp
    src/plot_callback.cpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    ${PROJECT_NAME}
    ${JSONCPP_LIBRARIES}
  CATKIN_DEPENDS
    trajopt_sco
    trajopt_utils
    tesseract_core
    roscpp
  DEPENDS
    EIGEN3
    Boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} ${TRAJOPT_SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${Boost_SYSTEM_LIBRARY} ${JSONCPP_LIBRARIES} ${catkin_LIBRARIES})

if (CATKIN_ENABLE_TESTING)

  find_package(rostest REQUIRED)
  find_package(catkin REQUIRED COMPONENTS tesseract_ros octomap_ros trajopt_test_support roslib)
  find_package(PCL REQUIRED COMPONENTS core features filters io segmentation surface)

  include_directories(
    test
    ${PCL_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
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
