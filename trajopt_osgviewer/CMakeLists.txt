cmake_minimum_required(VERSION 2.8.3)
project(trajopt_osgviewer)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS trajopt_utils)
find_package(Boost COMPONENTS system python thread program_options REQUIRED)
find_package(OpenRAVE REQUIRED)

link_directories(
    /usr/local/lib
    /usr/local/lib64
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS trajopt_utils
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenRAVE_INCLUDE_DIRS}
)

set(OpenRAVE_BOTH_LIBRARIES ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES})
set(OSG_LIBRARIES osg osgViewer OpenThreads osgDB osgGA osgUtil osgText)
add_library(${PROJECT_NAME} src/osgviewer.cpp src/robot_ui.cpp)
target_link_libraries(${PROJECT_NAME} ${OSG_LIBRARIES} ${OpenRAVE_BOTH_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${catkin_LIBRARIES} )

if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}_viewer_experiment test/viewer_experiment.cpp)
  target_link_libraries(${PROJECT_NAME}_viewer_experiment ${OpenRAVE_BOTH_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${PROJECT_NAME} ${catkin_LIBRARIES})

  catkin_add_gtest(${PROJECT_NAME}_osg_expt test/osg_expt.cpp)
  target_link_libraries(${PROJECT_NAME}_osg_expt ${OSG_LIBRARIES} ${catkin_LIBRARIES})

  catkin_add_gtest(${PROJECT_NAME}_robotui_expt test/robotui_expt.cpp)
  target_link_libraries(${PROJECT_NAME}_robotui_expt ${OpenRAVE_BOTH_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${PROJECT_NAME} ${catkin_LIBRARIES})

  catkin_add_gtest(${PROJECT_NAME}_test_osgviewer_text test/test_osgviewer_text.cpp)
  target_link_libraries(${PROJECT_NAME}_test_osgviewer_text ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()
