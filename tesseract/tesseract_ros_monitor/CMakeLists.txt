cmake_minimum_required(VERSION 2.8.3)
project(tesseract_ros_monitor)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tesseract_ros
  tesseract_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES tesseract_ros_monitor
  CATKIN_DEPENDS
    roscpp
    tesseract_ros
    tesseract_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Tesseract ROS Nodes
add_executable(${PROJECT_NAME}_contacts_node src/contact_monitor.cpp)
target_link_libraries(${PROJECT_NAME}_contacts_node ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}_environment
  src/environment_monitor.cpp
  src/current_state_monitor.cpp
  )
target_link_libraries(${PROJECT_NAME}_environment
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES})

add_executable(demo_scene demos/demo_scene.cpp)
target_link_libraries(demo_scene ${PROJECT_NAME}_environment ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_environment_node src/environment_monitor_node.cpp)
target_link_libraries(${PROJECT_NAME}_environment_node ${PROJECT_NAME}_environment ${catkin_LIBRARIES})
