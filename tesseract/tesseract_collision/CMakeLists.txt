cmake_minimum_required(VERSION 2.8.3)
project(tesseract_collision)

add_compile_options(-std=c++11 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  trajopt_ext
  roscpp
  geometric_shapes
  tesseract_core
  eigen_conversions
  pluginlib
)

find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system thread REQUIRED)
find_package(octomap REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    ${PROJECT_NAME}_bullet
    ${OCTOMAP_LIBRARIES}
  CATKIN_DEPENDS
    trajopt_ext
    roscpp
    geometric_shapes
    tesseract_core
    eigen_conversions
    pluginlib
  DEPENDS
    EIGEN3
    Boost
    OCTOMAP
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  SYSTEM ${EIGEN3_INCLUDE_DIRS}
  SYSTEM ${Boost_INCLUDE_DIRS}
  SYSTEM ${OCTOMAP_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_bullet
  src/bullet/bullet_contact_checker.cpp
  src/bullet/bullet_utils.cpp
)

target_link_libraries(${PROJECT_NAME}_bullet ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_library(${PROJECT_NAME}_bullet_plugin src/bullet/bullet_plugin.cpp)
target_link_libraries(${PROJECT_NAME}_bullet_plugin ${PROJECT_NAME}_bullet ${catkin_LIBRARIES})

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_bullet
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

if (CATKIN_ENABLE_TESTING)

  find_package(rostest REQUIRED)

  include_directories(
    ${catkin_INCLUDE_DIRS}
  )

  catkin_add_gtest(${PROJECT_NAME}_unit test/collision_unit.cpp)
  target_link_libraries(${PROJECT_NAME}_unit ${PROJECT_NAME}_bullet ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES})

  catkin_add_gtest(${PROJECT_NAME}_convex_concave_unit test/convex_concave_unit.cpp)
  target_link_libraries(${PROJECT_NAME}_convex_concave_unit ${PROJECT_NAME}_bullet ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES})
endif()
