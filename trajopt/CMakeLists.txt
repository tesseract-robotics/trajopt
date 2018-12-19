cmake_minimum_required(VERSION 2.8.3)
project(trajopt)

add_compile_options(-std=c++11 -Wall -Wextra)

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
    src/file_write_callback.cpp
)

catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIRS}
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
  SYSTEM ${EIGEN3_INCLUDE_DIRS}
  SYSTEM ${Boost_INCLUDE_DIRS}
  SYSTEM ${JSONCPP_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} ${TRAJOPT_SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${Boost_SYSTEM_LIBRARY} ${JSONCPP_LIBRARIES} ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME} PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp" PATTERN "*.hxx"
  PATTERN ".svn" EXCLUDE
 )

if (CATKIN_ENABLE_TESTING)

  find_package(rostest REQUIRED)
  find_package(catkin REQUIRED COMPONENTS tesseract_ros octomap_ros trajopt_test_support pcl_conversions pr2_description roslib)
  find_package(PCL REQUIRED COMPONENTS core features filters io segmentation surface)

  include_directories(
    test
    ${PCL_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
  )

  add_rostest_gtest(${PROJECT_NAME}_planning_unit test/planning_unit.launch test/planning_unit.cpp)
  target_link_libraries(${PROJECT_NAME}_planning_unit ${PROJECT_NAME} ${Boost_SYSTEM_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${catkin_LIBRARIES})
  target_compile_options(${PROJECT_NAME}_planning_unit PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

  add_rostest_gtest(${PROJECT_NAME}_interface_unit test/interface_unit.launch test/interface_unit.cpp)
  target_link_libraries(${PROJECT_NAME}_interface_unit ${PROJECT_NAME} ${Boost_SYSTEM_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${catkin_LIBRARIES})
  target_compile_options(${PROJECT_NAME}_interface_unit PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

  add_rostest_gtest(${PROJECT_NAME}_costs_unit test/costs_unit.launch test/costs_unit.cpp)
  target_link_libraries(${PROJECT_NAME}_costs_unit ${PROJECT_NAME} ${Boost_SYSTEM_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${catkin_LIBRARIES})
  target_compile_options(${PROJECT_NAME}_costs_unit PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

  add_rostest_gtest(${PROJECT_NAME}_cast_cost_unit test/cast_cost_unit.launch test/cast_cost_unit.cpp)
  target_link_libraries(${PROJECT_NAME}_cast_cost_unit ${PROJECT_NAME} ${Boost_SYSTEM_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${catkin_LIBRARIES})
  target_compile_options(${PROJECT_NAME}_cast_cost_unit PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

  add_rostest_gtest(${PROJECT_NAME}_cast_cost_world_unit test/cast_cost_world_unit.launch test/cast_cost_world_unit.cpp)
  target_link_libraries(${PROJECT_NAME}_cast_cost_world_unit ${PROJECT_NAME} ${Boost_SYSTEM_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${catkin_LIBRARIES})
  target_compile_options(${PROJECT_NAME}_cast_cost_world_unit PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

  add_rostest_gtest(${PROJECT_NAME}_cast_cost_attached_unit test/cast_cost_attached_unit.launch test/cast_cost_attached_unit.cpp)
  target_link_libraries(${PROJECT_NAME}_cast_cost_attached_unit ${PROJECT_NAME} ${Boost_SYSTEM_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${catkin_LIBRARIES})
  target_compile_options(${PROJECT_NAME}_cast_cost_attached_unit PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

  add_rostest_gtest(${PROJECT_NAME}_cast_cost_octomap_unit test/cast_cost_octomap_unit.launch test/cast_cost_octomap_unit.cpp)
  target_link_libraries(${PROJECT_NAME}_cast_cost_octomap_unit ${PROJECT_NAME} ${Boost_SYSTEM_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${PCL_LIBRARIES} ${catkin_LIBRARIES})
  target_compile_options(${PROJECT_NAME}_cast_cost_octomap_unit PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)
endif()
