cmake_minimum_required(VERSION 3.5.0)
project(trajopt VERSION 0.1.0 LANGUAGES CXX)

find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system python thread program_options REQUIRED)
find_package(console_bridge REQUIRED)
find_package(trajopt_sco REQUIRED)
find_package(trajopt_utils REQUIRED)
find_package(tesseract_environment REQUIRED)
find_package(tesseract_kinematics REQUIRED)
find_package(tesseract_collision REQUIRED)
find_package(tesseract_visualization REQUIRED)
find_package(tesseract_scene_graph REQUIRED) # Dont understand why this is needed. Should be included by tesseract_environment
find_package(tesseract_geometry REQUIRED) # Dont understand why this is needed. Should be included by tesseract_environment
find_package(orocos_kdl REQUIRED) # Dont understand why this is needed. Should be included by tesseract_environment

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)

set(TRAJOPT_PKGCONFIG_REQUIRES Eigen3 console_bridge Boost PkgConfig trajopt_sco trajopt_utils tesseract_environment tesseract_kinematics tesseract_collision tesseract_visualization orocos_kdl)

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

add_library(${PROJECT_NAME} SHARED ${TRAJOPT_SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} PUBLIC
    trajopt_sco
    trajopt_utils
    tesseract_environment_core
    tesseract_kinematics_core
    tesseract_collision_core
    tesseract_visualization
    tesseract_scene_graph
    tesseract_geometry
    console_bridge
    orocos-kdl
    ${Boost_SYSTEM_LIBRARY}
    ${JSONCPP_LIBRARIES})
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_11)
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
    ${JSONCPP_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${orocos_kdl_INCLUDE_DIRS}) # Orocos did not export target correctly

list (APPEND PACKAGE_LIBRARIES ${PROJECT_NAME})

# Mark executables and/or libraries for installation
install(TARGETS ${PACKAGE_LIBRARIES} EXPORT ${PROJECT_NAME}-targets DESTINATION lib)
install(EXPORT ${PROJECT_NAME}-targets DESTINATION lib/cmake/${PROJECT_NAME})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp" PATTERN "*.hxx"
  PATTERN ".svn" EXCLUDE
 )

install(FILES package.xml DESTINATION share/${PROJECT_NAME})

# Create cmake config files
include(CMakePackageConfigHelpers)
configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/cmake/${PROJECT_NAME}-config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
  INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
  NO_CHECK_REQUIRED_COMPONENTS_MACRO)

write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
  VERSION ${PROJECT_VERSION} COMPATIBILITY ExactVersion)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
  DESTINATION lib/cmake/${PROJECT_NAME})

export(EXPORT ${PROJECT_NAME}-targets FILE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-targets.cmake)

option(ENABLE_TESTS "Enable tests" OFF)
if (${ENABLE_TESTS})
  enable_testing()
  add_custom_target(run_tests ALL
      WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
      COMMAND ${CMAKE_CTEST_COMMAND} -C $<CONFIGURATION> -V)

  add_subdirectory(test)
endif()

#if (CATKIN_ENABLE_TESTING)

#  find_package(rostest REQUIRED)
#  find_package(catkin REQUIRED COMPONENTS octomap_ros pcl_conversions pr2_description roslib)
#  find_package(PCL REQUIRED COMPONENTS core features filters io segmentation surface)

#  add_definitions(-DDATA_DIR="${CMAKE_SOURCE_DIR}/test")

#  include_directories(
#    test
#    ${PCL_INCLUDE_DIRS}
#    ${catkin_INCLUDE_DIRS}
#  )

#  catkin_add_gtest(${PROJECT_NAME}_planning_unit test/planning_unit.cpp)
#  target_link_libraries(${PROJECT_NAME}_planning_unit ${PROJECT_NAME} ${Boost_SYSTEM_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${catkin_LIBRARIES} ${console_bridge_LIBRARIES})
#  target_compile_options(${PROJECT_NAME}_planning_unit PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

#  catkin_add_gtest(${PROJECT_NAME}_interface_unit test/interface_unit.cpp)
#  target_link_libraries(${PROJECT_NAME}_interface_unit ${PROJECT_NAME} ${Boost_SYSTEM_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${catkin_LIBRARIES} ${console_bridge_LIBRARIES})
#  target_compile_options(${PROJECT_NAME}_interface_unit PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

#  catkin_add_gtest(${PROJECT_NAME}_costs_unit test/costs_unit.cpp)
#  target_link_libraries(${PROJECT_NAME}_costs_unit ${PROJECT_NAME} ${Boost_SYSTEM_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${catkin_LIBRARIES} ${console_bridge_LIBRARIES})
#  target_compile_options(${PROJECT_NAME}_costs_unit PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

#  catkin_add_gtest(${PROJECT_NAME}_cast_cost_unit test/cast_cost_unit.cpp)
#  target_link_libraries(${PROJECT_NAME}_cast_cost_unit ${PROJECT_NAME} ${Boost_SYSTEM_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${catkin_LIBRARIES} ${console_bridge_LIBRARIES})
#  target_compile_options(${PROJECT_NAME}_cast_cost_unit PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

#  catkin_add_gtest(${PROJECT_NAME}_cast_cost_world_unit test/cast_cost_world_unit.cpp)
#  target_link_libraries(${PROJECT_NAME}_cast_cost_world_unit ${PROJECT_NAME} ${Boost_SYSTEM_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${catkin_LIBRARIES} ${console_bridge_LIBRARIES})
#  target_compile_options(${PROJECT_NAME}_cast_cost_world_unit PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

#  catkin_add_gtest(${PROJECT_NAME}_cast_cost_attached_unit test/cast_cost_attached_unit.cpp)
#  target_link_libraries(${PROJECT_NAME}_cast_cost_attached_unit ${PROJECT_NAME} ${Boost_SYSTEM_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${catkin_LIBRARIES} ${console_bridge_LIBRARIES})
#  target_compile_options(${PROJECT_NAME}_cast_cost_attached_unit PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

#  catkin_add_gtest(${PROJECT_NAME}_cast_cost_octomap_unit test/cast_cost_octomap_unit.cpp)
#  target_link_libraries(${PROJECT_NAME}_cast_cost_octomap_unit ${PROJECT_NAME} ${Boost_SYSTEM_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${console_bridge_LIBRARIES})
#  target_compile_options(${PROJECT_NAME}_cast_cost_octomap_unit PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)
#endif()
