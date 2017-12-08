cmake_minimum_required(VERSION 2.8.3)
project(trajopt_ros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system python thread program_options REQUIRED)

set(SCO_SOURCE_FILES
    src/sco/solver_interface.cpp
    src/sco/modeling.cpp
    src/sco/expr_ops.cpp
    src/sco/expr_vec_ops.cpp
    src/sco/optimizers.cpp
    src/sco/modeling_utils.cpp
    src/sco/num_diff.cpp
)

set(UTILS_SOURCE_FILES
    src/utils/stl_to_string.cpp
    src/utils/clock.cpp
    src/utils/config.cpp
    src/utils/logging.cpp
)

set(TRAJOPT_SOURCE_FILES
#    src/trajopt/traj_plotter.cpp
    src/trajopt/trajectory_costs.cpp
    src/trajopt/kinematic_terms.cpp
    src/trajopt/collision_terms.cpp
    src/trajopt/json_marshal.cpp
    src/trajopt/problem_description.cpp
    src/trajopt/bullet_collision_checker.cpp
    src/trajopt/configuration_space.cpp
    src/trajopt/utils.cpp
#    src/trajopt/rave_utils.cpp
    src/trajopt/collision_checker.cpp
    src/trajopt/plot_callback.cpp
    src/trajopt/bullet_unity.cpp
)

set(JSON_SOURCE_FILES
    src/json/json_reader.cpp
    src/json/json_value.cpp
    src/json/json_writer.cpp
)


if (NOT APPLE)
  set (HAVE_BPMPD TRUE)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fvisibility=hidden")
endif()

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES sco utils
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${JSON_INCLUDE_DIRS}
)

if (HAVE_BPMPD)
  add_executable(bpmpd_caller src/sco/bpmpd_caller.cpp)

  if( CMAKE_SIZEOF_VOID_P EQUAL 8 ) # 64 bits
        set(BPMPD_LIBRARY "${CMAKE_SOURCE_DIR}/3rdpartylib/bpmpd_linux64.a")
  else()
        set(BPMPD_LIBRARY "${CMAKE_SOURCE_DIR}/3rdpartylib/bpmpd_linux32.a")
  endif()

  target_link_libraries(bpmpd_caller ${BPMPD_LIBRARY})
  list(APPEND SCO_SOURCE_FILES src/sco/bpmpd_interface.cpp)
  set_property(SOURCE src/sco/bpmpd_interface.cpp APPEND PROPERTY COMPILE_DEFINITIONS BPMPD_CALLER="\\\"${CMAKE_BINARY_DIR}/bin/bpmpd_caller\\\"")

  set(BPMPD_WORKING_DIR ${CMAKE_CURRENT_BINARY_DIR})
  set_property(SOURCE src/sco/bpmpd_caller.cpp APPEND PROPERTY COMPILE_DEFINITIONS BPMPD_WORKING_DIR="${BPMPD_WORKING_DIR}")
  file(COPY src/sco/bpmpd.par DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

  set_property(SOURCE src/sco/solver_interface.cpp APPEND PROPERTY COMPILE_DEFINITIONS HAVE_BPMPD )
endif()

if (GUROBI_FOUND)
  set_property(SOURCE src/sco/solver_interface.cpp APPEND PROPERTY COMPILE_DEFINITIONS HAVE_GUROBI )
  list(APPEND SCO_SOURCE_FILES src/sco/gurobi_interface.cpp)
endif(GUROBI_FOUND)

add_library(json ${JSON_SOURCE_FILES})
set_target_properties(json PROPERTIES COMPILE_FLAGS -fPIC)

add_library(utils ${UTILS_SOURCE_FILES})
target_link_libraries(utils ${Boost_PROGRAM_OPTIONS_LIBRARY})

add_library(sco ${SCO_SOURCE_FILES})

set (SCO_LINK_LIBS utils)
if (GUROBI_FOUND)
  list(APPEND SCO_LINK_LIBS ${GUROBI_LIBRARIES})
endif()
if (HAVE_BPMPD)
  list(APPEND SCO_LINK_LIBS ${BPMPD_LIBRARY})
endif()

target_link_libraries(sco ${SCO_LINK_LIBS})

#add_library(trajopt ${TRAJOPT_SOURCE_FILES})
#target_link_libraries(trajopt ${Boost_SYSTEM_LIBRARY} sco utils json osgviewer)
#target_link_libraries(trajopt ${OpenRAVE_BOTH_LIBRARIES} ${Boost_SYSTEM_LIBRARY} sco utils json osgviewer)

## Add gtest based cpp test target and link libraries
set(SCO_TEST_SOURCE
  test/sco/unit.cpp
  test/sco/small-problems-unit.cpp
  test/sco/solver-interface-unit.cpp
)
add_definitions(-DDATA_DIR="${CMAKE_SOURCE_DIR}/data")

catkin_add_gtest(sco-unit ${SCO_TEST_SOURCE})
if(TARGET sco-unit)
    target_link_libraries(sco-unit ${PROJECT_NAME})
endif()


#catkin_add_gtest(collision-checker-unit test/trajopt/collision-checker-unit.cpp)
#if(TARGET collision-checker-unit)
#    target_link_libraries(collision-checker-unit ${PROJECT_NAME})
#endif()

#catkin_add_gtest(planning-unit test/trajopt/planning-unit.cpp)
#if(TARGET planning-unit)
#    target_link_libraries(planning-unit ${PROJECT_NAME})
#endif()

#catkin_add_gtest(render-collision-test test/trajopt/render_collision_test.cpp)
#if(TARGET render-collision-test)
#    target_link_libraries(render-collision-test ${PROJECT_NAME})
#endif()

#catkin_add_gtest(cast-cost-unit test/trajopt/cast-cost-unit.cpp)
#if(TARGET cast-cost-unit)
#    target_link_libraries(cast-cost-unit ${PROJECT_NAME})
#endif()


## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/trajopt_ros.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/trajopt_ros_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_trajopt_ros.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
