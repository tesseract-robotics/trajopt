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
find_package(OpenRAVE REQUIRED)

link_directories(
    /usr/local/lib
    /usr/local/lib64
)

set(OpenRAVE_BOTH_LIBRARIES ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES})
set(BULLET_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/ext/bullet/src)
set(BULLET_LIBRARIES BulletCollision LinearMath)
set(JSON_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/ext/json/include)

add_subdirectory(ext)

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
    src/trajopt/traj_plotter.cpp
    src/trajopt/trajectory_costs.cpp
    src/trajopt/kinematic_terms.cpp
    src/trajopt/collision_terms.cpp
    src/trajopt/json_marshal.cpp
    src/trajopt/problem_description.cpp
    src/trajopt/bullet_collision_checker.cpp
    src/trajopt/configuration_space.cpp
    src/trajopt/utils.cpp
    src/trajopt/rave_utils.cpp
    src/trajopt/collision_checker.cpp
    src/trajopt/plot_callback.cpp
    src/trajopt/bullet_unity.cpp
)

if (NOT APPLE)
  set (HAVE_BPMPD TRUE)
#  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fvisibility=hidden")
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sco utils trajopt
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${JSON_INCLUDE_DIRS}
  ${BULLET_INCLUDE_DIRS}
  ${OpenRAVE_INCLUDE_DIRS}
#  ${OSG_INCLUDE_DIRS}
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
  set_property(SOURCE src/sco/bpmpd_interface.cpp APPEND PROPERTY COMPILE_DEFINITIONS BPMPD_CALLER="\\\"${CATKIN_DEVEL_PREFIX}/lib/trajopt_ros/bpmpd_caller\\\"")

  #TODO: Levi check if this is correct.
  set(BPMPD_WORKING_DIR "${CATKIN_DEVEL_PREFIX}/lib/trajopt_ros/")
  set_property(SOURCE src/sco/bpmpd_caller.cpp APPEND PROPERTY COMPILE_DEFINITIONS BPMPD_WORKING_DIR="${BPMPD_WORKING_DIR}")
  file(COPY src/sco/bpmpd.par DESTINATION ${BPMPD_WORKING_DIR})

  set_property(SOURCE src/sco/solver_interface.cpp APPEND PROPERTY COMPILE_DEFINITIONS HAVE_BPMPD )
endif()

if (GUROBI_FOUND)
  set_property(SOURCE src/sco/solver_interface.cpp APPEND PROPERTY COMPILE_DEFINITIONS HAVE_GUROBI )
  list(APPEND SCO_SOURCE_FILES src/sco/gurobi_interface.cpp)
endif(GUROBI_FOUND)

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

target_link_libraries(sco ${catkin_LIBRARIES} ${SCO_LINK_LIBS})
#target_link_libraries(sco utils)

set(OSG_LIBRARIES osg osgViewer OpenThreads osgDB osgGA osgUtil osgText)
add_library(osgviewer src/osgviewer/osgviewer.cpp src/osgviewer/robot_ui.cpp)
target_link_libraries(osgviewer ${OSG_LIBRARIES} ${OpenRAVE_BOTH_LIBRARIES} ${Boost_SYSTEM_LIBRARY} utils )

add_library(trajopt ${TRAJOPT_SOURCE_FILES})
#target_link_libraries(trajopt ${Boost_SYSTEM_LIBRARY} sco utils json osgviewer)
target_link_libraries(trajopt ${OpenRAVE_BOTH_LIBRARIES} ${Boost_SYSTEM_LIBRARY} sco utils json osgviewer)


if (CATKIN_ENABLE_TESTING)
    include_directories(test)
    ### Add gtest based cpp test target and link libraries
    set(SCO_TEST_SOURCE
      test/sco/unit.cpp
      test/sco/small-problems-unit.cpp
      test/sco/solver-interface-unit.cpp
    )
    add_definitions(-DDATA_DIR="${CMAKE_SOURCE_DIR}/data")

    catkin_add_gtest(sco-unit ${SCO_TEST_SOURCE})
    target_link_libraries(sco-unit sco)

    catkin_add_gtest(collision-checker-unit test/trajopt/collision-checker-unit.cpp)
    target_link_libraries(collision-checker-unit trajopt ${Boost_SYSTEM_LIBRARY})

    catkin_add_gtest(planning-unit test/trajopt/planning-unit.cpp)
    target_link_libraries(planning-unit trajopt ${Boost_SYSTEM_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY} osgviewer)

    catkin_add_gtest(render-collision-test test/trajopt/render_collision_test.cpp)
    target_link_libraries(render-collision-test trajopt ${Boost_SYSTEM_LIBRARY} osgviewer)

    catkin_add_gtest(cast-cost-unit test/trajopt/cast-cost-unit.cpp)
    target_link_libraries(cast-cost-unit trajopt ${Boost_SYSTEM_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY} osgviewer)

    catkin_add_gtest(viewer_experiment test/osgviewer/viewer_experiment.cpp)
    target_link_libraries(viewer_experiment ${OpenRAVE_BOTH_LIBRARIES} ${Boost_SYSTEM_LIBRARY} osgviewer)

    catkin_add_gtest(osg_expt test/osgviewer/osg_expt.cpp)
    target_link_libraries(osg_expt ${OSG_LIBRARIES})

    catkin_add_gtest(robotui_expt test/osgviewer/robotui_expt.cpp)
    target_link_libraries(robotui_expt ${OpenRAVE_BOTH_LIBRARIES} ${Boost_SYSTEM_LIBRARY} osgviewer)

    catkin_add_gtest(test_osgviewer_text test/osgviewer/test_osgviewer_text.cpp)
    target_link_libraries(test_osgviewer_text osgviewer)
endif()
