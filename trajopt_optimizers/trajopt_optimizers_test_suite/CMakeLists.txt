cmake_minimum_required(VERSION 3.5.0)
project(trajopt_optimizers_test_suite)

find_package(console_bridge REQUIRED)
# find_package(Pagmo REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(trajopt_ifopt REQUIRED)
find_package(trajopt_utils REQUIRED)
# find_package(trajopt_pagmo REQUIRED)
find_package(trajopt_sqp REQUIRED)
find_package(ifopt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTest REQUIRED)

###########
## Build ##
###########

add_library(${PROJECT_NAME} SHARED
    # src/basic_optimization_unit.cpp
    # src/cart_position_optimization_unit.cpp
    src/joint_position_optimization_unit.cpp
    src/joint_velocity_optimization_unit.cpp
    include/basic_optimization_unit.hpp
    # include/cart_position_optimization_unit.hpp
    include/joint_position_optimization_unit.hpp
    include/joint_velocity_optimization_unit.hpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
    ifopt::ifopt_core
    console_bridge
    # Pagmo::pagmo
    OsqpEigen::OsqpEigen
)
trajopt_target_compile_options(${PROJECT_NAME} PUBLIC)
trajopt_clang_tidy(${PROJECT_NAME})
list (APPEND PACKAGE_LIBRARIES ${PROJECT_NAME})


#############
## Install ##
#############

# trajopt_configure_package(${PACKAGE_LIBRARIES})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
 )

#############
## Testing ##
#############

# if (ENABLE_TEST_OPT)
  enable_testing()
  add_custom_target(run_tests ALL
      WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
      COMMAND ${CMAKE_CTEST_COMMAND} -C $<CONFIGURATION> -V)
# endif()

macro(add_gtest test_name test_file)
  add_executable(${test_name} ${test_file})
  trajopt_target_compile_options(${test_name} PRIVATE)
#  trajopt_clang_tidy(${test_name})
  target_compile_definitions(${test_name} PRIVATE TRAJOPT_IFOPT_DIR="${CMAKE_SOURCE_DIR}")
  target_link_libraries(${test_name}
      ${PROJECT_NAME}
      ifopt::ifopt_ipopt
      trajopt::trajopt_ifopt
      trajopt::trajopt_test_data
      # trajopt::trajopt_pagmo
      trajopt::trajopt_sqp
      GTest::GTest
      GTest::Main
      trajopt::trajopt_test_data)
  target_include_directories(${test_name} PRIVATE
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
      ${GTEST_INCLUDE_DIRS})
  target_include_directories(${test_name} SYSTEM PRIVATE
      ${PCL_INCLUDE_DIRS})
  trajopt_gtest_discover_tests(${test_name})
  add_dependencies(${test_name} ${PROJECT_NAME})
  add_dependencies(run_tests ${test_name})
endmacro()

add_gtest(${PROJECT_NAME}_joint_position_optimization_unit src/joint_position_optimization_unit.cpp)
add_gtest(${PROJECT_NAME}_joint_velocity_optimization_unit src/joint_velocity_optimization_unit.cpp)
# add_gtest(${PROJECT_NAME}_cart_position_optimization_unit src/cart_position_optimization_unit.cpp)
# add_gtest(${PROJECT_NAME}_basic_optimization_unit src/basic_optimization_unit.cpp)
