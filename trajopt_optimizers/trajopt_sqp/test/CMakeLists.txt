find_gtest()
find_package(trajopt REQUIRED)

macro(add_gtest test_name test_file)
  add_executable(${test_name} ${test_file})
  target_compile_options(${test_name} PRIVATE ${TRAJOPT_COMPILE_OPTIONS_PRIVATE} ${TRAJOPT_COMPILE_OPTIONS_PUBLIC})
  target_compile_definitions(${test_name} PRIVATE ${TRAJOPT_COMPILE_DEFINITIONS_PUBLIC}
                                                  TRAJOPT_IFOPT_DIR="${CMAKE_SOURCE_DIR}")
  target_cxx_version(${test_name} PRIVATE VERSION ${TRAJOPT_CXX_VERSION})
  target_clang_tidy(${test_name} ENABLE ${TRAJOPT_ENABLE_CLANG_TIDY})
  target_link_libraries(
    ${test_name}
    ${PROJECT_NAME}
    console_bridge
    trajopt::trajopt_ifopt
    trajopt::trajopt_common
    GTest::GTest
    GTest::Main)
  target_include_directories(${test_name} PRIVATE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
                                                  ${GTEST_INCLUDE_DIRS})
  add_gtest_discover_tests(${test_name})
  add_dependencies(${test_name} ${PROJECT_NAME})
  add_dependencies(run_tests ${test_name})
endmacro()

add_gtest(${PROJECT_NAME}_expressions_unit expressions_unit.cpp)
add_gtest(${PROJECT_NAME}_joint_position_optimization_unit joint_position_optimization_unit.cpp)
add_gtest(${PROJECT_NAME}_joint_velocity_optimization_unit joint_velocity_optimization_unit.cpp)
add_gtest(${PROJECT_NAME}_joint_acceleration_optimization_unit joint_acceleration_optimization_unit.cpp)
add_gtest(${PROJECT_NAME}_joint_jerk_optimization_unit joint_jerk_optimization_unit.cpp)
add_gtest(${PROJECT_NAME}_cast_cost_attached_unit cast_cost_attached_unit.cpp)
add_gtest(${PROJECT_NAME}_cast_cost_octomap_unit cast_cost_octomap_unit.cpp)
add_gtest(${PROJECT_NAME}_cast_cost_unit cast_cost_unit.cpp)
add_gtest(${PROJECT_NAME}_cast_cost_world_unit cast_cost_world_unit.cpp)
add_gtest(${PROJECT_NAME}_numerical_ik_unit numerical_ik_unit.cpp)
add_gtest(${PROJECT_NAME}_planning_unit planning_unit.cpp)
add_gtest(${PROJECT_NAME}_simple_collision_unit simple_collision_unit.cpp)
add_gtest(${PROJECT_NAME}_cart_position_optimization_unit cart_position_optimization_unit.cpp)
add_gtest(${PROJECT_NAME}_hessian_gradient_unit hessian_gradient_unit.cpp)
