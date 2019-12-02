find_package(GTest REQUIRED)
find_package(PCL REQUIRED COMPONENTS core features filters io segmentation surface)

macro(add_gtest test_name test_file)
  add_executable(${test_name} ${test_file})
  trajopt_target_compile_options(${test_name} PRIVATE)
  trajopt_clang_tidy(${test_name})
  target_compile_definitions(${test_name} PRIVATE TRAJOPT_DIR="${CMAKE_SOURCE_DIR}")
  target_link_libraries(${test_name}
      ${PROJECT_NAME}
      GTest::GTest
      GTest::Main
      ${Boost_SYSTEM_LIBRARY}
      ${Boost_PROGRAM_OPTIONS_LIBRARY})
  target_include_directories(${test_name} PRIVATE
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
      ${GTEST_INCLUDE_DIRS})
  target_include_directories(${test_name} SYSTEM PRIVATE
      ${PCL_INCLUDE_DIRS})
  trajopt_gtest_discover_tests(${test_name})
  add_dependencies(${test_name} ${PROJECT_NAME})
  add_dependencies(run_tests ${test_name})
endmacro()

add_gtest(${PROJECT_NAME}_planning_unit planning_unit.cpp)
#add_gtest(${PROJECT_NAME}_interface_unit interface_unit.cpp)
add_gtest(${PROJECT_NAME}_joint_costs_unit joint_costs_unit.cpp)
add_gtest(${PROJECT_NAME}_kinematic_costs_unit kinematic_costs_unit.cpp)
add_gtest(${PROJECT_NAME}_cast_cost_unit cast_cost_unit.cpp)
add_gtest(${PROJECT_NAME}_cast_cost_world_unit cast_cost_world_unit.cpp)
add_gtest(${PROJECT_NAME}_cast_cost_attached_unit cast_cost_attached_unit.cpp)
add_gtest(${PROJECT_NAME}_cast_cost_octomap_unit cast_cost_octomap_unit.cpp)
