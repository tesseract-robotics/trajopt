find_package(GTest REQUIRED)

macro(add_gtest test_name test_file)
  add_executable(${test_name} ${test_file})
  trajopt_target_compile_options(${test_name} PRIVATE)
#  trajopt_clang_tidy(${test_name})
  target_compile_definitions(${test_name} PRIVATE TRAJOPT_IFOPT_DIR="${CMAKE_SOURCE_DIR}")
  target_link_libraries(${test_name}
      ${PROJECT_NAME}
      ifopt::ifopt_ipopt
      GTest::GTest
      GTest::Main
      ${Boost_SYSTEM_LIBRARY}
      ${Boost_PROGRAM_OPTIONS_LIBRARY}
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

add_gtest(${PROJECT_NAME}_cast_cost_unit cast_cost_unit.cpp)
add_gtest(${PROJECT_NAME}_collision_unit collision_unit.cpp)
add_gtest(${PROJECT_NAME}_joint_terms_unit joint_terms_unit.cpp)
add_gtest(${PROJECT_NAME}_utils_unit utils_unit.cpp)
add_gtest(${PROJECT_NAME}_variable_sets_unit variable_sets_unit.cpp)

