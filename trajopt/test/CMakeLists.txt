find_package(GTest REQUIRED)
find_package(PCL REQUIRED COMPONENTS core features filters io segmentation surface)

macro(add_gtest test_name test_file)
  add_executable(${test_name} ${test_file})
  target_compile_options(${test_name} PRIVATE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
  if(CXX_FEATURE_FOUND EQUAL "-1")
      target_compile_options(${test_name} PUBLIC -std=c++11)
  else()
      target_compile_features(${test_name} PUBLIC cxx_std_11)
  endif()
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      target_compile_options(${test_name} PUBLIC -mno-avx)
    else()
      message(WARNING "Non-GNU compiler detected. If using AVX instructions, Eigen alignment issues may result.")
    endif()
  target_compile_definitions(${test_name} PRIVATE TRAJOPT_DIR="${CMAKE_SOURCE_DIR}")
  target_link_libraries(${test_name}
      ${PROJECT_NAME}
      ${GTEST_BOTH_LIBRARIES}
      ${Boost_SYSTEM_LIBRARY}
      ${Boost_PROGRAM_OPTIONS_LIBRARY})
  target_include_directories(${test_name} PRIVATE
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
      ${GTEST_INCLUDE_DIRS})
  target_include_directories(${test_name} SYSTEM PRIVATE
      ${PCL_INCLUDE_DIRS})
  add_test(${test_name} ${test_name})
  add_dependencies(${test_name} ${PACKAGE_LIBRARIES})
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
