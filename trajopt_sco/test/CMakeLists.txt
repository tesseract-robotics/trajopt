find_package(GTest REQUIRED)

set(SCO_TEST_SOURCE
    unit.cpp
    small-problems-unit.cpp
    solver-interface-unit.cpp
    solver-utils-unit.cpp
)

add_executable(${PROJECT_NAME}-test ${SCO_TEST_SOURCE})
target_link_libraries(${PROJECT_NAME}-test ${GTEST_BOTH_LIBRARIES} ${PROJECT_NAME})
if (osqp_FOUND)
    target_link_libraries(${PROJECT_NAME}-test osqp::osqpstatic)
endif()
target_compile_options(${PROJECT_NAME}-test PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME}-test PRIVATE -std=c++11)
else()
    target_compile_features(${PROJECT_NAME}-test PRIVATE cxx_std_11)
endif()
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(${PROJECT_NAME}-test PRIVATE -mno-avx)
  else()
    message(WARNING "Non-GNU compiler detected. If using AVX instructions, Eigen alignment issues may result.")
  endif()
target_include_directories(${PROJECT_NAME}-test PRIVATE ${GTEST_INCLUDE_DIRS})
add_test(${PROJECT_NAME}-test ${PROJECT_NAME}-test)
add_dependencies(${PROJECT_NAME}-test ${PACKAGE_LIBRARIES} bpmpd_caller)
add_dependencies(run_tests ${PROJECT_NAME}-test)
