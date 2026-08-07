find_gtest()

set(SCO_TEST_SOURCE unit.cpp solver-utils-unit.cpp)
if(HAVE_GUROBI OR HAVE_qpOASES)
  list(
    APPEND
    SCO_TEST_SOURCE
    small-problems-unit.cpp
    solver-interface-unit.cpp)
endif()

add_executable(${PROJECT_NAME}-test ${SCO_TEST_SOURCE})
target_link_libraries(
  ${PROJECT_NAME}-test
  GTest::GTest
  GTest::Main
  ${PROJECT_NAME})
if(osqp_FOUND)
  target_link_libraries(${PROJECT_NAME}-test osqp::osqp)
endif()
target_compile_options(${PROJECT_NAME}-test PRIVATE ${TRAJOPT_COMPILE_OPTIONS_PRIVATE}
                                                    ${TRAJOPT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}-test PRIVATE ${TRAJOPT_COMPILE_DEFINITIONS_PUBLIC}
                                                        TRAJOPT_IFOPT_DIR="${CMAKE_SOURCE_DIR}")
target_cxx_version(${PROJECT_NAME}-test PRIVATE VERSION ${TRAJOPT_CXX_VERSION})
target_clang_tidy(${PROJECT_NAME}-test ENABLE ${TRAJOPT_ENABLE_CLANG_TIDY})
add_gtest_discover_tests(${PROJECT_NAME}-test)
if(HAVE_BPMPD)
  add_dependencies(${PROJECT_NAME}-test bpmpd_caller)
endif()

add_dependencies(${PROJECT_NAME}-test ${PROJECT_NAME})
add_dependencies(run_tests ${PROJECT_NAME}-test)
