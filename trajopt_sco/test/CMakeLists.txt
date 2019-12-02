find_package(GTest REQUIRED)

set(SCO_TEST_SOURCE
    unit.cpp
    small-problems-unit.cpp
    solver-interface-unit.cpp
    solver-utils-unit.cpp
)

add_executable(${PROJECT_NAME}-test ${SCO_TEST_SOURCE})
target_link_libraries(${PROJECT_NAME}-test GTest::GTest GTest::Main ${PROJECT_NAME})
if (osqp_FOUND)
    target_link_libraries(${PROJECT_NAME}-test osqp::osqpstatic)
endif()
trajopt_target_compile_options(${PROJECT_NAME}-test PRIVATE)
trajopt_clang_tidy(${PROJECT_NAME}-test)
trajopt_gtest_discover_tests(${PROJECT_NAME}-test)
add_dependencies(${PROJECT_NAME}-test ${PACKAGE_LIBRARIES} bpmpd_caller)
add_dependencies(run_tests ${PROJECT_NAME}-test)
