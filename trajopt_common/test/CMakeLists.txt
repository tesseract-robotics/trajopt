find_package(GTest REQUIRED)

# TrajOpt Common YAML Unit tests
add_executable(${PROJECT_NAME}_yaml_unit trajopt_common_yaml_conversions_tests.cpp)
target_link_libraries(${PROJECT_NAME}_yaml_unit PRIVATE GTest::GTest GTest::Main ${PROJECT_NAME})
target_compile_options(${PROJECT_NAME}_yaml_unit PRIVATE ${TRAJOPT_COMPILE_OPTIONS_PRIVATE}
                                                         ${TRAJOPT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_yaml_unit PRIVATE ${TRAJOPT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_yaml_unit ENABLE ${TRAJOPT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_yaml_unit PRIVATE VERSION ${TRAJOPT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_yaml_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TRAJOPT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(${PROJECT_NAME}_yaml_unit)
add_dependencies(${PROJECT_NAME}_yaml_unit ${PROJECT_NAME})
add_dependencies(run_tests ${PROJECT_NAME}_yaml_unit)
