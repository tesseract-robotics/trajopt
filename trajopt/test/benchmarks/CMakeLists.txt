find_package(benchmark REQUIRED)
find_package(osqp REQUIRED)

macro(add_benchmark benchmark_name benchmark_file)
  add_executable(${benchmark_name} ${benchmark_file})
  target_compile_options(${benchmark_name} PRIVATE ${TRAJOPT_COMPILE_OPTIONS_PRIVATE} ${TRAJOPT_COMPILE_OPTIONS_PUBLIC})
  target_compile_definitions(${benchmark_name} PRIVATE ${TRAJOPT_COMPILE_DEFINITIONS} TRAJOPT_DIR="${CMAKE_SOURCE_DIR}")
  target_cxx_version(${benchmark_name} PRIVATE VERSION ${TRAJOPT_CXX_VERSION})
  target_clang_tidy(${benchmark_name} ENABLE ${TRAJOPT_ENABLE_CLANG_TIDY})
  target_link_libraries(
    ${benchmark_name}
    ${PROJECT_NAME}
    tesseract::tesseract
    osqp::osqpstatic
    benchmark::benchmark)
  target_include_directories(${benchmark_name} PRIVATE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>")
  add_dependencies(${benchmark_name} ${PROJECT_NAME})
  add_run_benchmark_target(${benchmark_name})
endmacro()

add_benchmark(${PROJECT_NAME}_joint_term_benchmarks joint_term_benchmarks.cpp)
