find_package(benchmark REQUIRED)

macro(add_benchmark benchmark_name benchmark_file)
  add_executable(${benchmark_name} ${benchmark_file})
  target_compile_options(${benchmark_name} PRIVATE ${TRAJOPT_COMPILE_OPTIONS_PRIVATE} ${TRAJOPT_COMPILE_OPTIONS_PUBLIC})
  target_compile_definitions(${benchmark_name} PRIVATE ${TRAJOPT_COMPILE_DEFINITIONS})
  target_cxx_version(${benchmark_name} PRIVATE VERSION ${TRAJOPT_CXX_VERSION})
  target_clang_tidy(${benchmark_name} ENABLE ${TRAJOPT_ENABLE_CLANG_TIDY})
  target_link_libraries(
    ${benchmark_name}
    ${PROJECT_NAME}
    ifopt::ifopt_ipopt
    trajopt::trajopt_ifopt
    trajopt::trajopt_common
    benchmark::benchmark)
  target_include_directories(${benchmark_name} PRIVATE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>")
  add_dependencies(${benchmark_name} ${PROJECT_NAME})
  if(TRAJOPT_ENABLE_RUN_BENCHMARKING)
    message(STATUS "Running benchmark ${benchmark_name}")
    add_run_benchmark_target(${benchmark_name})
  endif()
endmacro()

add_benchmark(${PROJECT_NAME}_solve_benchmarks solve_benchmarks.cpp)
