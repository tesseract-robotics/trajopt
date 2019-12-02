cmake_minimum_required(VERSION 3.5.0)
project(trajopt VERSION 0.1.0 LANGUAGES CXX)

find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system python thread program_options REQUIRED)
find_package(console_bridge REQUIRED)
find_package(trajopt_sco REQUIRED)
find_package(trajopt_utils REQUIRED)
find_package(tesseract REQUIRED)
find_package(tesseract_visualization REQUIRED)
find_package(jsoncpp REQUIRED)

link_directories(
    /usr/local/lib
    /usr/local/lib64
)

set(TRAJOPT_SOURCE_FILES
    src/trajectory_costs.cpp
    src/kinematic_terms.cpp
    src/collision_terms.cpp
    src/json_marshal.cpp
    src/problem_description.cpp
    src/utils.cpp
    src/plot_callback.cpp
    src/file_write_callback.cpp
)

add_library(${PROJECT_NAME} SHARED ${TRAJOPT_SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} PUBLIC
    trajopt::trajopt_sco
    trajopt::trajopt_utils
    tesseract::tesseract
    tesseract::tesseract_visualization
    console_bridge
    ${Boost_SYSTEM_LIBRARY}
    jsoncpp_lib)
trajopt_target_compile_options(${PROJECT_NAME} PUBLIC)
trajopt_clang_tidy(${PROJECT_NAME})
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS})

trajopt_configure_package(${PROJECT_NAME})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp" PATTERN "*.hxx"
  PATTERN ".svn" EXCLUDE
 )

if (ENABLE_TESTS)
  enable_testing()
  add_custom_target(run_tests ALL
      WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
      COMMAND ${CMAKE_CTEST_COMMAND} -C $<CONFIGURATION> -V)

  add_subdirectory(test)
endif()
