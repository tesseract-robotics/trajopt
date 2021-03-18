cmake_minimum_required(VERSION 3.10.0)
project(trajopt_utils VERSION 0.1.0 LANGUAGES CXX)

include(cmake/trajopt_macros.cmake)

if(WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)
find_package(ros_industrial_cmake_boilerplate REQUIRED)

# Load variable for clang tidy args, compiler options and cxx version
trajopt_variables()

set(UTILS_SOURCE_FILES
    src/stl_to_string.cpp
    src/clock.cpp
    src/config.cpp
    src/logging.cpp
)

add_library(${PROJECT_NAME} ${UTILS_SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen Boost::program_options)
target_compile_options(${PROJECT_NAME} PRIVATE ${TRAJOPT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME} PUBLIC ${TRAJOPT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME} PUBLIC ${TRAJOPT_COMPILE_DEFINITIONS})
target_cxx_version(${PROJECT_NAME} PUBLIC VERSION ${TRAJOPT_CXX_VERSION})
target_clang_tidy(${PROJECT_NAME} ARGUMENTS ${TRAJOPT_CLANG_TIDY_ARGS} ENABLE ${TRAJOPT_ENABLE_CLANG_TIDY})
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")

configure_package(NAMESPACE trajopt TARGETS ${PROJECT_NAME})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
 )

install(FILES
  "${CMAKE_CURRENT_LIST_DIR}/cmake/trajopt_macros.cmake"
  DESTINATION lib/cmake/${PROJECT_NAME})
