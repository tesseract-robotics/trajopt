cmake_minimum_required(VERSION 3.5.0)
project(trajopt_utils VERSION 0.1.0 LANGUAGES CXX)
include(cmake/trajopt_macros.cmake)

find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system python thread program_options REQUIRED)

set(UTILS_SOURCE_FILES
    src/stl_to_string.cpp
    src/clock.cpp
    src/config.cpp
    src/logging.cpp
)

add_library(${PROJECT_NAME} SHARED ${UTILS_SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} PUBLIC ${Boost_LIBRARIES})
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
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
 )

install(FILES
  "${CMAKE_CURRENT_LIST_DIR}/cmake/trajopt_macros.cmake"
  DESTINATION lib/cmake/${PROJECT_NAME})
