cmake_minimum_required(VERSION 3.5.0)
project(trajopt_utils VERSION 0.1.0 LANGUAGES CXX)

find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system python thread program_options REQUIRED)

list(FIND CMAKE_CXX_COMPILE_FEATURES cxx_std_11 CXX_FEATURE_FOUND)

set(UTILS_SOURCE_FILES
    src/stl_to_string.cpp
    src/clock.cpp
    src/config.cpp
    src/logging.cpp
)

add_library(${PROJECT_NAME} SHARED ${UTILS_SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} PUBLIC ${Boost_LIBRARIES})
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME} PUBLIC -std=c++11)
else()
    target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_11)
endif()
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(${PROJECT_NAME} PUBLIC -mno-avx)
  else()
    message(WARNING "Non-GNU compiler detected. If using AVX instructions, Eigen alignment issues may result.")
  endif()
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS})

list (APPEND PACKAGE_LIBRARIES ${PROJECT_NAME})

# Mark executables and/or libraries for installation
install(TARGETS ${PACKAGE_LIBRARIES} EXPORT ${PROJECT_NAME}-targets DESTINATION lib)
install(EXPORT ${PROJECT_NAME}-targets NAMESPACE trajopt:: DESTINATION lib/cmake/${PROJECT_NAME})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
 )

install(FILES package.xml DESTINATION share/${PROJECT_NAME})

# Create cmake config files
include(CMakePackageConfigHelpers)
configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/cmake/${PROJECT_NAME}-config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
  INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
  NO_CHECK_REQUIRED_COMPONENTS_MACRO)

write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
  VERSION ${PROJECT_VERSION} COMPATIBILITY ExactVersion)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
  DESTINATION lib/cmake/${PROJECT_NAME})

export(EXPORT ${PROJECT_NAME}-targets FILE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-targets.cmake)
