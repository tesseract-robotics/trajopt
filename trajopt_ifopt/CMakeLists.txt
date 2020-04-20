cmake_minimum_required(VERSION 3.5.0)
project(trajopt_ifopt VERSION 0.1.0 LANGUAGES CXX)

find_package(trajopt_utils)
find_package(trajopt)
find_package(console_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ifopt REQUIRED)
find_package(tesseract REQUIRED)

list(FIND CMAKE_CXX_COMPILE_FEATURES cxx_std_14 CXX_FEATURE_FOUND)

###########
## Build ##
###########

set(TRAJOPT_IFOPT_SOURCE_FILES
    src/cartesian_position_constraint.cpp
    src/joint_position_constraint.cpp
    src/joint_velocity_constraint.cpp
    src/squared_cost.cpp
)

add_library(${PROJECT_NAME} SHARED ${TRAJOPT_IFOPT_SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} PUBLIC
  console_bridge
  ifopt::ifopt_core
  trajopt::trajopt_utils
  trajopt::trajopt
  tesseract::tesseract)
trajopt_target_compile_options(${PROJECT_NAME} PUBLIC)
trajopt_clang_tidy(${PROJECT_NAME})
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_cart_position_example examples/cart_position_example.cpp)
target_link_libraries(${PROJECT_NAME}_cart_position_example ${PROJECT_NAME} ifopt::ifopt_ipopt trajopt::trajopt_test_data)
set_property(TARGET ${PROJECT_NAME}_cart_position_example PROPERTY CXX_STANDARD 14)

add_executable(${PROJECT_NAME}_joint_position_example examples/joint_position_example.cpp)
target_link_libraries(${PROJECT_NAME}_joint_position_example ${PROJECT_NAME} ifopt::ifopt_ipopt)
set_property(TARGET ${PROJECT_NAME}_joint_position_example PROPERTY CXX_STANDARD 14)

add_executable(${PROJECT_NAME}_velocity_smoother examples/velocity_smoother.cpp)
target_link_libraries(${PROJECT_NAME}_velocity_smoother ${PROJECT_NAME} ifopt::ifopt_ipopt)
set_property(TARGET ${PROJECT_NAME}_velocity_smoother PROPERTY CXX_STANDARD 14)

#############
## Install ##
#############

list (APPEND PACKAGE_LIBRARIES ${PROJECT_NAME})

# Mark executables and/or libraries for installation
install(TARGETS
  ${PACKAGE_LIBRARIES}
  ${PROJECT_NAME}_cart_position_example
  ${PROJECT_NAME}_joint_position_example
  ${PROJECT_NAME}_velocity_smoother

    EXPORT ${PROJECT_NAME}-targets DESTINATION lib)
install(EXPORT ${PROJECT_NAME}-targets
    NAMESPACE trajopt:: DESTINATION lib/cmake/${PROJECT_NAME})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
 )

install(FILES package.xml DESTINATION share/${PROJECT_NAME})

# Create cmake config files
include(CMakePackageConfigHelpers)
configure_package_config_file(
    ${CMAKE_CURRENT_LIST_DIR}/cmake/${PROJECT_NAME}-config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
    INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
    NO_CHECK_REQUIRED_COMPONENTS_MACRO)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
    VERSION ${PROJECT_VERSION} COMPATIBILITY ExactVersion)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
    DESTINATION lib/cmake/${PROJECT_NAME})

export(EXPORT ${PROJECT_NAME}-targets FILE
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-targets.cmake)

#############
## Testing ##
#############

if (TRAJOPT_ENABLE_TESTING)
  enable_testing()
  add_custom_target(run_tests ALL
      WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
      COMMAND ${CMAKE_CTEST_COMMAND} -C $<CONFIGURATION> -V)

  add_subdirectory(test)
endif()
