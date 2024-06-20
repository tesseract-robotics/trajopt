find_package(Eigen3 REQUIRED)

# ######################################################################################################################
# Build ##
# ######################################################################################################################
add_library(${PROJECT_NAME}_ifopt src/problem.cc src/composite.cc src/leaves.cc)
target_include_directories(${PROJECT_NAME}_ifopt PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                                                        $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_ifopt PUBLIC Eigen3::Eigen)
# better specify features you need, not c++ version
target_compile_features(${PROJECT_NAME}_ifopt PUBLIC cxx_range_for cxx_auto_type)

# Configure Components
configure_component(
  COMPONENT
  ifopt
  NAMESPACE
  trajopt
  TARGETS
  ${PROJECT_NAME}_ifopt
  DEPENDENCIES
  Eigen3)

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE)

# ######################################################################################################################
# Testing ##
# ######################################################################################################################
if(TRAJOPT_ENABLE_TESTING)
  enable_testing()
  find_package(GTest QUIET)

  add_executable(${PROJECT_NAME}_ifopt_test test/composite_test.cc test/problem_test.cc)
  target_include_directories(${PROJECT_NAME}_ifopt_test PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>)
  target_link_libraries(${PROJECT_NAME}_ifopt_test PRIVATE ${PROJECT_NAME}_ifopt GTest::GTest GTest::Main)
  add_test(${PROJECT_NAME}_ifopt_test ${PROJECT_NAME}_ifopt_test)
endif()
