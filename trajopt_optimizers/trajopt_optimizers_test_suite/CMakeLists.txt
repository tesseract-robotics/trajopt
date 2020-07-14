cmake_minimum_required(VERSION 3.5.0)
project(trajopt_optimizers_test_suite VERSION 0.1.0 LANGUAGES CXX)

find_package(console_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ifopt REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(trajopt_ifopt REQUIRED)
find_package(trajopt_utils REQUIRED)
find_package(trajopt_sqp REQUIRED)

###########
## Build ##
###########

add_library(${PROJECT_NAME} SHARED
    src/run_tests.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
    ifopt::ifopt_core
    console_bridge
    OsqpEigen::OsqpEigen
)
trajopt_target_compile_options(${PROJECT_NAME} PUBLIC)
trajopt_clang_tidy(${PROJECT_NAME})
list (APPEND PACKAGE_LIBRARIES ${PROJECT_NAME})


#############
## Install ##
#############

trajopt_configure_package(${PACKAGE_LIBRARIES})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
 )

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
