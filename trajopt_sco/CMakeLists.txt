cmake_minimum_required(VERSION 3.5.0)
project(trajopt_sco VERSION 0.1.0 LANGUAGES CXX)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")
find_package(GUROBI QUIET)
find_package(osqp QUIET)
find_package(qpOASES QUIET)
find_package(Eigen3 REQUIRED)
find_package(trajopt_utils REQUIRED)

list(FIND CMAKE_CXX_COMPILE_FEATURES cxx_std_11 CXX_FEATURE_FOUND)

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)

set(SCO_SOURCE_FILES
    src/solver_interface.cpp
    src/solver_utils.cpp
    src/modeling.cpp
    src/expr_ops.cpp
    src/expr_vec_ops.cpp
    src/optimizers.cpp
    src/modeling_utils.cpp
    src/num_diff.cpp
)

if (NOT APPLE)
  set (HAVE_BPMPD TRUE)
endif()

if (HAVE_BPMPD)
  add_executable(bpmpd_caller src/bpmpd_caller.cpp)

  if( CMAKE_SIZEOF_VOID_P EQUAL 8 ) # 64 bits
        set(BPMPD_LIBRARY "${CMAKE_CURRENT_SOURCE_DIR}/3rdpartylib/bpmpd_linux64.a")
  else()
        set(BPMPD_LIBRARY "${CMAKE_CURRENT_SOURCE_DIR}/3rdpartylib/bpmpd_linux32.a")
  endif()

  target_link_libraries(bpmpd_caller ${BPMPD_LIBRARY} -static)
  target_compile_options(bpmpd_caller PRIVATE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
  if(CXX_FEATURE_FOUND EQUAL "-1")
      target_compile_options(bpmpd_caller PRIVATE -std=c++11)
  else()
      target_compile_features(bpmpd_caller PRIVATE cxx_std_11)
  endif()
  target_compile_definitions(bpmpd_caller PRIVATE BPMPD_WORKING_DIR="${CMAKE_CURRENT_BINARY_DIR}")
  target_include_directories(bpmpd_caller PUBLIC
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
      "$<INSTALL_INTERFACE:include>")
  target_include_directories(bpmpd_caller SYSTEM PUBLIC
      $<TARGET_PROPERTY:trajopt::trajopt_utils,INTERFACE_INCLUDE_DIRECTORIES>)

  install(TARGETS bpmpd_caller DESTINATION bin)

  set_property(SOURCE src/solver_interface.cpp APPEND PROPERTY COMPILE_DEFINITIONS HAVE_BPMPD)
  list(APPEND SCO_SOURCE_FILES src/bpmpd_interface.cpp)
endif()

if (GUROBI_FOUND)
  set_property(SOURCE src/solver_interface.cpp APPEND PROPERTY COMPILE_DEFINITIONS HAVE_GUROBI)
  list(APPEND SCO_SOURCE_FILES src/gurobi_interface.cpp)
endif(GUROBI_FOUND)

if (osqp_FOUND)
  set_property(SOURCE src/solver_interface.cpp APPEND PROPERTY COMPILE_DEFINITIONS HAVE_OSQP)
  list(APPEND SCO_SOURCE_FILES src/osqp_interface.cpp)
endif()

if (qpOASES_FOUND)
  set_property(SOURCE src/solver_interface.cpp APPEND PROPERTY COMPILE_DEFINITIONS HAVE_QPOASES)
  list(APPEND SCO_SOURCE_FILES src/qpoases_interface.cpp)
endif()

add_library(${PROJECT_NAME} SHARED ${SCO_SOURCE_FILES})

if (GUROBI_FOUND)
  target_link_libraries(${PROJECT_NAME} PUBLIC ${GUROBI_LIBRARIES})
  target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${GUROBI_INCLUDE_DIR})
endif()
if (HAVE_BPMPD)
  install(FILES src/bpmpd.par DESTINATION "${CMAKE_INSTALL_PREFIX}/bin")

  target_link_libraries(${PROJECT_NAME} PUBLIC ${BPMPD_LIBRARY})
  target_compile_definitions(${PROJECT_NAME} PRIVATE BPMPD_CALLER="${CMAKE_INSTALL_PREFIX}/bin/bpmpd_caller")
endif()
if (osqp_FOUND)
  target_link_libraries(${PROJECT_NAME} PUBLIC osqp::osqpstatic)
endif()
if (qpOASES_FOUND)
  target_link_libraries(${PROJECT_NAME} PUBLIC ${qpOASES_LIBRARIES})
  target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${qpOASES_INCLUDE_DIRS})
endif()

target_link_libraries(${PROJECT_NAME} PUBLIC trajopt::trajopt_utils ${CMAKE_DL_LIBS} ${JSONCPP_LIBRARIES})
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME} PUBLIC -std=c++11)
else()
    target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_11)
endif()
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${JSONCPP_INCLUDE_DIRS})

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

install(FILES cmake/FindGUROBI.cmake cmake/FindqpOASES.cmake DESTINATION lib/cmake/${PROJECT_NAME})

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

if (ENABLE_TESTS)
  enable_testing()
  add_custom_target(run_tests ALL
      WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
      COMMAND ${CMAKE_CTEST_COMMAND} -C $<CONFIGURATION> -V)

  if (HAVE_BPMPD)
    # BPMPD Expects the caller to be in the install location so for
    # testing it must manually be copied so tests will run.
    add_custom_target(test_depend ALL
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_CURRENT_SOURCE_DIR}/src/bpmpd.par
            ${CMAKE_INSTALL_PREFIX}/bin/bpmpd.par
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_BINARY_DIR}/bpmpd_caller
            ${CMAKE_INSTALL_PREFIX}/bin/bpmpd_caller)
    add_dependencies(test_depend ${PROJECT_NAME})

    add_dependencies(run_tests test_depend)
  endif()

  add_subdirectory(test)
endif()
