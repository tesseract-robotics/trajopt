find_package(GTest REQUIRED)

if(NOT TARGET GTest::GTest)
  add_library(GTest::GTest INTERFACE IMPORTED)
  set_target_properties(GTest::GTest PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${GTEST_INCLUDE_DIRS}")
  if(${GTEST_LIBRARIES})
    set_target_properties(GTest::GTest PROPERTIES INTERFACE_LINK_LIBRARIES "${GTEST_LIBRARIES}")
  else()
    if(MSVC)
      set_target_properties(GTest::GTest PROPERTIES INTERFACE_LINK_LIBRARIES "gtest.lib")
    else()
      set_target_properties(GTest::GTest PROPERTIES INTERFACE_LINK_LIBRARIES "libgtest.so")
    endif()
  endif()
endif()

if(NOT TARGET GTest::Main)
  add_library(GTest::Main INTERFACE IMPORTED)
  set_target_properties(GTest::Main PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${GTEST_INCLUDE_DIRS}")
  if(${GTEST_MAIN_LIBRARIES})
    set_target_properties(GTest::Main PROPERTIES INTERFACE_LINK_LIBRARIES "${GTEST_MAIN_LIBRARIES}")
  else()
    if(MSVC)
      set_target_properties(GTest::Main PROPERTIES INTERFACE_LINK_LIBRARIES "gtest_main.lib")
    else()
      set_target_properties(GTest::Main PROPERTIES INTERFACE_LINK_LIBRARIES "libgtest_main.so")
    endif()
  endif()
endif()

include(GoogleTest)

set(SCO_TEST_SOURCE
    unit.cpp
    small-problems-unit.cpp
    solver-interface-unit.cpp
    solver-utils-unit.cpp)

add_executable(${PROJECT_NAME}-test ${SCO_TEST_SOURCE})
target_link_libraries(
  ${PROJECT_NAME}-test
  GTest::GTest
  GTest::Main
  ${PROJECT_NAME})
if(osqp_FOUND)
  target_link_libraries(${PROJECT_NAME}-test osqp::osqpstatic)
endif()
target_compile_options(${PROJECT_NAME}-test PRIVATE ${TRAJOPT_COMPILE_OPTIONS_PRIVATE}
                                                    ${TRAJOPT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}-test PRIVATE ${TRAJOPT_COMPILE_DEFINITIONS}
                                                        TRAJOPT_IFOPT_DIR="${CMAKE_SOURCE_DIR}")
target_cxx_version(${PROJECT_NAME}-test PRIVATE VERSION ${TRAJOPT_CXX_VERSION})
# target_clang_tidy(${PROJECT_NAME}-test ENABLE ${TRAJOPT_ENABLE_CLANG_TIDY})
add_gtest_discover_tests(${PROJECT_NAME}-test)
if(HAVE_BPMPD)
  add_dependencies(${PROJECT_NAME}-test bpmpd_caller)
endif()

add_dependencies(${PROJECT_NAME}-test ${PROJECT_NAME})
add_dependencies(run_tests ${PROJECT_NAME}-test)
