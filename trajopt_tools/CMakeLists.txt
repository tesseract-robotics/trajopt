cmake_minimum_required(VERSION 3.10.0)
project(trajopt_tools VERSION 0.1.0 LANGUAGES CXX)

if(WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

find_package(vhacd REQUIRED)
find_package(Bullet REQUIRED)
find_package(trajopt_utils REQUIRED)
find_package(console_bridge REQUIRED)
find_package(Boost COMPONENTS system thread program_options REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(cmake_common_scripts REQUIRED)

# Load variable for clang tidy args, compiler options and cxx version
trajopt_variables()

find_library(CONVEX_DECOMPOSITION_LIBRARY ConvexDecomposition HINTS ${CMAKE_INSTALL_PREFIX}/lib)
find_library(HACD_LIBRARY HACD HINTS ${CMAKE_INSTALL_PREFIX}/lib)

add_executable(convex_decomposition_hacd_node src/convex_decomposition_hacd.cpp)
target_link_libraries(convex_decomposition_hacd_node trajopt::trajopt_utils trajopt::vhacd console_bridge ${BULLET_LIBRARIES} ${CONVEX_DECOMPOSITION_LIBRARY} ${HACD_LIBRARY}  ${Boost_LIBRARIES})
target_compile_options(convex_decomposition_hacd_node PRIVATE ${TRAJOPT_COMPILE_OPTIONS_PRIVATE})
target_compile_definitions(convex_decomposition_hacd_node PUBLIC ${TRAJOPT_COMPILE_DEFINITIONS})
target_cxx_version(convex_decomposition_hacd_node PUBLIC VERSION ${TRAJOPT_CXX_VERSION})
target_clang_tidy(convex_decomposition_hacd_node ARGUMENTS ${TRAJOPT_CLANG_TIDY_ARGS} ENABLE ${TRAJOPT_ENABLE_CLANG_TIDY})
target_include_directories(convex_decomposition_hacd_node SYSTEM PRIVATE
    ${EIGEN3_INCLUDE_DIRS}
    ${BULLET_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS})

add_executable(convex_decomposition_vhacd_node src/convex_decomposition_vhacd.cpp)
target_link_libraries(convex_decomposition_vhacd_node trajopt::trajopt_utils trajopt::vhacd console_bridge ${BULLET_LIBRARIES})
target_compile_options(convex_decomposition_vhacd_node PRIVATE ${TRAJOPT_COMPILE_OPTIONS_PRIVATE})
target_compile_definitions(convex_decomposition_vhacd_node PUBLIC ${TRAJOPT_COMPILE_DEFINITIONS})
target_cxx_version(convex_decomposition_vhacd_node PUBLIC VERSION ${TRAJOPT_CXX_VERSION})
target_clang_tidy(convex_decomposition_vhacd_node ARGUMENTS ${TRAJOPT_CLANG_TIDY_ARGS} ENABLE ${TRAJOPT_ENABLE_CLANG_TIDY})
target_include_directories(convex_decomposition_vhacd_node SYSTEM PRIVATE
    ${EIGEN3_INCLUDE_DIRS}
    ${BULLET_INCLUDE_DIRS})

configure_package(NAMESPACE trajopt TARGETS convex_decomposition_hacd_node convex_decomposition_vhacd_node)

