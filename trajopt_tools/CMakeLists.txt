cmake_minimum_required(VERSION 3.5.0)
project(trajopt_tools VERSION 0.1.0 LANGUAGES CXX)

find_package(vhacd REQUIRED)
find_package(Bullet REQUIRED)
find_package(trajopt_utils REQUIRED)
find_package(console_bridge REQUIRED)
find_package(Boost COMPONENTS system thread program_options REQUIRED)
find_package(Eigen3 REQUIRED)

find_library(CONVEX_DECOMPOSITION_LIBRARY ConvexDecomposition HINTS ${CMAKE_INSTALL_PREFIX}/lib)
find_library(HACD_LIBRARY HACD HINTS ${CMAKE_INSTALL_PREFIX}/lib)

add_executable(convex_decomposition_hacd_node src/convex_decomposition_hacd.cpp)
target_link_libraries(convex_decomposition_hacd_node trajopt::trajopt_utils trajopt::vhacd console_bridge ${BULLET_LIBRARIES} ${CONVEX_DECOMPOSITION_LIBRARY} ${HACD_LIBRARY}  ${Boost_LIBRARIES})
trajopt_target_compile_options(convex_decomposition_hacd_node PRIVATE)
trajopt_clang_tidy(convex_decomposition_hacd_node)
target_include_directories(convex_decomposition_hacd_node SYSTEM PRIVATE
    ${EIGEN3_INCLUDE_DIRS}
    ${BULLET_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS})

add_executable(convex_decomposition_vhacd_node src/convex_decomposition_vhacd.cpp)
target_link_libraries(convex_decomposition_vhacd_node trajopt::trajopt_utils trajopt::vhacd console_bridge ${BULLET_LIBRARIES})
trajopt_target_compile_options(convex_decomposition_vhacd_node PRIVATE)
trajopt_clang_tidy(convex_decomposition_vhacd_node)
target_include_directories(convex_decomposition_vhacd_node SYSTEM PRIVATE
    ${EIGEN3_INCLUDE_DIRS}
    ${BULLET_INCLUDE_DIRS})

trajopt_configure_package(convex_decomposition_hacd_node convex_decomposition_vhacd_node)

