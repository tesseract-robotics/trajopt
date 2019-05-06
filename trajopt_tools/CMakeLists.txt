cmake_minimum_required(VERSION 3.5.0)
project(trajopt_tools)

set(TRAJOPT_TOOLS_VERSION_MAJOR 0 CACHE STRING "major version" FORCE)
set(TRAJOPT_TOOLS_VERSION_MINOR 1 CACHE STRING "minor version" FORCE)
set(TRAJOPT_TOOLS_VERSION_PATCH 0 CACHE STRING "patch version" FORCE)
set(TRAJOPT_TOOLS_VERSION ${TRAJOPT_TOOLS_VERSION_MAJOR}.${TRAJOPT_TOOLS_VERSION_MINOR}.${TRAJOPT_TOOLS_VERSION_PATCH} CACHE STRING "version" FORCE)

find_package(vhacd REQUIRED)
find_package(Bullet REQUIRED)
find_package(trajopt_utils REQUIRED)
find_package(console_bridge REQUIRED)
find_package(Boost COMPONENTS system thread program_options REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(OpenMP)
if(NOT TARGET OpenMP::OpenMP_CXX)
    find_package(Threads REQUIRED)
    add_library(OpenMP::OpenMP_CXX IMPORTED INTERFACE)
    set_property(TARGET OpenMP::OpenMP_CXX
                 PROPERTY INTERFACE_COMPILE_OPTIONS ${OpenMP_CXX_FLAGS})
    # Only works if the same flag is passed to the linker; use CMake 3.9+ otherwise (Intel, AppleClang)
    set_property(TARGET OpenMP::OpenMP_CXX
                 PROPERTY INTERFACE_LINK_LIBRARIES ${OpenMP_CXX_FLAGS} Threads::Threads)

endif()

find_library(CONVEX_DECOMPOSITION_LIBRARY ConvexDecomposition HINTS ${CMAKE_INSTALL_PREFIX}/lib)
find_library(HACD_LIBRARY HACD HINTS ${CMAKE_INSTALL_PREFIX}/lib)

add_executable(convex_decomposition_hacd_node src/convex_decomposition_hacd.cpp)
target_link_libraries(convex_decomposition_hacd_node trajopt_utils vhacd console_bridge Eigen3::Eigen ${BULLET_LIBRARIES} ${CONVEX_DECOMPOSITION_LIBRARY} ${HACD_LIBRARY}  ${Boost_LIBRARIES})
target_compile_options(convex_decomposition_hacd_node PRIVATE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
target_compile_features(convex_decomposition_hacd_node PRIVATE cxx_std_11)
target_include_directories(convex_decomposition_hacd_node SYSTEM PRIVATE
    ${BULLET_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS})

add_executable(convex_decomposition_vhacd_node src/convex_decomposition_vhacd.cpp)
target_link_libraries(convex_decomposition_vhacd_node trajopt_utils vhacd console_bridge Eigen3::Eigen ${BULLET_LIBRARIES})
target_compile_options(convex_decomposition_vhacd_node PRIVATE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
target_compile_features(convex_decomposition_vhacd_node PRIVATE cxx_std_11)
target_include_directories(convex_decomposition_vhacd_node SYSTEM PRIVATE
    ${BULLET_INCLUDE_DIRS})

# Mark executables and/or libraries for installation
install(TARGETS convex_decomposition_hacd_node convex_decomposition_vhacd_node DESTINATION bin)

install(FILES package.xml DESTINATION share/${PROJECT_NAME})
