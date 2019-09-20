cmake_minimum_required(VERSION 3.5.0)
project(trajopt_tools VERSION 0.1.0 LANGUAGES CXX)

find_package(vhacd REQUIRED)
find_package(Bullet REQUIRED)
find_package(trajopt_utils REQUIRED)
find_package(console_bridge REQUIRED)
find_package(Boost COMPONENTS system thread program_options REQUIRED)
find_package(Eigen3 REQUIRED)

list(FIND CMAKE_CXX_COMPILE_FEATURES cxx_std_11 CXX_FEATURE_FOUND)

find_library(CONVEX_DECOMPOSITION_LIBRARY ConvexDecomposition HINTS ${CMAKE_INSTALL_PREFIX}/lib)
find_library(HACD_LIBRARY HACD HINTS ${CMAKE_INSTALL_PREFIX}/lib)

add_executable(convex_decomposition_hacd_node src/convex_decomposition_hacd.cpp)
target_link_libraries(convex_decomposition_hacd_node trajopt::trajopt_utils vhacd console_bridge ${BULLET_LIBRARIES} ${CONVEX_DECOMPOSITION_LIBRARY} ${HACD_LIBRARY}  ${Boost_LIBRARIES})
target_compile_options(convex_decomposition_hacd_node PRIVATE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(convex_decomposition_hacd_node PRIVATE -std=c++11)
else()
    target_compile_features(convex_decomposition_hacd_node PRIVATE cxx_std_11)
endif()
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(convex_decomposition_hacd_node PRIVATE -mno-avx)
  else()
    message(WARNING "Non-GNU compiler detected. If using AVX instructions, Eigen alignment issues may result.")
  endif()
target_include_directories(convex_decomposition_hacd_node SYSTEM PRIVATE
    ${EIGEN3_INCLUDE_DIRS}
    ${BULLET_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS})

add_executable(convex_decomposition_vhacd_node src/convex_decomposition_vhacd.cpp)
target_link_libraries(convex_decomposition_vhacd_node trajopt::trajopt_utils vhacd console_bridge ${BULLET_LIBRARIES})
target_compile_options(convex_decomposition_vhacd_node PRIVATE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(convex_decomposition_vhacd_node PRIVATE -std=c++11)
else()
    target_compile_features(convex_decomposition_vhacd_node PRIVATE cxx_std_11)
endif()
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(convex_decomposition_vhacd_node PRIVATE -mno-avx)
  else()
    message(WARNING "Non-GNU compiler detected. If using AVX instructions, Eigen alignment issues may result.")
  endif()
target_include_directories(convex_decomposition_vhacd_node SYSTEM PRIVATE
    ${EIGEN3_INCLUDE_DIRS}
    ${BULLET_INCLUDE_DIRS})

# Mark executables and/or libraries for installation
install(TARGETS convex_decomposition_hacd_node convex_decomposition_vhacd_node DESTINATION bin)

install(FILES package.xml DESTINATION share/${PROJECT_NAME})
