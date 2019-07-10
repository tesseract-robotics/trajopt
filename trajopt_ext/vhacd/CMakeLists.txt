cmake_minimum_required(VERSION 3.5.0)
project(vhacd VERSION 0.1.0 LANGUAGES CXX)

find_package(Bullet REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)

list(FIND CMAKE_CXX_COMPILE_FEATURES cxx_std_11 CXX_FEATURE_FOUND)

option(NO_OPENCL "NO_OPENCL" OFF)
message("NO_OPENCL " ${NO_OPENCL})

include("${CMAKE_SOURCE_DIR}/scripts/cmake_common.cmake")

if (NOT NO_OPENCL)
  #include(FindOpenCL OPTIONAL)
  find_package(OpenCL)
endif()

if(NOT TARGET OpenMP::OpenMP_CXX)
    find_package(Threads REQUIRED)
    add_library(OpenMP::OpenMP_CXX IMPORTED INTERFACE)
    set_property(TARGET OpenMP::OpenMP_CXX
                 PROPERTY INTERFACE_COMPILE_OPTIONS ${OpenMP_CXX_FLAGS})
    # Only works if the same flag is passed to the linker; use CMake 3.9+ otherwise (Intel, AppleClang)
    set_property(TARGET OpenMP::OpenMP_CXX
                 PROPERTY INTERFACE_LINK_LIBRARIES ${OpenMP_CXX_FLAGS} Threads::Threads)

endif()

add_library(vhacd SHARED ${PROJECT_CPP_FILES} ${PROJECT_C_FILES} ${PROJECT_INC_FILES} ${PROJECT_INL_FILES} ${PROJECT_CL_FILES})
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(vhacd PRIVATE -std=c++11)
else()
    target_compile_features(vhacd PRIVATE cxx_std_11)
endif()

if (OpenCL_FOUND)
    target_include_directories(${PROJECT_NAME} PRIVATE "${OpenCL_INCLUDE_DIR}")
    target_link_libraries(${PROJECT_NAME} PRIVATE "${OpenCL_LIBRARY}")
    target_compile_options(${PROJECT_NAME} PRIVATE -w)
    if(CXX_FEATURE_FOUND EQUAL "-1")
        target_compile_options(${PROJECT_NAME} PRIVATE -std=c++11)
    else()
        target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_11)
    endif()
    target_compile_definitions(${PROJECT_NAME} PRIVATE -DOPENCL_FOUND=1 )
    target_compile_definitions(${PROJECT_NAME} PRIVATE -DOPENCL_CL_FILES="${PROJECT_CL_FILES}" )
endif()


target_link_libraries(${PROJECT_NAME} PUBLIC OpenMP::OpenMP_CXX)
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${BULLET_INCLUDE_DIRS})

list (APPEND VHACD_LIBRARIES ${PROJECT_NAME})

# Mark executables and/or libraries for installation
install(TARGETS ${VHACD_LIBRARIES} EXPORT ${PROJECT_NAME}-targets DESTINATION lib)
install(EXPORT ${PROJECT_NAME}-targets DESTINATION lib/cmake/${PROJECT_NAME})

install(FILES ${PROJECT_INC_FILES} DESTINATION include/${PROJECT_NAME})
install(FILES ${PROJECT_INL_FILES} DESTINATION include/${PROJECT_NAME})

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
