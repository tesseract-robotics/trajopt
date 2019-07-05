cmake_minimum_required(VERSION 3.5.0)
project(vhacd)

set(VHACD_VERSION_MAJOR 0 CACHE STRING "major version" FORCE)
set(VHACD_VERSION_MINOR 1 CACHE STRING "minor version" FORCE)
set(VHACD_VERSION_PATCH 0 CACHE STRING "patch version" FORCE)
set(VHACD_VERSION ${VHACD_VERSION_MAJOR}.${VHACD_VERSION_MINOR}.${VHACD_VERSION_PATCH} CACHE STRING "version" FORCE)

find_package(Bullet REQUIRED)
find_package(Eigen3 REQUIRED)

list(FIND CMAKE_CXX_COMPILE_FEATURES cxx_std_11 CXX_FEATURE_FOUND)

option(NO_OPENCL "NO_OPENCL" OFF)
option(NO_OPENMP "NO_OPENMP" OFF)

message("NO_OPENCL " ${NO_OPENCL})
message("NO_OPENMP " ${NO_OPENMP})

include("${CMAKE_SOURCE_DIR}/scripts/cmake_common.cmake")

if (NOT NO_OPENCL)
  #include(FindOpenCL OPTIONAL)
  find_package(OpenCL)
endif()

if (NOT NO_OPENMP)
  #include(FindOpenMP OPTIONAL)
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

if(OPENMP_FOUND)
#    target_compile_options(${PROJECT_NAME} PRIVATE ${OpenMP_C_FLAGS} ${OpenMP_CXX_FLAGS} ${OpenMP_EXE_LINKER_FLAGS})
    target_link_libraries(${PROJECT_NAME} PUBLIC OpenMP::OpenMP_CXX)

endif()

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
set(CMAKE_INSTALL_INCLUDEDIR include)
set(CMAKE_INSTALL_LIBDIR lib)
set(CMAKE_INSTALL_BINDIR bin)

include(CMakePackageConfigHelpers)
configure_package_config_file(cmake/${PROJECT_NAME}-config.cmake.in ${PROJECT_NAME}-config.cmake
  INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}"
  PATH_VARS CMAKE_INSTALL_INCLUDEDIR CMAKE_INSTALL_LIBDIR
  NO_CHECK_REQUIRED_COMPONENTS_MACRO)

write_basic_package_version_file(${PROJECT_NAME}-config-version.cmake
  VERSION ${VHACD_VERSION} COMPATIBILITY ExactVersion)

install(FILES
  "${CMAKE_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
  "${CMAKE_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}")

set(VHACD_PKGCONFIG_DESCRIPTION "Libraries for performing collision checking")
configure_file(cmake/${PROJECT_NAME}.pc.in ${PROJECT_NAME}.pc @ONLY)
install(FILES "${CMAKE_BINARY_DIR}/${PROJECT_NAME}.pc"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/pkgconfig")
