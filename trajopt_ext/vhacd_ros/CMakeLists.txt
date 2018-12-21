cmake_minimum_required(VERSION 3.0.0 FATAL_ERROR)
project(vhacd_ros)

add_compile_options(-std=c++11 -w)

find_package(catkin REQUIRED COMPONENTS bullet3_ros)

option(NO_OPENCL "NO_OPENCL" OFF)
option(NO_OPENMP "NO_OPENMP" OFF)

message("NO_OPENCL " ${NO_OPENCL})
message("NO_OPENMP " ${NO_OPENMP})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vhacd
  CATKIN_DEPENDS bullet3_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

include("${CMAKE_SOURCE_DIR}/scripts/cmake_common.cmake")

if (NOT NO_OPENCL)
  #include(FindOpenCL OPTIONAL)
  find_package(OpenCL)
endif()

if (NOT NO_OPENMP)
  #include(FindOpenMP OPTIONAL)
  find_package(OpenMP)
endif()

if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

add_library(vhacd ${PROJECT_CPP_FILES} ${PROJECT_C_FILES} ${PROJECT_INC_FILES} ${PROJECT_INL_FILES} ${PROJECT_CL_FILES})

if (OpenCL_FOUND)
    target_include_directories(vhacd PRIVATE "${OpenCL_INCLUDE_DIR}")
    target_link_libraries(vhacd PRIVATE "${OpenCL_LIBRARY}")
    target_compile_definitions(vhacd PRIVATE -DOPENCL_FOUND=1 )
    target_compile_definitions(vhacd PRIVATE -DOPENCL_CL_FILES="${PROJECT_CL_FILES}" )
endif()

# Mark executables and/or libraries for installation
install(TARGETS vhacd
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES ${PROJECT_INC_FILES} DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES ${PROJECT_INL_FILES} DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
