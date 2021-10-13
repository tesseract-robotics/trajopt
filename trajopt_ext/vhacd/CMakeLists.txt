cmake_minimum_required(VERSION 3.10.0)
project(vhacd VERSION 0.1.0 LANGUAGES CXX)

find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(trajopt_utils REQUIRED)
find_package(ros_industrial_cmake_boilerplate REQUIRED)
find_package(
  Bullet
  REQUIRED
  CONFIGS
  BulletConfig-float64.cmake
  BulletConfig.cmake)

if(NOT
   ${BULLET_DEFINITIONS}
   MATCHES
   ".*-DBT_USE_DOUBLE_PRECISION.*")
  message(
    WARNING "Bullet does not appear to be build with double precision, current definitions: ${BULLET_DEFINITIONS}")
endif()

include_directories(BEFORE "${BULLET_ROOT_DIR}/${BULLET_INCLUDE_DIRS}")
link_directories(BEFORE "${BULLET_ROOT_DIR}/${BULLET_LIBRARY_DIRS}")

# Load variable for clang tidy args, compiler options and cxx version
trajopt_variables()

option(NO_OPENCL "NO_OPENCL" OFF)
message("NO_OPENCL " ${NO_OPENCL})

include("${CMAKE_SOURCE_DIR}/scripts/cmake_common.cmake")

if(NOT NO_OPENCL)
  # include(FindOpenCL OPTIONAL)
  find_package(OpenCL)
endif()

if(NOT TARGET OpenMP::OpenMP_CXX)
  find_package(Threads REQUIRED)
  add_library(OpenMP::OpenMP_CXX IMPORTED INTERFACE)
  set_property(TARGET OpenMP::OpenMP_CXX PROPERTY INTERFACE_COMPILE_OPTIONS ${OpenMP_CXX_FLAGS})
  # Only works if the same flag is passed to the linker; use CMake 3.9+ otherwise (Intel, AppleClang)
  set_property(TARGET OpenMP::OpenMP_CXX PROPERTY INTERFACE_LINK_LIBRARIES ${OpenMP_CXX_FLAGS} Threads::Threads)

endif()

add_library(
  ${PROJECT_NAME}
  ${PROJECT_CPP_FILES}
  ${PROJECT_C_FILES}
  ${PROJECT_INC_FILES}
  ${PROJECT_INL_FILES}
  ${PROJECT_CL_FILES})
target_cxx_version(${PROJECT_NAME} PUBLIC VERSION ${TRAJOPT_CXX_VERSION})

if(OpenCL_FOUND)
  target_include_directories(${PROJECT_NAME} PRIVATE "${OpenCL_INCLUDE_DIR}")
  target_link_libraries(${PROJECT_NAME} PRIVATE "${OpenCL_LIBRARY}")
  target_compile_definitions(${PROJECT_NAME} PRIVATE -DOPENCL_FOUND=1)
  target_compile_definitions(${PROJECT_NAME} PRIVATE -DOPENCL_CL_FILES="${PROJECT_CL_FILES}")
endif()

target_link_libraries(${PROJECT_NAME} PUBLIC OpenMP::OpenMP_CXX trajopt::trajopt_utils Eigen3::Eigen)
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                  "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${BULLET_ROOT_DIR}/${BULLET_INCLUDE_DIRS}")

configure_package(NAMESPACE trajopt TARGETS ${PROJECT_NAME})

install(FILES ${PROJECT_INC_FILES} DESTINATION include/${PROJECT_NAME})
install(FILES ${PROJECT_INL_FILES} DESTINATION include/${PROJECT_NAME})
