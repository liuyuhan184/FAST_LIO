cmake_minimum_required(VERSION 3.5)
project(fast_lio)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(ENABLE_LIVOX_SUPPORT "Build support for Livox CustomMsg input" ON)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-fexceptions -pthread)
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

message(STATUS "Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")
include(ProcessorCount)
ProcessorCount(FAST_LIO_PROCESSOR_COUNT)
if(NOT FAST_LIO_PROCESSOR_COUNT OR FAST_LIO_PROCESSOR_COUNT LESS 1)
  set(FAST_LIO_PROCESSOR_COUNT 1)
endif()
message(STATUS "Processor count: ${FAST_LIO_PROCESSOR_COUNT}")

# 0 selects a conservative automatic value.  The matching loop is memory
# intensive, so using every Orin core is slower and also starves the LiDAR
# driver/MAVROS.  Three workers is the original FAST-LIO upper bound on x86 and
# works well on the 8-core Orin NX.
set(FAST_LIO_MP_THREADS 0 CACHE STRING
    "OpenMP workers for FAST-LIO matching (0: auto, 1: serial)")
if(FAST_LIO_MP_THREADS LESS 0)
  message(FATAL_ERROR "FAST_LIO_MP_THREADS must be >= 0")
endif()
set(FAST_LIO_EFFECTIVE_MP_THREADS ${FAST_LIO_MP_THREADS})
if(FAST_LIO_EFFECTIVE_MP_THREADS EQUAL 0)
  if(FAST_LIO_PROCESSOR_COUNT GREATER 4)
    set(FAST_LIO_EFFECTIVE_MP_THREADS 3)
  elseif(FAST_LIO_PROCESSOR_COUNT GREATER 2)
    set(FAST_LIO_EFFECTIVE_MP_THREADS 2)
  else()
    set(FAST_LIO_EFFECTIVE_MP_THREADS 1)
  endif()
endif()

find_package(OpenMP REQUIRED)
if(FAST_LIO_EFFECTIVE_MP_THREADS GREATER 1)
  add_definitions(
    -DMP_EN
    -DMP_PROC_NUM=${FAST_LIO_EFFECTIVE_MP_THREADS}
  )
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()
message(STATUS "FAST-LIO matching threads: ${FAST_LIO_EFFECTIVE_MP_THREADS}")

find_package(PythonLibs REQUIRED)

set(FAST_LIO_CATKIN_COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  pcl_conversions
  tf
  visualization_msgs
  message_generation
  eigen_conversions
)
if(ENABLE_LIVOX_SUPPORT)
  list(APPEND FAST_LIO_CATKIN_COMPONENTS livox_ros_driver)
  add_definitions(-DFAST_LIO_HAS_LIVOX)
  message(STATUS "Livox CustomMsg support: ON")
else()
  message(STATUS "Livox CustomMsg support: OFF")
endif()

find_package(catkin REQUIRED COMPONENTS ${FAST_LIO_CATKIN_COMPONENTS})
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  include
)

add_message_files(
  FILES
  Pose6D.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

set(FAST_LIO_CATKIN_DEPENDS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  pcl_conversions
  tf
  visualization_msgs
  message_runtime
  eigen_conversions
)
if(ENABLE_LIVOX_SUPPORT)
  list(APPEND FAST_LIO_CATKIN_DEPENDS livox_ros_driver)
endif()

catkin_package(
  CATKIN_DEPENDS ${FAST_LIO_CATKIN_DEPENDS}
  DEPENDS EIGEN3 PCL
)

add_executable(
  fastlio_mapping
  src/laserMapping.cpp
  include/ikd-Tree/ikd_Tree.cpp
  src/preprocess.cpp
)
add_dependencies(
  fastlio_mapping
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  fastlio_mapping
  OpenMP::OpenMP_CXX
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${PYTHON_LIBRARIES}
)

install(
  TARGETS fastlio_mapping
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
  DIRECTORY launch config rviz_cfg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
