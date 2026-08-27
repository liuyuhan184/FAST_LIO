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
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
  include(ProcessorCount)
  ProcessorCount(N)
  message(STATUS "Processor count: ${N}")
  if(N GREATER 4)
    add_definitions(-DMP_EN -DMP_PROC_NUM=3)
  elseif(N GREATER 3)
    add_definitions(-DMP_EN -DMP_PROC_NUM=2)
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)
if(OpenMP_CXX_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
endif()

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
