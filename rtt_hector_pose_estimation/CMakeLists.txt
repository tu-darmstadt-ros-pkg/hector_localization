cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_find_ros_package( rtt )
set( RTT_HINTS HINTS ${rtt_PACKAGE_PATH}/../install )
find_package(OROCOS-RTT 2.0.0 ${RTT_HINTS})
if (NOT OROCOS-RTT_FOUND)
  message(FATAL_ERROR "\n   RTT not found. Is the version correct? Use the CMAKE_PREFIX_PATH cmake or environment variable to point to the installation directory of RTT.")
else()
  include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)
  #add_definitions( -DRTT_COMPONENT )
endif()

option(USE_HECTOR_UAV_MSGS "Use hector_uav_msgs message types" ON)
if(USE_HECTOR_UAV_MSGS)
  rosbuild_find_ros_package(hector_uav_msgs)
  if(hector_uav_msgs_PACKAGE_PATH)
    message(STATUS "Found package hector_uav_msgs. Adding hector_uav_msgs support.")
    include_directories(${hector_uav_msgs_PACKAGE_PATH}/msg_gen/cpp/include)
    add_definitions(-DUSE_HECTOR_UAV_MSGS)
  else()
    message(STATUS "Package hector_uav_msgs not found. Disabled hector_uav_msgs support.")
    set(USE_HECTOR_UAV_MSGS OFF)
  endif()
endif()

add_subdirectory(src)
