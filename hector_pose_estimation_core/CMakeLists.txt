cmake_minimum_required(VERSION 2.8.3)
project(hector_pose_estimation_core)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS rostime rosconsole roscpp sensor_msgs geometry_msgs nav_msgs tf)

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES hector_pose_estimation
    CATKIN_DEPENDS rostime rosconsole roscpp sensor_msgs geometry_msgs nav_msgs tf
    DEPENDS Eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

add_library(hector_pose_estimation
  src/global_reference.cpp
  src/pose_estimation.cpp
  src/parameters.cpp
  src/types.cpp
  src/state.cpp

  src/filter.cpp
  src/filter/ekf.cpp

  src/system.cpp
  src/system/generic_quaternion_system_model.cpp
  src/system/ground_vehicle_model.cpp
  src/system/imu_model.cpp

  src/measurement.cpp
  src/measurements/gravity.cpp
  src/measurements/zerorate.cpp
  src/measurements/heading.cpp
  src/measurements/magnetic.cpp
  src/measurements/height.cpp
  src/measurements/baro.cpp
  src/measurements/gps.cpp
  src/measurements/poseupdate.cpp
  src/measurements/rate.cpp
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(hector_pose_estimation hector_worldmodel_msgs_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(hector_pose_estimation
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS hector_pose_estimation
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/hector_pose_estimation
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
# install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch/
# )
