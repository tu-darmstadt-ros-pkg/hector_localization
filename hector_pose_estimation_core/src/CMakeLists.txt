set(hector_pose_estimation_SOURCES
  pose_estimation.cpp
  parameters.cpp
  system.cpp
  system_model.cpp
  types.cpp
  system/generic_quaternion_system_model.cpp
  measurement.cpp
  measurement_model.cpp
  measurements/gravity.cpp
  measurements/zerorate.cpp
  measurements/heading.cpp
  measurements/magnetic.cpp
  measurements/height.cpp
  measurements/baro.cpp
  measurements/gps.cpp
  measurements/poseupdate.cpp
  measurements/rate.cpp
)

option(USE_RATE_SYSTEM_MODEL "Use system model with angular rates." OFF)
if(USE_RATE_SYSTEM_MODEL)
  add_definitions(-DUSE_RATE_SYSTEM_MODEL)
endif(USE_RATE_SYSTEM_MODEL)

rosbuild_add_library(hector_pose_estimation ${hector_pose_estimation_SOURCES})
