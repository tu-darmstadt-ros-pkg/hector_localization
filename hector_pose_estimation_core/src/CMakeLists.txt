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
  measurements/poseupdate.cpp
)

rosbuild_add_library(hector_pose_estimation ${hector_pose_estimation_SOURCES})
