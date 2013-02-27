set(hector_pose_estimation_SOURCES
  global_reference.cpp
  pose_estimation.cpp
  parameters.cpp
  types.cpp
  state.cpp

  filter.cpp
  filter/ekf.cpp

  system.cpp
  system/generic_quaternion_system_model.cpp
  system/imu_model.cpp

  measurement.cpp
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

rosbuild_add_library(hector_pose_estimation ${hector_pose_estimation_SOURCES})
