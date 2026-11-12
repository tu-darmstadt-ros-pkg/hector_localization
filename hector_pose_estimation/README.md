## ROS API

<a id="NodeApi"></a>

### pose_estimation

`pose_estimation` is a node for estimating the 6DOF of a robot based
on the EKF of various sensor sources.

#### Subscribed Topics

- `fix`
  ([sensor_msgs/msg/NavSatFix](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/NavSatFix.html)):
  GPS data.
- `raw_imu`
  ([Sensor_msgs/msg/Imu](https://docs.ros.org/en/rolling/p/Sensor_msgs/msg/Imu.html)):
  Raw IMU data.
- `syscommand`
  ([std_msgs/msg/String](https://docs.ros.org/en/rolling/p/std_msgs/msg/String.html)):
  System command. If the string equals "reset" the map and robot pose
  are reset to their initial state.
- `magnetic`
  ([geometry_msgs/msg/Vector3Stamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/Vector3Stamped.html)):
  Magnetic orientation.
- `pressure_height`
  ([geometry_msgs/msg/PointStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/PointStamped.html)):
  Altitude from barometric pressure.
- `fix_velocity`
  ([geometry_msgs/msg/Vector3Stamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/Vector3Stamped.html)):
  Velocities from GPS data (North = x, East = y).
- `twistupdate`
  ([geometry_msgs/msg/TwistWithCovarianceStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/TwistWithCovarianceStamped.html)):
  Twist update.
- `poseupdate`
  ([geometry_msgs/msg/PoseWithCovarianceStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/PoseWithCovarianceStamped.html)):
  Estimated robot pose with a gaussian estimate of uncertainty.

#### Published Topics

- `pose`
  ([geometry_msgs/msg/PoseStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/PoseStamped.html)):
  Estimated pose of the robot based on the sensor fusing.
- `velocity`
  ([geometry_msgs/msg/Vector3Stamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/Vector3Stamped.html)):
  X, Y and Z velocities of the robot.
- `state`
  ([nav_msgs/msg/Odometry](https://docs.ros.org/en/rolling/p/nav_msgs/msg/Odometry.html)):
  Odometry computed based on robot's sensors.
- `fix/pose`
  ([geometry_msgs/msg/PoseStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/PoseStamped.html)):
  The estimated robot pose based on the GPS data.
- `imu`
  ([sensor_msgs/msg/Imu](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Imu.html)):
  Filtered IMU data.
- `angular_velocity_bias`
  ([geometry_msgs/msg/Vector3Stamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/Vector3Stamped.html)):
  TBD
- `tf`
  ([tf2_msgs/msg/TFMessage](https://docs.ros.org/en/rolling/p/tf2_msgs/msg/TFMessage.html)):
  TBD
- `linear_acceleration_bias`
  ([geometry_msgs/msg/Vector3Stamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/Vector3Stamped.html)):
  TBD
- `global`
  ([sensor_msgs/msg/NavSatFix](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/NavSatFix.html)):
  Global GPS coordinates.
- `euler`
  ([geometry_msgs/msg/Vector3Stamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/Vector3Stamped.html)):
  Euler angles.
