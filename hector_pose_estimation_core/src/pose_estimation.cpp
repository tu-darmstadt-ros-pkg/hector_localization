//=================================================================================================
// Copyright (c) 2011, Johannes Meyer and Martin Nowara, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include <hector_pose_estimation/pose_estimation.h>
#include <hector_pose_estimation/system/imu_input.h>

namespace hector_pose_estimation {

namespace {
  static PoseEstimation *the_instance = 0;
}

PoseEstimation::PoseEstimation(const SystemPtr& system)
  : system_(system)
  , state_is_dirty_(true)
  , covariance_is_dirty_(true)
  , status_()
  , measurement_status_()
  , rate_(new Rate("rate"))
  , gravity_(new Gravity ("gravity"))
  , zerorate_(new ZeroRate("zerorate"))
//  , heading_(new Heading("heading"))
{
  if (!the_instance) the_instance = this;

  world_frame_ = "world";
  nav_frame_ = "nav";
  base_frame_ = "base_link";
  stabilized_frame_ = "base_stabilized";
  footprint_frame_ = "base_footprint";
  // position_frame_ = "base_position";
  alignment_time_ = 0.0;

  parameters().add("world_frame", world_frame_);
  parameters().add("nav_frame", nav_frame_);
  parameters().add("base_frame", base_frame_);
  parameters().add("stabilized_frame", stabilized_frame_);
  parameters().add("footprint_frame", footprint_frame_);
  parameters().add("position_frame", position_frame_);
  parameters().add(global_reference_.parameters());
  parameters().add("alignment_time", alignment_time_);

  // add default measurements
  addMeasurement(rate_);
  addMeasurement(gravity_);
  addMeasurement(zerorate_);
//  addMeasurement(heading_);
}

PoseEstimation::~PoseEstimation()
{
  cleanup();
}

PoseEstimation *PoseEstimation::Instance() {
  if (!the_instance) the_instance = new PoseEstimation();
  return the_instance;
}

bool PoseEstimation::init()
{
  // initialize global reference
  globalReference()->updated();

  // check if system is initialized
  if (!system_) return false;

  // initialize system
  if (!system_->init()) return false;

  // initialize all measurements
  for(Measurements::iterator it = measurements_.begin(); it != measurements_.end(); ++it)
    if (!(*it)->init()) return false;

  // reset (or initialize) filter and measurements
  reset();

  return true;
}

void PoseEstimation::cleanup()
{
  // delete filter instance
  if (filter_) filter_.reset();

  // cleanup system
  system_->cleanup();

  // cleanup all measurements
  for(Measurements::iterator it = measurements_.begin(); it != measurements_.end(); ++it) (*it)->cleanup();
}

void PoseEstimation::reset()
{
  // reset extended Kalman filter
  if (filter_) cleanup();

  // check if system is initialized
  if (!system_) return;

  // initialize new filter
  filter_.reset(new BFL::ExtendedKalmanFilter(system_->getPrior()));
  updated();

  // set initial status
  alignment_start_ = ros::Time();
  if (alignment_time_ > 0) {
    status_ = STATE_ALIGNMENT;
  } else {
    status_ = static_cast<SystemStatus>(0);
  }
  measurement_status_ = 0;

  // reset system and all measurements
  system_->reset();
  for(Measurements::iterator it = measurements_.begin(); it != measurements_.end(); ++it) (*it)->reset();
}

void PoseEstimation::update(const SystemInput& input, ros::Time new_timestamp)
{
  ros::Duration dt;

  // check if system is initialized
  if (!system_) return;

  // set input and calculate time diff dt
  system_->setInput(input);
  if (!timestamp_.isZero()) dt = new_timestamp - timestamp_;
  timestamp_ = new_timestamp;

  // do the update step
  update(dt.toSec());
}

void PoseEstimation::update(double dt)
{
  // check dt
  if (dt < -1.0)
    reset();
  else if (dt < 0.0)
    return;

  // check if system and filter is initialized
  if (!system_ || !filter_) return;

  // filter rate measurement first
  const ImuInput *imu = dynamic_cast<const ImuInput *>(&system_->getInput());
#ifdef USE_RATE_SYSTEM_MODEL
  if (imu && rate_) {
    ROS_DEBUG("Updating with measurement model %s", rate_->getName().c_str());
    rate_->update(*this, Rate::Update(imu->getRate()));
  }
#endif // USE_RATE_SYSTEM_MODEL

  // time update step
  system_->update(*this, dt);
  updateSystemStatus(system_->getStatusFlags(), STATE_ROLLPITCH | STATE_YAW | STATE_XY_POSITION | STATE_XY_VELOCITY | STATE_Z_POSITION | STATE_Z_VELOCITY);

  // iterate through measurements and do the measurement update steps
  SystemStatus measurement_status = 0;

  for(Measurements::iterator it = measurements_.begin(); it != measurements_.end(); ++it) {
    MeasurementPtr measurement = *it;
    if (!measurement->active(getSystemStatus())) continue;

    // special updates
    if (measurement == rate_) continue;

    if (imu && measurement == gravity_) {
      ROS_DEBUG("Updating with pseudo measurement model %s", gravity_->getName().c_str());
      gravity_->update(*this, Gravity::Update(imu->getAccel()));
      continue;
    }

    if (imu && measurement == zerorate_) {
      ROS_DEBUG("Updating with pseudo measurement model %s", zerorate_->getName().c_str());
      zerorate_->update(*this, ZeroRate::Update(imu->getRate()[2]));
      continue;
    }

//    if (measurement == &heading_) {
//      ROS_DEBUG("Updating with pseudo measurement model %s", heading_.getName().c_str());
//      Heading::Update y(0.0);
//      heading_.update(*this, y);
//      continue;
//    }

    // skip all other measurements during alignment
    if (inSystemStatus(STATE_ALIGNMENT)) continue;

    // process the incoming queue
    measurement->process(*this);
    measurement_status |= measurement->getStatusFlags();
    measurement->increase_timer(dt);
  }

  // update the measurement status
  setMeasurementStatus(measurement_status);

//  // pseudo updates
//  if (gravity_.active(getSystemStatus())) {
//    ROS_DEBUG("Updating with pseudo measurement model %s", gravity_.getName().c_str());
//    Gravity::Update y(imu->getAccel());
//    // gravity_.enable();
//    if (gravity_.update(*this, y)) measurement_status |= gravity_.getStatusFlags();
//  } else {
//    // gravity_.disable();
//  }

//  if (zerorate_.active(getSystemStatus())) {
//    ROS_DEBUG("Updating with pseudo measurement model %s", zerorate_.getName().c_str());
//    ZeroRate::Update y(imu->getRate());
//    // zerorate_.enable();
//    if (zerorate_.update(*this, y)) measurement_status |= zerorate_.getStatusFlags();
//  } else {
//    // zerorate_.disable();
//  }

//  std::cout << "x_est = [" << getState().transpose() << "]" << std::endl;
//  std::cout << "P_est = [" << filter_->PostGet()->CovarianceGet() << "]" << std::endl;

  // switch overall system state
  if (inSystemStatus(STATE_ALIGNMENT)) {
    if (alignment_start_.isZero()) alignment_start_ = timestamp_;
    if ((timestamp_ - alignment_start_).toSec() >= alignment_time_) {
      updateSystemStatus(STATE_DEGRADED, STATE_ALIGNMENT);
    }
  } else if (inSystemStatus(STATE_ROLLPITCH | STATE_YAW | STATE_XY_POSITION | STATE_Z_POSITION)) {
    // if (!(status_ & STATE_READY) && (status_ & STATE_ROLLPITCH) && (status_ & STATE_YAW) && (status_ & STATE_XY_POSITION) && (status_ & STATE_Z_POSITION)) {
    updateSystemStatus(STATE_READY, STATE_DEGRADED);
  } else {
    // if ((status_ & STATE_READY) && !((status_ & STATE_ROLLPITCH) && (status_ & STATE_YAW) && (status_ & STATE_XY_POSITION) && (status_ & STATE_Z_POSITION))) {
    updateSystemStatus(STATE_DEGRADED, STATE_READY);
  }
}

void PoseEstimation::updated() {
  state_is_dirty_ = covariance_is_dirty_ = true;
  setState(system_->limitState(getState()));
}

const SystemPtr& PoseEstimation::setSystem(const SystemPtr& new_system) {
  if (system_) {
    cleanup();
    system_.reset();
  }

  system_ = new_system;
  return system_;
}

const SystemPtr& PoseEstimation::setSystem(System *new_system) {
  return setSystem(SystemPtr(new_system));
}

const SystemModel *PoseEstimation::getSystemModel() const {
  if (!system_) return 0;
  return system_->getModel();
}

const SystemPtr& PoseEstimation::getSystem() const {
  return system_;
}

const MeasurementPtr& PoseEstimation::addMeasurement(const MeasurementPtr& measurement) {
  measurements_.push_back(measurement);
  return measurement;
}

const MeasurementPtr& PoseEstimation::addMeasurement(const std::string& name, const MeasurementPtr& measurement) {
	measurement->setName(name);
	return addMeasurement(measurement);
}

const MeasurementPtr& PoseEstimation::addMeasurement(Measurement *measurement) {
  return addMeasurement(MeasurementPtr(measurement));
}

MeasurementPtr PoseEstimation::getMeasurement(const std::string &name) const {
  for(Measurements::const_iterator it = measurements_.begin(); it != measurements_.end(); ++it) {
    if ((*it)->getName() == name) return *it;
  }
  return MeasurementPtr();
}

const StateVector& PoseEstimation::getState() {
  if (state_is_dirty_) {
    state_ = filter_->PostGet()->ExpectedValueGet();
    state_is_dirty_ = false;
  }
  return state_;
}

const StateCovariance& PoseEstimation::getCovariance() {
  if (covariance_is_dirty_) {
    covariance_ = filter_->PostGet()->CovarianceGet();
    covariance_is_dirty_ = false;
  }
  return covariance_;
}

void PoseEstimation::setState(const StateVector& state) {
  filter_->PostGet()->ExpectedValueSet(state);
  state_ = state;
  state_is_dirty_ = false;
}

void PoseEstimation::setCovariance(const StateCovariance& covariance) {
  filter_->PostGet()->CovarianceSet(covariance);
  covariance_is_dirty_ = true;
}


SystemStatus PoseEstimation::getSystemStatus() const {
  return status_;
}

SystemStatus PoseEstimation::getMeasurementStatus() const {
  return measurement_status_;
}

bool PoseEstimation::inSystemStatus(SystemStatus test_status) const {
  return (getSystemStatus() & test_status) == test_status;
}

bool PoseEstimation::setSystemStatus(SystemStatus new_status) {
  if (status_callback_ && !status_callback_(new_status)) return false;

  SystemStatus set = new_status & ~status_;
  SystemStatus cleared = status_ & ~new_status;
  if (set)     ROS_INFO_STREAM("Set system state " << getSystemStatusString(set));
  if (cleared) ROS_INFO_STREAM("Cleared system state " << getSystemStatusString(cleared));

  status_ = new_status;
  return true;
}

bool PoseEstimation::setMeasurementStatus(SystemStatus new_measurement_status) {
  SystemStatus set = new_measurement_status & ~measurement_status_;
  SystemStatus cleared = measurement_status_ & ~new_measurement_status;
  if (set)     ROS_INFO_STREAM("Set measurement state " << getSystemStatusString(set));
  if (cleared) ROS_INFO_STREAM("Cleared measurement state " << getSystemStatusString(cleared));

  measurement_status_ = new_measurement_status;
  return true;
}

bool PoseEstimation::updateSystemStatus(SystemStatus set, SystemStatus clear) {
  return setSystemStatus((status_ & ~clear) | set);
}

bool PoseEstimation::updateMeasurementStatus(SystemStatus set, SystemStatus clear) {
  return setMeasurementStatus((measurement_status_ & ~clear) | set);
}

void PoseEstimation::setSystemStatusCallback(SystemStatusCallback callback) {
  status_callback_ = callback;
}

ros::Time PoseEstimation::getTimestamp() const {
  return timestamp_;
}

void PoseEstimation::setTimestamp(ros::Time timestamp) {
  timestamp_ = timestamp;
}

void PoseEstimation::getHeader(std_msgs::Header& header) {
  header.stamp = timestamp_;
  header.frame_id = nav_frame_;
}

void PoseEstimation::getState(nav_msgs::Odometry& state, bool with_covariances) {
  getHeader(state.header);
  getPose(state.pose.pose);
  getVelocity(state.twist.twist.linear);
  getRate(state.twist.twist.angular);
  state.child_frame_id = base_frame_;

  if (with_covariances) {
    double qw = state_(QUATERNION_W);
    double qx = state_(QUATERNION_X);
    double qy = state_(QUATERNION_Y);
    double qz = state_(QUATERNION_Z);
    Matrix quat_to_angular_rate(3,4);
    quat_to_angular_rate(1,1) = -qx;
    quat_to_angular_rate(1,2) = qw;
    quat_to_angular_rate(1,3) = -qz;
    quat_to_angular_rate(1,4) = qy;
    quat_to_angular_rate(2,1) = -qy;
    quat_to_angular_rate(2,2) = -qz;
    quat_to_angular_rate(2,3) = qw;
    quat_to_angular_rate(2,4) = qx;
    quat_to_angular_rate(3,1) = -qz;
    quat_to_angular_rate(3,2) = qy;
    quat_to_angular_rate(3,3) = -qx;
    quat_to_angular_rate(3,4) = qw;
    quat_to_angular_rate *= 2.0;

    getCovariance();

    // position covariance
    for(int i = 0; i < 3; ++i)
      for(int j = 0; j < 3; ++j)
        state.pose.covariance[i*6+j] = covariance_(POSITION_X + i, POSITION_X + j);

    // rotation covariance (world-fixed)
    SymmetricMatrix covariance_rot(quat_to_angular_rate * covariance_.sub(QUATERNION_W,QUATERNION_Z,QUATERNION_W,QUATERNION_Z) * quat_to_angular_rate.transpose());
    for(int i = 0; i < 3; ++i)
      for(int j = 0; j < 3; ++j)
        state.pose.covariance[(i+3)*6+(j+3)] = covariance_rot(i+1,j+1);

    // cross position/rotation covariance
    Matrix covariance_cross(quat_to_angular_rate * covariance_.sub(QUATERNION_W,QUATERNION_Z,POSITION_X,POSITION_Z));
    for(int i = 0; i < 3; ++i)
      for(int j = 0; j < 3; ++j)
        state.pose.covariance[(i+3)*6+j] = state.pose.covariance[j*6+(i+3)] = covariance_cross(i+1,j+1);

    // velocity covariance
    for(int i = 0; i < 3; ++i)
      for(int j = 0; j < 3; ++j)
        state.twist.covariance[i*6+j] = covariance_(VELOCITY_X + i, VELOCITY_X + j);

    // angular rate covariance
    SymmetricMatrix gyro_noise(quat_to_angular_rate * system_->getModel()->AdditiveNoiseSigmaGet().sub(QUATERNION_W,QUATERNION_Z,QUATERNION_W,QUATERNION_Z) * quat_to_angular_rate.transpose());
    for(int i = 0; i < 3; ++i)
      for(int j = 0; j < 3; ++j)
#ifdef USE_RATE_SYSTEM_MODEL
        state.twist.covariance[(i+3)*6+(j+3)] = covariance_(RATE_X + i, RATE_X + j);
#else // USE_RATE_SYSTEM_MODEL
        state.twist.covariance[(i+3)*6+(j+3)] = covariance_(BIAS_GYRO_X + i, BIAS_GYRO_X + j) + gyro_noise(i+1,j+1);
#endif // USE_RATE_SYSTEM_MODEL

    // cross velocity/angular_rate variance
    for(int i = 0; i < 3; ++i)
      for(int j = 0; j < 3; ++j)
#ifdef USE_RATE_SYSTEM_MODEL
        state.twist.covariance[(i+3)*6+(j+3)] = state.twist.covariance[j*6+(i+3)] = covariance_(RATE_X + i, VELOCITY_X + j);
#else // USE_RATE_SYSTEM_MODEL
       state.twist.covariance[(i+3)*6+j] = state.twist.covariance[j*6+(i+3)] = covariance_(BIAS_GYRO_X + i, VELOCITY_X + j);
#endif // USE_RATE_SYSTEM_MODEL
  }
}

void PoseEstimation::getPose(tf::Pose& pose) {
  tf::Quaternion quaternion;
  getPosition(pose.getOrigin());
  getOrientation(quaternion);
  pose.setRotation(quaternion);
}

void PoseEstimation::getPose(tf::Stamped<tf::Pose>& pose) {
  getPose(static_cast<tf::Pose &>(pose));
  pose.stamp_ = timestamp_;
  pose.frame_id_ = nav_frame_;
}

void PoseEstimation::getPose(geometry_msgs::Pose& pose) {
  getPosition(pose.position);
  getOrientation(pose.orientation);
}

void PoseEstimation::getPose(geometry_msgs::PoseStamped& pose) {
  getHeader(pose.header);
  getPose(pose.pose);
}

void PoseEstimation::getPosition(tf::Point& point) {
  getState();
  point = tf::Point(state_(POSITION_X), state_(POSITION_Y), state_(POSITION_Z));
}

void PoseEstimation::getPosition(tf::Stamped<tf::Point>& point) {
  getPosition(static_cast<tf::Point &>(point));
  point.stamp_ = timestamp_;
  point.frame_id_ = nav_frame_;
}

void PoseEstimation::getPosition(geometry_msgs::Point& point) {
  getState();
  point.x = state_(POSITION_X);
  point.y = state_(POSITION_Y);
  point.z = state_(POSITION_Z);
}

void PoseEstimation::getPosition(geometry_msgs::PointStamped& point) {
  getHeader(point.header);
  getPosition(point.point);
}

void PoseEstimation::getGlobalPosition(double &latitude, double &longitude, double &altitude) {
  getState();
  double north =  state_(POSITION_X) * globalReference()->heading().cos - state_(POSITION_Y) * globalReference()->heading().sin;
  double east  = -state_(POSITION_X) * globalReference()->heading().sin - state_(POSITION_Y) * globalReference()->heading().cos;
  latitude  = global_reference_.position().latitude  + north / globalReference()->radius().north;
  longitude = global_reference_.position().longitude + east  / globalReference()->radius().east;
  altitude  = global_reference_.position().altitude  + state_(POSITION_Z);
}

void PoseEstimation::getGlobalPosition(sensor_msgs::NavSatFix& global)
{
  getHeader(global.header);
  global.header.frame_id = world_frame_;

  getGlobalPosition(global.latitude, global.longitude, global.altitude);
  global.latitude  *= 180.0/M_PI;
  global.longitude *= 180.0/M_PI;
}

void PoseEstimation::getOrientation(tf::Quaternion& quaternion) {
  getState();
  quaternion = tf::Quaternion(state_(QUATERNION_X), state_(QUATERNION_Y), state_(QUATERNION_Z), state_(QUATERNION_W));
}

void PoseEstimation::getOrientation(tf::Stamped<tf::Quaternion>& quaternion) {
  getOrientation(static_cast<tf::Quaternion &>(quaternion));
  quaternion.stamp_ = timestamp_;
  quaternion.frame_id_ = nav_frame_;
}

void PoseEstimation::getOrientation(geometry_msgs::Quaternion& quaternion) {
  getState();
  quaternion.w = state_(QUATERNION_W);
  quaternion.x = state_(QUATERNION_X);
  quaternion.y = state_(QUATERNION_Y);
  quaternion.z = state_(QUATERNION_Z);
}

void PoseEstimation::getOrientation(geometry_msgs::QuaternionStamped& quaternion) {
  getHeader(quaternion.header);
  getOrientation(quaternion.quaternion);
}

void PoseEstimation::getOrientation(double &yaw, double &pitch, double &roll) {
  tf::Quaternion quaternion;
  getOrientation(quaternion);
#ifdef TF_MATRIX3x3_H
  tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
#else
  btMatrix3x3(quaternion).getRPY(roll, pitch, yaw);
#endif
}

void PoseEstimation::getImuWithBiases(geometry_msgs::Vector3& linear_acceleration, geometry_msgs::Vector3& angular_velocity) {
  getState();
  const ImuInput *imu = dynamic_cast<const ImuInput *>(&system_->getInput());
  if (imu) {
    linear_acceleration.x = imu->getAccel()[0] + state_(BIAS_ACCEL_X);
    linear_acceleration.y = imu->getAccel()[1] + state_(BIAS_ACCEL_Y);
    linear_acceleration.z = imu->getAccel()[2] + state_(BIAS_ACCEL_Z);
  } else {
    linear_acceleration.x = 0.0;
    linear_acceleration.y = 0.0;
    linear_acceleration.z = 0.0;
  }

#ifdef USE_RATE_SYSTEM_MODEL
  Rate::MeasurementVector rate_body = rate_->getModel()->PredictionGet(0, state_);
  angular_velocity.x    = rate_body(1);
  angular_velocity.y    = rate_body(2);
  angular_velocity.z    = rate_body(3);
#else // USE_RATE_SYSTEM_MODEL
  if (imu) {
    angular_velocity.x    = imu->getRate()[0] + state_(BIAS_GYRO_X);
    angular_velocity.y    = imu->getRate()[1] + state_(BIAS_GYRO_Y);
    angular_velocity.z    = imu->getRate()[2] + state_(BIAS_GYRO_Z);
  } else {
    angular_velocity.x   = 0.0;
    angular_velocity.y   = 0.0;
    angular_velocity.z   = 0.0;
  }
#endif // USE_RATE_SYSTEM_MODEL
}

void PoseEstimation::getVelocity(tf::Vector3& vector) {
  getState();
  vector = tf::Vector3(state_(VELOCITY_X), state_(VELOCITY_Y), state_(VELOCITY_Z));
}

void PoseEstimation::getVelocity(tf::Stamped<tf::Vector3>& vector) {
  getVelocity(static_cast<tf::Vector3 &>(vector));
  vector.stamp_ = timestamp_;
  vector.frame_id_ = nav_frame_;
}

void PoseEstimation::getVelocity(geometry_msgs::Vector3& vector) {
  getState();
  vector.x = state_(VELOCITY_X);
  vector.y = state_(VELOCITY_Y);
  vector.z = state_(VELOCITY_Z);
}

void PoseEstimation::getVelocity(geometry_msgs::Vector3Stamped& vector) {
  getHeader(vector.header);
  getVelocity(vector.vector);
}

void PoseEstimation::getRate(tf::Vector3& vector) {
  getState();
#ifdef USE_RATE_SYSTEM_MODEL
  vector = tf::Vector3(state_(RATE_X), state_(RATE_Y), state_(RATE_Z));
#else // USE_RATE_SYSTEM_MODEL
  const ImuInput *imu = dynamic_cast<const ImuInput *>(&system_->getInput());
  if (imu) {
    vector = tf::Vector3(imu->getRate()[0] + state_(BIAS_GYRO_X), imu->getRate()[1] + state_(BIAS_GYRO_Y), imu->getRate()[2] + state_(BIAS_GYRO_Z));
  } else {
    vector = tf::Vector3(0,0,0);
  }
#endif // USE_RATE_SYSTEM_MODEL
}

void PoseEstimation::getRate(tf::Stamped<tf::Vector3>& vector) {
  getRate(static_cast<tf::Vector3 &>(vector));
  vector.stamp_ = timestamp_;
  vector.frame_id_ = nav_frame_;
}

void PoseEstimation::getRate(geometry_msgs::Vector3& vector) {
  getState();
#ifdef USE_RATE_SYSTEM_MODEL
  vector.x = state_(RATE_X);
  vector.y = state_(RATE_Y);
  vector.z = state_(RATE_Z);
#else // USE_RATE_SYSTEM_MODEL
  const ImuInput *imu = dynamic_cast<const ImuInput *>(&system_->getInput());
  if (imu) {
    vector.x = imu->getRate()[0]  + state_(BIAS_GYRO_X);
    vector.y = imu->getRate()[1]  + state_(BIAS_GYRO_Y);
    vector.z = imu->getRate()[2]  + state_(BIAS_GYRO_Z);
  } else {
    vector.x = 0.0;
    vector.y = 0.0;
    vector.z = 0.0;
  }
#endif // USE_RATE_SYSTEM_MODEL
}

void PoseEstimation::getRate(geometry_msgs::Vector3Stamped& vector) {
  getHeader(vector.header);
  getRate(vector.vector);
}

void PoseEstimation::getBias(tf::Vector3& angular_velocity, tf::Vector3& linear_acceleration) {
  getState();
  angular_velocity.setX(state_(BIAS_GYRO_X));
  angular_velocity.setY(state_(BIAS_GYRO_Y));
  angular_velocity.setZ(state_(BIAS_GYRO_Z));
  linear_acceleration.setX(state_(BIAS_ACCEL_X));
  linear_acceleration.setY(state_(BIAS_ACCEL_Y));
  linear_acceleration.setZ(state_(BIAS_ACCEL_Z));
}

void PoseEstimation::getBias(tf::Stamped<tf::Vector3>& angular_velocity, tf::Stamped<tf::Vector3>& linear_acceleration) {
  getBias(static_cast<tf::Vector3 &>(angular_velocity), static_cast<tf::Vector3 &>(linear_acceleration));
  angular_velocity.stamp_ = timestamp_;
  angular_velocity.frame_id_ = base_frame_;
  linear_acceleration.stamp_ = timestamp_;
  linear_acceleration.frame_id_ = base_frame_;
}

void PoseEstimation::getBias(geometry_msgs::Vector3& angular_velocity, geometry_msgs::Vector3& linear_acceleration) {
  getState();
  angular_velocity.x = state_(BIAS_GYRO_X);
  angular_velocity.y = state_(BIAS_GYRO_Y);
  angular_velocity.z = state_(BIAS_GYRO_Z);
  linear_acceleration.x = state_(BIAS_ACCEL_X);
  linear_acceleration.y = state_(BIAS_ACCEL_Y);
  linear_acceleration.z = state_(BIAS_ACCEL_Z);
}

void PoseEstimation::getBias(geometry_msgs::Vector3Stamped& angular_velocity, geometry_msgs::Vector3Stamped& linear_acceleration) {
  getBias(angular_velocity.vector, linear_acceleration.vector);
  angular_velocity.header.stamp = timestamp_;
  angular_velocity.header.frame_id = base_frame_;
  linear_acceleration.header.stamp = timestamp_;
  linear_acceleration.header.frame_id = base_frame_;
}

void PoseEstimation::getTransforms(std::vector<tf::StampedTransform>& transforms) {
  tf::Quaternion orientation;
  tf::Point position;
  getOrientation(orientation);
  getPosition(position);

  tf::Transform transform(orientation, position);
  double y,p,r;
  transform.getBasis().getEulerYPR(y,p,r);

  std::string parent_frame = nav_frame_;

  if(!position_frame_.empty()) {
    tf::Transform position_transform;
    position_transform.getBasis().setIdentity();
    position_transform.setOrigin(tf::Point(position.x(), position.y(), position.z()));
    transforms.push_back(tf::StampedTransform(position_transform, timestamp_, parent_frame, position_frame_ ));
  }

  if (!footprint_frame_.empty()) {
    tf::Transform footprint_transform;
    footprint_transform.getBasis().setEulerYPR(y, 0.0, 0.0);
    footprint_transform.setOrigin(tf::Point(position.x(), position.y(), 0.0));
    transforms.push_back(tf::StampedTransform(footprint_transform, timestamp_, parent_frame, footprint_frame_));

    parent_frame = footprint_frame_;
    transform = footprint_transform.inverseTimes(transform);
  }

  if (!stabilized_frame_.empty()) {
    tf::Transform stabilized_transform(transform);
#ifdef TF_MATRIX3x3_H
    tf::Matrix3x3 rollpitch_rotation; rollpitch_rotation.setEulerYPR(0.0, p, r);
#else
    btMatrix3x3 rollpitch_rotation; rollpitch_rotation.setEulerYPR(0.0, p, r);
#endif
    stabilized_transform = stabilized_transform * tf::Transform(rollpitch_rotation.inverse());
    transforms.push_back(tf::StampedTransform(stabilized_transform, timestamp_, parent_frame, stabilized_frame_));

    parent_frame = stabilized_frame_;
    transform = stabilized_transform.inverseTimes(transform);
  }

  transforms.push_back(tf::StampedTransform(transform, timestamp_, parent_frame, base_frame_));

//  transforms.resize(3);

//  transforms[0].stamp_ = timestamp_;
//  transforms[0].frame_id_ = nav_frame_;
//  transforms[0].child_frame_id_ = footprint_frame_;
//  transforms[0].setOrigin(tf::Point(position.x(), position.y(), 0.0));
//  rotation.setEulerYPR(y,0.0,0.0);
//  transforms[0].setBasis(rotation);

//  transforms[1].stamp_ = timestamp_;
//  transforms[1].frame_id_ = footprint_frame_;
//  transforms[1].child_frame_id_ = stabilized_frame_;
//  transforms[1].setIdentity();
//  transforms[1].setOrigin(tf::Point(0.0, 0.0, position.z()));

//  transforms[2].stamp_ = timestamp_;
//  transforms[2].frame_id_ = stabilized_frame_;
//  transforms[2].child_frame_id_ = base_frame_;
//  transforms[2].setIdentity();
//  rotation.setEulerYPR(0.0,p,r);
//  transforms[2].setBasis(rotation);
}

void PoseEstimation::updateWorldToOtherTransform(tf::StampedTransform& world_to_other_transform) {
  world_to_other_transform.frame_id_ = world_frame_;

  double y,p,r;
  world_to_other_transform.getBasis().getEulerYPR(y,p,r);
  if (!(getSystemStatus() & STATE_ROLLPITCH))   { r = p = 0.0; }
  if (!(getSystemStatus() & STATE_YAW))         { y = 0.0; }
  if (!(getSystemStatus() & STATE_XY_POSITION)) { world_to_other_transform.getOrigin().setX(0.0); world_to_other_transform.getOrigin().setY(0.0); }
  if (!(getSystemStatus() & STATE_Z_POSITION))  { world_to_other_transform.getOrigin().setZ(0.0); }
  world_to_other_transform.getBasis().setEulerYPR(y, p, r);
}

ParameterList PoseEstimation::getParameters() const {
  ParameterList parameters = parameters_;

  if (system_) {
    parameters.copy(system_->getName(), system_->parameters());
  }

  for(Measurements::const_iterator it = measurements_.begin(); it != measurements_.end(); ++it) {
    parameters.copy((*it)->getName(), (*it)->parameters());
  }

  return parameters;
}

GlobalReference* PoseEstimation::globalReference() {
  return &global_reference_;
}

GlobalReference::GlobalReference() {
  parameters().add("reference_latitude",  position_.latitude);
  parameters().add("reference_longitude", position_.longitude);
  parameters().add("reference_altitude",  position_.altitude);
  parameters().add("reference_heading",   heading_.value);
}

ParameterList& GlobalReference::parameters() {
  return parameters_;
}

void GlobalReference::updated() {
  static const double radius_earth = 6371e3;
  radius_.north = radius_earth + position_.altitude;
  radius_.east  = radius_.north * cos(position_.latitude);
  sincos(heading_.value, &heading_.sin, &heading_.cos);
}

GlobalReference& GlobalReference::setPosition(double latitude, double longitude) {
  position_.latitude = latitude;
  position_.longitude = longitude;
  updated();
  return *this;
}

GlobalReference& GlobalReference::setHeading(double heading) {
  heading_.value = heading;
  updated();
  return *this;
}

GlobalReference& GlobalReference::setAltitude(double altitude) {
  position_.altitude = altitude;
  updated();
  return *this;
}

} // namespace hector_pose_estimation
