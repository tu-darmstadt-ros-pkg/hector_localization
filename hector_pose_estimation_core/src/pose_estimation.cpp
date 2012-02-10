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
#include <hector_pose_estimation/measurements/gravity.h>

namespace hector_pose_estimation {

namespace {
  static PoseEstimation *the_instance = 0;
}

PoseEstimation::PoseEstimation(SystemModel *system_model)
  : system_(0), filter_(0)
  , state_is_dirty_(true)
  , covariance_is_dirty_(true)
  , status_()
  , measurement_status_()
  , gravity_("gravity")
  , zerorate_("zerorate")
{
  if (!the_instance) the_instance = this;

  base_frame_ = "base_link";
  nav_frame_ = "nav";
  alignment_time_ = 0.0;

  parameters().add("base_frame", base_frame_);
  parameters().add("nav_frame", nav_frame_);
  parameters().add("reference_latitude",  global_reference_.latitude);
  parameters().add("reference_longitude", global_reference_.longitude);
  parameters().add("reference_altitude",  global_reference_.altitude);
  parameters().add("reference_heading",   global_reference_.heading);
  parameters().add("alignment_time",      alignment_time_);

  // initialize system model
  setSystemModel(system_model);

  // add default measurements
  addMeasurement(&gravity_);
  addMeasurement(&zerorate_);
}

PoseEstimation::~PoseEstimation()
{
  cleanup();
}

PoseEstimation *PoseEstimation::Instance() {
  if (!the_instance) the_instance = new PoseEstimation();
  return the_instance;
}

void PoseEstimation::setSystemModel(SystemModel *new_system_model) {
  if (system_) {
    cleanup();
    delete system_;
    system_ = 0;
  }

  if (!new_system_model) return;
  system_ = new System(new_system_model);
}

const SystemModel *PoseEstimation::getSystemModel() const {
  return system_->getModel();
}

System *PoseEstimation::getSystem() const {
  return system_;
}

bool PoseEstimation::init()
{
  // cleanup everything
  if (!system_) return false;

  // reset (or initialize) filter and measurements
  reset();

  // initialize all measurements
  for(Measurements::iterator it = measurements_.begin(); it != measurements_.end(); ++it) (*it)->init();

  return true;
}

void PoseEstimation::cleanup()
{
  // delete filter instance
  if (filter_) {
    delete filter_;
    filter_ = 0;
  }

  // cleanup all measurements
  for(Measurements::iterator it = measurements_.begin(); it != measurements_.end(); ++it) (*it)->cleanup();
}

void PoseEstimation::reset()
{
  // reset extended Kalman filter
  if (filter_) cleanup();
  filter_ = new BFL::ExtendedKalmanFilter(system_->getPrior());
  updated();

  // set initial status
  if (alignment_time_ > 0) {
    status_ = STATE_ALIGNMENT;
    alignment_start_ = ros::Time();
  } else {
    status_ = static_cast<SystemStatus>(0);
  }

  // reset all measurements
  for(Measurements::iterator it = measurements_.begin(); it != measurements_.end(); ++it) (*it)->reset(getState());
}

void PoseEstimation::update(const InputVector& input, ros::Time new_timestamp)
{
  ros::Duration dt;

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

  // check if filter is initialized
  if (!system_ || !filter_) return;

  // time update step
  system_->update(*this, dt);

//  std::cout << "     u = [" << system_->getInput().transpose() << "]" << std::endl;
//  std::cout << "x_pred = [" << getState().transpose() << "]" << std::endl;
//  std::cout << "P_pred = [" << filter_->PostGet()->CovarianceGet() << "]" << std::endl;

  // iterate through measurements and do the measurement update steps
  SystemStatus measurement_status = 0;
  for(Measurements::iterator it = measurements_.begin(); it != measurements_.end(); ++it) {
    Measurement *measurement = *it;
    if (!measurement->active(getSystemStatus())) continue;

    // process the incoming queue
    measurement->process(*this);
    measurement_status |= measurement->getStatusFlags();
    measurement->increase_timer(dt);
  }

  // pseudo updates
  if (!(measurement_status & (STATE_XY_VELOCITY | STATE_XY_POSITION))) {
    ROS_DEBUG("Updating with pseudo measurement model %s", gravity_.getName().c_str());
    Gravity::Update y(system_->getInput().sub(ACCEL_X, ACCEL_Z));
    gravity_.enable();
    if (gravity_.update(*this, y)) measurement_status |= gravity_.getStatusFlags();
  } else {
    gravity_.disable();
  }

  if (!(measurement_status & STATE_YAW)) {
    ROS_DEBUG("Updating with pseudo measurement model %s", zerorate_.getName().c_str());
    ZeroRate::Update y(system_->getInput().sub(GYRO_Z, GYRO_Z));
    zerorate_.enable();
    if (zerorate_.update(*this, y)) measurement_status |= zerorate_.getStatusFlags();
  } else {
    zerorate_.disable();
  }

  // update the measurement status
  updateMeasurementStatus(measurement_status, STATE_ROLLPITCH | STATE_YAW | STATE_XY_POSITION | STATE_XY_VELOCITY | STATE_Z_POSITION | STATE_Z_VELOCITY);

//  std::cout << "x_est = [" << getState().transpose() << "]" << std::endl;
//  std::cout << "P_est = [" << filter_->PostGet()->CovarianceGet() << "]" << std::endl;

  // switch overall system state
  if (status_ & STATE_ALIGNMENT) {
    if (alignment_start_.isZero()) alignment_start_ = timestamp_;
    if ((timestamp_ - alignment_start_).toSec() >= alignment_time_) {
      updateSystemStatus(STATE_DEGRADED, STATE_ALIGNMENT);
    }
  } else if ((status_ & STATE_ROLLPITCH) && (status_ & STATE_YAW) && (status_ & STATE_XY_POSITION) && (status_ & STATE_Z_POSITION)) {
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

Measurement *PoseEstimation::addMeasurement(Measurement *measurement) {
  measurements_.push_back(measurement);
  return measurement;
}

Measurement *PoseEstimation::addMeasurement(const std::string& name, Measurement *measurement) {
	measurement->setName(name);
	return addMeasurement(measurement);
}

Measurement *PoseEstimation::getMeasurement(const std::string &name) const {
  for(Measurements::const_iterator it = measurements_.begin(); it != measurements_.end(); ++it) {
    if ((*it)->getName() == name) return *it;
  }
  return 0;
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
  state_is_dirty_ = true;
}

void PoseEstimation::setCovariance(const StateCovariance& covariance) {
  filter_->PostGet()->CovarianceSet(covariance);
  covariance_is_dirty_ = true;
}


SystemStatus PoseEstimation::getSystemStatus() const {
  return status_ | measurement_status_ | system_->getStatusFlags();
}

SystemStatus PoseEstimation::getMeasurementStatus() const {
  return measurement_status_;
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
  setSystemStatus(status_ | new_measurement_status);

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

ros::Time PoseEstimation::getTimestamp() const {
  return timestamp_;
}

void PoseEstimation::setTimestamp(ros::Time timestamp) {
  timestamp_ = timestamp;
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

void PoseEstimation::getPosition(tf::Point& point) {
  getState();
  point = tf::Point(state_(POSITION_X), state_(POSITION_Y), state_(POSITION_Z));
}

void PoseEstimation::getPosition(tf::Stamped<tf::Point>& point) {
  getPosition(static_cast<tf::Point &>(point));
  point.stamp_ = timestamp_;
  point.frame_id_ = nav_frame_;
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

void PoseEstimation::getImuWithBiases(geometry_msgs::Vector3& linear_acceleration, geometry_msgs::Vector3& angular_velocity) {
  getState();
  const InputVector &input = system_->getInput();
  linear_acceleration.x = input(ACCEL_X) + state_(BIAS_ACCEL_X);
  linear_acceleration.y = input(ACCEL_Y) + state_(BIAS_ACCEL_Y);
  linear_acceleration.z = input(ACCEL_Z) + state_(BIAS_ACCEL_Z);
  angular_velocity.x    = input(GYRO_X)  + state_(BIAS_GYRO_X);
  angular_velocity.y    = input(GYRO_Y)  + state_(BIAS_GYRO_Y);
  angular_velocity.z    = input(GYRO_Z)  + state_(BIAS_GYRO_Z);
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

void PoseEstimation::getTransforms(std::vector<tf::StampedTransform>& transforms) {
  transforms.resize(3);
  tf::Quaternion orientation;
  tf::Point position;
  getOrientation(orientation);
  getPosition(position);
  btMatrix3x3 rotation(orientation);
  double y,p,r;
  rotation.getEulerYPR(y,p,r);

  transforms[0].stamp_ = timestamp_;
  transforms[0].frame_id_ = nav_frame_;
  transforms[0].child_frame_id_ = "base_footprint";
  transforms[0].setOrigin(tf::Point(position.x(), position.y(), 0.0));
  rotation.setEulerYPR(y,0.0,0.0);
  transforms[0].setBasis(rotation);

  transforms[1].stamp_ = timestamp_;
  transforms[1].frame_id_ = "base_footprint";
  transforms[1].child_frame_id_ = "base_stabilized";
  transforms[1].setIdentity();
  transforms[1].setOrigin(tf::Point(0.0, 0.0, position.z()));

  transforms[2].stamp_ = timestamp_;
  transforms[2].frame_id_ = "base_stabilized";
  transforms[2].child_frame_id_ = base_frame_;
  transforms[2].setIdentity();
  rotation.setEulerYPR(0.0,p,r);
  transforms[2].setBasis(rotation);
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

} // namespace hector_pose_estimation
