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
#include <hector_pose_estimation/filter/ekf.h>
#include <hector_pose_estimation/global_reference.h>

#include <hector_pose_estimation/system/imu_input.h>
#include <hector_pose_estimation/system/imu_model.h>

#include <boost/weak_ptr.hpp>

namespace hector_pose_estimation {

namespace {
  static PoseEstimation *the_instance = 0;
}

PoseEstimation::PoseEstimation(const SystemPtr& system)
  : state_()
//  , rate_(new Rate("rate"))
//  , gravity_(new Gravity ("gravity"))
//  , zerorate_(new ZeroRate("zerorate"))
//  , heading_(new Heading("heading"))
{
  if (!the_instance) the_instance = this;
  if (system) addSystem(system);

  world_frame_ = "world";
  nav_frame_ = "nav";
  base_frame_ = "base_link";
  stabilized_frame_ = "base_stabilized";
  footprint_frame_ = "base_footprint";
  // position_frame_ = "base_position";
  alignment_time_ = 0.0;
  gravity_ = -9.8065;

  parameters().add("world_frame", world_frame_);
  parameters().add("nav_frame", nav_frame_);
  parameters().add("base_frame", base_frame_);
  parameters().add("stabilized_frame", stabilized_frame_);
  parameters().add("footprint_frame", footprint_frame_);
  parameters().add("position_frame", position_frame_);
  parameters().add(globalReference()->parameters());
  parameters().add("alignment_time", alignment_time_);
  parameters().add("gravity", gravity_);

  // add default measurements
//  addMeasurement(rate_);
//  addMeasurement(gravity_);
//  addMeasurement(zerorate_);
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
  if (systems_.empty()) return false;

  // initialize new filter
  filter_.reset(new filter::EKF(state_));

  // initialize measurements (new systems could be added during initialization!)
  for(Measurements::iterator it = measurements_.begin(); it != measurements_.end(); ++it)
    if (!(*it)->init(*this, *filter_, state_)) return false;

  // initialize systems (new systems could be added during initialization!)
  for(Systems::iterator it = systems_.begin(); it != systems_.end(); ++it)
    if (!(*it)->init(*this, *filter_, state_)) return false;

  // reset (or initialize) filter and measurements
  reset();

  return true;
}

void PoseEstimation::cleanup()
{
  // cleanup system
  for(Systems::iterator it = systems_.begin(); it != systems_.end(); ++it) (*it)->cleanup();

  // cleanup measurements
  for(Measurements::iterator it = measurements_.begin(); it != measurements_.end(); ++it) (*it)->cleanup();

  // delete filter instance
  if (filter_) filter_.reset();
}

void PoseEstimation::reset()
{
  // check if system is initialized
  if (systems_.empty()) return;

  // set initial status
  if (filter_) filter_->reset();

  // restart alignment
  alignment_start_ = ros::Time();
  if (alignment_time_ > 0) {
    state().setSystemStatus(STATE_ALIGNMENT);
  }

  // reset systems and measurements
  for(Systems::iterator it = systems_.begin(); it != systems_.end(); ++it) {
    (*it)->reset(state_);
    (*it)->getPrior(state());
  }

  for(Measurements::iterator it = measurements_.begin(); it != measurements_.end(); ++it) {
    (*it)->reset(state_);
  }

  updated();
}

void PoseEstimation::update(ros::Time new_timestamp)
{
  // check if system is initialized
  if (systems_.empty()) return;

  ros::Duration dt;
  if (!getTimestamp().isZero()) dt = new_timestamp - getTimestamp();
  setTimestamp(new_timestamp);

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
  if (systems_.empty() || !filter_) return;

  // filter rate measurement first
  boost::shared_ptr<ImuInput> imu = boost::shared_dynamic_cast<ImuInput>(getInput("imu"));
#ifdef USE_RATE_SYSTEM_MODEL
//  if (imu && rate_) {
//    ROS_DEBUG("Updating with measurement model %s", rate_->getName().c_str());
//    rate_->update(*this, Rate::Update(imu->getRate()));
//  }
#endif // USE_RATE_SYSTEM_MODEL

  // time update step
  filter_->predict(state_, systems_, dt);

  // measurement updates
  filter_->update(state_, measurements_);

//  // iterate through measurements and do the measurement update steps
//  SystemStatus measurement_status = 0;

//  for(Measurements::iterator it = measurements_.begin(); it != measurements_.end(); ++it) {
//    MeasurementPtr measurement = *it;
//    if (!measurement->active(getSystemStatus())) continue;

////    // special updates
////    if (measurement == rate_) continue;

////    if (imu && measurement == gravity_) {
////      ROS_DEBUG("Updating with pseudo measurement model %s", gravity_->getName().c_str());
////      gravity_->update(*this, Gravity::Update(imu->getAccel()));
////      continue;
////    }

////    if (imu && measurement == zerorate_) {
////      ROS_DEBUG("Updating with pseudo measurement model %s", zerorate_->getName().c_str());
////      zerorate_->update(*this, ZeroRate::Update(imu->getRate()[2]));
////      continue;
////    }

////    if (measurement == &heading_) {
////      ROS_DEBUG("Updating with pseudo measurement model %s", heading_.getName().c_str());
////      Heading::Update y(0.0);
////      heading_.update(*this, y);
////      continue;
////    }

//    // skip all other measurements during alignment
//    if (inSystemStatus(STATE_ALIGNMENT)) continue;

//    // process the incoming queue
//    measurement->process(filter_, state_);
//    measurement_status |= measurement->getStatusFlags();
//    measurement->increase_timer(dt);
//  }

//  // update the measurement status
//  setMeasurementStatus(measurement_status);

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
    if (alignment_start_.isZero()) alignment_start_ = getTimestamp();
    if ((getTimestamp() - alignment_start_).toSec() >= alignment_time_) {
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
  for(Systems::iterator it = systems_.begin(); it != systems_.end(); ++it) {
    (*it)->limitState(state());
  }
}

const SystemPtr& PoseEstimation::addSystem(const SystemPtr& system, const std::string& name) {
  if (!name.empty() && system->getName().empty()) system->setName(name);
  return systems_.add(system, system->getName());
}

InputPtr PoseEstimation::addInput(const InputPtr& input, const std::string& name)
{
  if (!name.empty()) input->setName(name);
  return inputs_.add(input, input->getName());
}

InputPtr PoseEstimation::setInput(const Input& value, const std::string& name)
{
  InputPtr input = inputs_.get(!name.empty() ? name : input->getName());
  if (!input) return input;

  *input = value;
  return input;
}

const MeasurementPtr& PoseEstimation::addMeasurement(const MeasurementPtr& measurement, const std::string& name) {
  if (!name.empty()) measurement->setName(name);
  return measurements_.add(measurement, measurement->getName());
}

const State::Vector& PoseEstimation::getStateVector() {
//  if (state_is_dirty_) {
//    state_ = filter_->PostGet()->ExpectedValueGet();
//    state_is_dirty_ = false;
//  }
  return state_.getVector();
}

const State::Covariance& PoseEstimation::getCovariance() {
//  if (covariance_is_dirty_) {
//    covariance_ = filter_->PostGet()->CovarianceGet();
//    covariance_is_dirty_ = false;
//  }
  return state_.getCovariance();
}

SystemStatus PoseEstimation::getSystemStatus() const {
  return state_.getSystemStatus();
}

SystemStatus PoseEstimation::getMeasurementStatus() const {
  return state_.getMeasurementStatus();
}

bool PoseEstimation::inSystemStatus(SystemStatus test_status) const {
  return state_.inSystemStatus(test_status);
}

bool PoseEstimation::setSystemStatus(SystemStatus new_status) {
  return state_.setSystemStatus(new_status);
}

bool PoseEstimation::setMeasurementStatus(SystemStatus new_measurement_status) {
  return state_.setMeasurementStatus(new_measurement_status);
}

bool PoseEstimation::updateSystemStatus(SystemStatus set, SystemStatus clear) {
  return state_.updateSystemStatus(set, clear);
}

bool PoseEstimation::updateMeasurementStatus(SystemStatus set, SystemStatus clear) {
  return state_.updateMeasurementStatus(set, clear);
}

const ros::Time& PoseEstimation::getTimestamp() const {
  return state_.getTimestamp();
}

void PoseEstimation::setTimestamp(const ros::Time& timestamp) {
  state_.setTimestamp(timestamp);
}

void PoseEstimation::getHeader(std_msgs::Header& header) {
  header.stamp = getTimestamp();
  header.frame_id = nav_frame_;
}

void PoseEstimation::getState(nav_msgs::Odometry& state, bool with_covariances) {
  getHeader(state.header);
  getPose(state.pose.pose);
  getVelocity(state.twist.twist.linear);
  getRate(state.twist.twist.angular);
  state.child_frame_id = base_frame_;

  if (with_covariances) {
    Quaternion q(state_.getOrientation());
    Matrix_<3,4> quat_to_angular_rate;
    quat_to_angular_rate(1,1) = -q.x();
    quat_to_angular_rate(1,2) = q.w();
    quat_to_angular_rate(1,3) = -q.z();
    quat_to_angular_rate(1,4) = q.y();
    quat_to_angular_rate(2,1) = -q.y();
    quat_to_angular_rate(2,2) = -q.z();
    quat_to_angular_rate(2,3) = q.w();
    quat_to_angular_rate(2,4) = q.x();
    quat_to_angular_rate(3,1) = -q.z();
    quat_to_angular_rate(3,2) = q.y();
    quat_to_angular_rate(3,3) = -q.x();
    quat_to_angular_rate(3,4) = q.w();
    quat_to_angular_rate *= 2.0;

    Matrix_<State::Dimension,State::Dimension> covariance(state_.getCovariance());

    // position covariance
    if (state_.getOrientationIndex() >= 0) {
      for(int i = 0; i < 3; ++i)
        for(int j = 0; j < 3; ++j)
          state.pose.covariance[i*6+j] = covariance(state_.getPositionIndex() + i, state_.getPositionIndex() + j);
    }

    // rotation covariance (world-fixed)
    if (state_.getOrientationIndex() >= 0) {
      SymmetricMatrix_<3> covariance_rot(quat_to_angular_rate * covariance.block<4,4>(state_.getOrientationIndex(), state_.getOrientationIndex()) * quat_to_angular_rate.transpose());
      for(int i = 0; i < 3; ++i)
        for(int j = 0; j < 3; ++j)
          state.pose.covariance[(i+3)*6+(j+3)] = covariance_rot(i+1,j+1);
    }

    // cross position/rotation covariance
    if (state_.getOrientationIndex() >= 0 && state_.getPositionIndex() >= 0) {
      Matrix_<3,3> covariance_cross(quat_to_angular_rate * covariance.block<4,3>(state_.getOrientationIndex(), state_.getPositionIndex()));
      for(int i = 0; i < 3; ++i)
        for(int j = 0; j < 3; ++j)
          state.pose.covariance[(i+3)*6+j] = state.pose.covariance[j*6+(i+3)] = covariance_cross(i+1,j+1);
    }

    // velocity covariance
    if (state_.getVelocityIndex() >= 0) {
      for(int i = 0; i < 3; ++i)
        for(int j = 0; j < 3; ++j)
          state.twist.covariance[i*6+j] = covariance(state_.getVelocityIndex() + i, state_.getVelocityIndex() + j);
    }

    // angular rate covariance
    if (state_.getRateIndex() >= 0) {
      for(int i = 0; i < 3; ++i)
        for(int j = 0; j < 3; ++j)
          state.twist.covariance[(i+3)*6+(j+3)] = covariance(state_.getRateIndex() + i, state_.getRateIndex() + j);
    }

    // cross velocity/angular_rate variance
    if (state_.getRateIndex() >= 0 && state_.getVelocityIndex() >= 0) {
      for(int i = 0; i < 3; ++i)
        for(int j = 0; j < 3; ++j)
          state.twist.covariance[(i+3)*6+(j+3)] = state.twist.covariance[j*6+(i+3)] = covariance(state_.getRateIndex() + i, state_.getVelocityIndex() + j);
    }
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
  pose.stamp_ = getTimestamp();
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
  const typename State::PositionType& position = state().getPosition();
  point = tf::Point(position.x(), position.y(), position.z());
}

void PoseEstimation::getPosition(tf::Stamped<tf::Point>& point) {
  getPosition(static_cast<tf::Point &>(point));
  point.stamp_ = getTimestamp();
  point.frame_id_ = nav_frame_;
}

void PoseEstimation::getPosition(geometry_msgs::Point& point) {
  const typename State::PositionType& position = state().getPosition();
  point.x = position.x();
  point.y = position.y();
  point.z = position.z();
}

void PoseEstimation::getPosition(geometry_msgs::PointStamped& point) {
  getHeader(point.header);
  getPosition(point.point);
}

void PoseEstimation::getGlobalPosition(double &latitude, double &longitude, double &altitude) {
  const typename State::PositionType& position = state().getPosition();
  double north =  position.x() * globalReference()->heading().cos - position.y() * globalReference()->heading().sin;
  double east  = -position.x() * globalReference()->heading().sin - position.y() * globalReference()->heading().cos;
  latitude  = globalReference()->position().latitude  + north / globalReference()->radius().north;
  longitude = globalReference()->position().longitude + east  / globalReference()->radius().east;
  altitude  = globalReference()->position().altitude  + position.z();
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
  Quaternion orientation(state().getOrientation());
  quaternion = tf::Quaternion(orientation.x(), orientation.y(), orientation.z(), orientation.w());
}

void PoseEstimation::getOrientation(tf::Stamped<tf::Quaternion>& quaternion) {
  getOrientation(static_cast<tf::Quaternion &>(quaternion));
  quaternion.stamp_ = getTimestamp();
  quaternion.frame_id_ = nav_frame_;
}

void PoseEstimation::getOrientation(geometry_msgs::Quaternion& quaternion) {
  Quaternion orientation(state().getOrientation());
  quaternion.w = orientation.w();
  quaternion.x = orientation.x();
  quaternion.y = orientation.y();
  quaternion.z = orientation.z();
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
  boost::shared_ptr<const ImuInput>  input     = boost::shared_dynamic_cast<const ImuInput>(getInput("imu"));
  boost::shared_ptr<const Accelerometer> accel = boost::shared_dynamic_cast<const Accelerometer>(getSystem("accelerometer"));

  if (input) {
    linear_acceleration.x = input->getAcceleration().x();
    linear_acceleration.y = input->getAcceleration().y();
    linear_acceleration.z = input->getAcceleration().z();
  } else {
    linear_acceleration.x = 0.0;
    linear_acceleration.y = 0.0;
    linear_acceleration.z = 0.0;
  }

  if (accel) {
    linear_acceleration.x += accel->getModel()->getBias().x();
    linear_acceleration.y += accel->getModel()->getBias().y();
    linear_acceleration.z += accel->getModel()->getBias().z();
  }

  getRate(angular_velocity);
}

void PoseEstimation::getVelocity(tf::Vector3& vector) {
  const typename State::VelocityType& velocity = state().getVelocity();
  vector = tf::Vector3(velocity.x(), velocity.y(), velocity.z());
}

void PoseEstimation::getVelocity(tf::Stamped<tf::Vector3>& vector) {
  getVelocity(static_cast<tf::Vector3 &>(vector));
  vector.stamp_ = getTimestamp();
  vector.frame_id_ = nav_frame_;
}

void PoseEstimation::getVelocity(geometry_msgs::Vector3& vector) {
  const typename State::VelocityType& velocity = state().getVelocity();
  vector.x = velocity.x();
  vector.y = velocity.y();
  vector.z = velocity.z();
}

void PoseEstimation::getVelocity(geometry_msgs::Vector3Stamped& vector) {
  getHeader(vector.header);
  getVelocity(vector.vector);
}

void PoseEstimation::getRate(tf::Vector3& vector) {
  geometry_msgs::Vector3 rate;
  getRate(rate);
  vector = tf::Vector3(rate.x, rate.y, rate.z);
}

void PoseEstimation::getRate(tf::Stamped<tf::Vector3>& vector) {
  getRate(static_cast<tf::Vector3 &>(vector));
  vector.stamp_ = getTimestamp();
  vector.frame_id_ = nav_frame_;
}

void PoseEstimation::getRate(geometry_msgs::Vector3& vector) {
  if (state().getRateIndex() >= 0) {
    vector.x    = state().getRate().x();
    vector.y    = state().getRate().y();
    vector.z    = state().getRate().z();

  } else {
    boost::shared_ptr<const ImuInput> input = boost::shared_dynamic_cast<const ImuInput>(getInput("imu"));
    boost::shared_ptr<const Gyro> gyro      = boost::shared_dynamic_cast<const Gyro>(getSystem("gyro"));

    if (input) {
      vector.x = input->getRate().x();
      vector.y = input->getRate().y();
      vector.z = input->getRate().z();
    } else {
      vector.x = 0.0;
      vector.y = 0.0;
      vector.z = 0.0;
    }

    if (gyro) {
      vector.x += gyro->getModel()->getBias().x();
      vector.y += gyro->getModel()->getBias().y();
      vector.z += gyro->getModel()->getBias().z();
    }
  }
}

void PoseEstimation::getRate(geometry_msgs::Vector3Stamped& vector) {
  getHeader(vector.header);
  getRate(vector.vector);
}

void PoseEstimation::getBias(geometry_msgs::Vector3& angular_velocity, geometry_msgs::Vector3& linear_acceleration) {
  boost::shared_ptr<const ImuInput>  input     = boost::shared_dynamic_cast<const ImuInput>(getInput("imu"));
  boost::shared_ptr<const Accelerometer> accel = boost::shared_dynamic_cast<const Accelerometer>(getSystem("accelerometer"));
  boost::shared_ptr<const Gyro> gyro           = boost::shared_dynamic_cast<const Gyro>(getSystem("gyro"));

  if (gyro) {
    angular_velocity.x = gyro->getModel()->getBias().x();
    angular_velocity.y = gyro->getModel()->getBias().y();
    angular_velocity.z = gyro->getModel()->getBias().z();
  } else {
    angular_velocity.x = 0.0;
    angular_velocity.y = 0.0;
    angular_velocity.z = 0.0;
  }

  if (accel) {
    linear_acceleration.x = accel->getModel()->getBias().x();
    linear_acceleration.y = accel->getModel()->getBias().y();
    linear_acceleration.z = accel->getModel()->getBias().z();
  } else {
    linear_acceleration.x = 0.0;
    linear_acceleration.y = 0.0;
    linear_acceleration.z = 0.0;
  }
}

void PoseEstimation::getBias(geometry_msgs::Vector3Stamped& angular_velocity, geometry_msgs::Vector3Stamped& linear_acceleration) {
  getBias(angular_velocity.vector, linear_acceleration.vector);
  angular_velocity.header.stamp = getTimestamp();
  angular_velocity.header.frame_id = base_frame_;
  linear_acceleration.header.stamp = getTimestamp();
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
    transforms.push_back(tf::StampedTransform(position_transform, getTimestamp(), parent_frame, position_frame_ ));
  }

  if (!footprint_frame_.empty()) {
    tf::Transform footprint_transform;
    footprint_transform.getBasis().setEulerYPR(y, 0.0, 0.0);
    footprint_transform.setOrigin(tf::Point(position.x(), position.y(), 0.0));
    transforms.push_back(tf::StampedTransform(footprint_transform, getTimestamp(), parent_frame, footprint_frame_));

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
    transforms.push_back(tf::StampedTransform(stabilized_transform, getTimestamp(), parent_frame, stabilized_frame_));

    parent_frame = stabilized_frame_;
    transform = stabilized_transform.inverseTimes(transform);
  }

  transforms.push_back(tf::StampedTransform(transform, getTimestamp(), parent_frame, base_frame_));

//  transforms.resize(3);

//  transforms[0].stamp_ = getTimestamp();
//  transforms[0].frame_id_ = nav_frame_;
//  transforms[0].child_frame_id_ = footprint_frame_;
//  transforms[0].setOrigin(tf::Point(position.x(), position.y(), 0.0));
//  rotation.setEulerYPR(y,0.0,0.0);
//  transforms[0].setBasis(rotation);

//  transforms[1].stamp_ = getTimestamp();
//  transforms[1].frame_id_ = footprint_frame_;
//  transforms[1].child_frame_id_ = stabilized_frame_;
//  transforms[1].setIdentity();
//  transforms[1].setOrigin(tf::Point(0.0, 0.0, position.z()));

//  transforms[2].stamp_ = getTimestamp();
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

  for(Systems::iterator it = systems_.begin(); it != systems_.end(); ++it) {
    parameters.copy((*it)->getName(), (*it)->parameters());
  }

  for(Measurements::iterator it = measurements_.begin(); it != measurements_.end(); ++it) {
    parameters.copy((*it)->getName(), (*it)->parameters());
  }

  return parameters;
}

const GlobalReferencePtr &PoseEstimation::globalReference() {
  return GlobalReference::Instance();
}

} // namespace hector_pose_estimation
