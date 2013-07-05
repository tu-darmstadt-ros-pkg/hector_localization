//=================================================================================================
// Copyright (c) 2013, Johannes Meyer, TU Darmstadt
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

#include <hector_pose_estimation/state.h>
#include <hector_pose_estimation/substate.h>

namespace hector_pose_estimation {

State::State()
  : vector_(Dimension)
  , covariance_(Dimension)
  , base_(new SubState_<0>(*this))
{
  reset();
}

State::State(const Vector &vector, const Covariance& covariance)
  : vector_(vector)
  , covariance_(covariance)
  , base_(new SubState_<0>(*this))
{
  reset();
}

State::~State()
{
}

void State::reset()
{
  // reset state
  vector_.setZero();
  covariance_.setZero();
  orientation().w() = 1.0;

  // reset status flags
  system_status_ = 0;
  measurement_status_ = 0;

  // reset pseudo-states
  fake_rate_.resize(3,1);
  fake_rate_ << 0.0, 0.0, 0.0;
  fake_orientation_.resize(4,1);
  fake_orientation_ << 0.0, 0.0, 0.0, 1.0;
  fake_position_.resize(3,1);
  fake_position_ << 0.0, 0.0, 0.0;
  fake_velocity_.resize(3,1);
  fake_velocity_ << 0.0, 0.0, 0.0;
  fake_acceleration_.resize(3,1);
  fake_acceleration_ << 0.0, 0.0, 0.0;
}

bool State::valid() const {
  return (vector_ == vector_);
}

void State::updated()
{
  normalize();
  P().symmetric();
}

State::RotationMatrix State::getRotationMatrix() const {
  RotationMatrix R;
  getRotationMatrix(R);
  return R;
}

void State::getRotationMatrix(RotationMatrix &R) const
{
  ConstOrientationType q(getOrientation());
  R << (q.w()*q.w()+q.x()*q.x()-q.y()*q.y()-q.z()*q.z()), (2.0*q.x()*q.y()-2.0*q.w()*q.z()),                 (2.0*q.x()*q.z()+2.0*q.w()*q.y()),
       (2.0*q.x()*q.y()+2.0*q.w()*q.z()),                 (q.w()*q.w()-q.x()*q.x()+q.y()*q.y()-q.z()*q.z()), (2.0*q.y()*q.z()-2.0*q.w()*q.x()),
       (2.0*q.x()*q.z()-2.0*q.w()*q.y()),                 (2.0*q.y()*q.z()+2.0*q.w()*q.x()),                 (q.w()*q.w()-q.x()*q.x()-q.y()*q.y()+q.z()*q.z());
}

double State::getYaw() const
{
  ConstOrientationType q(getOrientation());
  return atan2(2*q.x()*q.y() + 2*q.w()*q.z(), q.x()*q.x() + q.w()*q.w() - q.z()*q.z() - q.y()*q.y());
}

bool State::inSystemStatus(SystemStatus test_status) const {
  return (getSystemStatus() & test_status) == test_status;
}

bool State::setSystemStatus(SystemStatus new_status) {
  if (new_status == system_status_) return true;

  // iterate through StatusCallbacks
  for(std::vector<SystemStatusCallback>::const_iterator it = status_callbacks_.begin(); it != status_callbacks_.end(); ++it)
    if (!(*it)(new_status)) return false;

  SystemStatus set = new_status & ~system_status_;
  SystemStatus cleared = system_status_ & ~new_status;
  if (set)     ROS_INFO_STREAM("Set system status " << getSystemStatusString(new_status, set));
  if (cleared) ROS_INFO_STREAM("Cleared system status " << getSystemStatusString(cleared, cleared));

  system_status_ = new_status;
  return true;
}

bool State::setMeasurementStatus(SystemStatus new_measurement_status) {
  SystemStatus set = new_measurement_status & ~measurement_status_;
  SystemStatus cleared = measurement_status_ & ~new_measurement_status;
  if (set)     ROS_INFO_STREAM("Set measurement status " << getSystemStatusString(new_measurement_status, set));
  if (cleared) ROS_INFO_STREAM("Cleared measurement status " << getSystemStatusString(cleared, cleared));

  measurement_status_ = new_measurement_status;
  return true;
}

bool State::updateSystemStatus(SystemStatus set, SystemStatus clear) {
  return setSystemStatus((system_status_ & ~clear) | set);
}

bool State::updateMeasurementStatus(SystemStatus set, SystemStatus clear) {
  return setMeasurementStatus((measurement_status_ & ~clear) | set);
}

void State::addSystemStatusCallback(const SystemStatusCallback& callback) {
//  for(std::vector<SystemStatusCallback>::const_iterator it = status_callbacks_.begin(); it != status_callbacks_.end(); ++it)
//    if (*it == callback) return;
  status_callbacks_.push_back(callback);
}

double State::normalize() {
  double s = 1.0 / orientation().norm();
  orientation() = orientation() * s;
  return s;
}

template class SubState_<0>;

} // namespace hector_pose_estimation
