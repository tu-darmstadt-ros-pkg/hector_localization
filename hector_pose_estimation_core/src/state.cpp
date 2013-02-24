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
#include <hector_pose_estimation/system_model.h>

namespace hector_pose_estimation {

State::State()
  : state_(Dimension)
  , covariance_(Dimension)
  , orientation_(state_.segment<4>(QUATERNION_X))
#ifdef USE_RATE_SYSTEM_MODEL
  , rate_(state_.segment<3>(RATE_X))
#else
  , rate_storage_(new ColumnVector(3))
  , rate_(rate_storage_->segment<3>(0))
#endif
  , position_(state_.segment<3>(POSITION_X))
  , velocity_(state_.segment<3>(VELOCITY_X))
  , acceleration_storage_(new ColumnVector(3))
  , acceleration_(acceleration_storage_->segment<3>(0))
{
  reset();
}

State::State(const Vector &vector, const Covariance& covariance)
  : state_(vector)
  , covariance_(covariance)
  , orientation_(state_.segment<4>(QUATERNION_X))
#ifdef USE_RATE_SYSTEM_MODEL
  , rate_(state_.segment<3>(RATE_X))
#else
  , rate_storage_(new ColumnVector(3))
  , rate_(rate_storage_->segment<3>(0))
#endif
  , position_(state_.segment<3>(POSITION_X))
  , velocity_(state_.segment<3>(VELOCITY_X))
  , acceleration_storage_(new ColumnVector(3))
  , acceleration_(acceleration_storage_->segment<3>(0))
{
  reset();
}

State::~State()
{}

void State::reset()
{
  // reset state
  state_.setZero();
  covariance_.setZero();
  orientation().w() = 1.0;

  // reset status flags
  system_status_ = 0;
  measurement_status_ = 0;

  // reset pseudo-states
  if (rate_storage_)         rate_storage_->setZero();
  if (acceleration_storage_) acceleration_storage_->setZero();

  // reset all substates
  for(SubStates::iterator sub = substates_.begin(); sub != substates_.end(); ++sub) {
    (*sub)->reset();
  }
}

bool State::valid() const {
  return (state_ == state_);
}

void State::updated()
{
  normalize();
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
  if (set)     ROS_INFO_STREAM("Set system state " << getSystemStatusString(set));
  if (cleared) ROS_INFO_STREAM("Cleared system state " << getSystemStatusString(cleared));

  system_status_ = new_status;
  return true;
}

bool State::setMeasurementStatus(SystemStatus new_measurement_status) {
  SystemStatus set = new_measurement_status & ~measurement_status_;
  SystemStatus cleared = measurement_status_ & ~new_measurement_status;
  if (set)     ROS_INFO_STREAM("Set measurement state " << getSystemStatusString(set));
  if (cleared) ROS_INFO_STREAM("Cleared measurement state " << getSystemStatusString(cleared));

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
  double s = orientation().norm();
  orientation() = orientation() / s;
  return s;
}

} // namespace hector_pose_estimation
