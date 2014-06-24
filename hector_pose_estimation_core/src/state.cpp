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

enum StateIndex {
  QUATERNION_X = 0,
  QUATERNION_Y,
  QUATERNION_Z,
  QUATERNION_W,
#ifdef USE_RATE_SYSTEM_MODEL
  RATE_X, // body frame
  RATE_Y, // body frame
  RATE_Z, // body frame
#endif // USE_RATE_SYSTEM_MODEL
  POSITION_X,
  POSITION_Y,
  POSITION_Z,
  VELOCITY_X,
  VELOCITY_Y,
  VELOCITY_Z,
  BaseDimension,

#ifndef USE_RATE_SYSTEM_MODEL
  RATE_X = -1,
  RATE_Y = -1,
  RATE_Z = -1,
#endif // USE_RATE_SYSTEM_MODEL
  ACCELERATION_X = -1,
  ACCELERATION_Y = -1,
  ACCELERATION_Z = -1
};

FullState::FullState()
  : State(BaseDimension)
{
  orientation_index_  = QUATERNION_X;
  rate_index_         = RATE_X;
  position_index_     = POSITION_X;
  velocity_index_     = VELOCITY_X;
  acceleration_index_ = ACCELERATION_X;
}

OrientationOnlyState::OrientationOnlyState()
  : State(BaseDimension - 6)
{
  orientation_index_  = QUATERNION_X;
  rate_index_         = RATE_X;
  position_index_     = -1;
  velocity_index_     = -1;
  acceleration_index_ = -1;
}

#ifdef USE_RATE_SYSTEM_MODEL
PositionVelocityState::PositionVelocityState()
  : State(BaseDimension - 7)
{
  orientation_index_  = -1;
  rate_index_         = -1;
  position_index_     = POSITION_X - 7;
  velocity_index_     = VELOCITY_X - 7;
  acceleration_index_ = ACCELERATION_X - 7;
}
#else
PositionVelocityState::PositionVelocityState()
  : State(BaseDimension - 4)
{
  orientation_index_  = -1;
  rate_index_         = -1;
  position_index_     = POSITION_X - 4;
  velocity_index_     = VELOCITY_X - 4;
  acceleration_index_ = ACCELERATION_X - 4;
}
#endif // USE_RATE_SYSTEM_MODEL

State::State(IndexType base_dimension)
  : vector_(base_dimension)
  , covariance_(base_dimension)
  , base_dimension_(base_dimension)
  , base_(new SubState_<Dynamic>(*this, base_dimension))
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

void State::setOrientation(const Quaternion& orientation)
{
  setOrientation(orientation.coeffs());
}

void State::setRollPitch(const Quaternion& orientation)
{
  ColumnVector3 euler = orientation.matrix().eulerAngles(2,1,0);
  setRollPitch(euler(2), euler(1));
}

void State::setRollPitch(ScalarType roll, ScalarType pitch)
{
  ColumnVector3 euler = Quaternion(this->orientation()).matrix().eulerAngles(2,1,0);
  this->orientation() = Quaternion(Eigen::AngleAxis<ScalarType>(euler(0), ColumnVector3::UnitZ()) * Eigen::AngleAxis<ScalarType>(pitch, ColumnVector3::UnitY()) * Eigen::AngleAxis<ScalarType>(roll, ColumnVector3::UnitX())).coeffs();
  rollpitchSet();
}

void State::setYaw(const Quaternion& orientation)
{
  ColumnVector3 euler = orientation.matrix().eulerAngles(2,1,0);
  setYaw(euler(0));
}

void State::setYaw(ScalarType yaw)
{
  ColumnVector3 euler = Quaternion(this->orientation()).matrix().eulerAngles(2,1,0);
  this->orientation() = Quaternion(Eigen::AngleAxis<ScalarType>(yaw, ColumnVector3::UnitZ()) * Eigen::AngleAxis<ScalarType>(euler(1), ColumnVector3::UnitY()) * Eigen::AngleAxis<ScalarType>(euler(2), ColumnVector3::UnitX())).coeffs();
  yawSet();
}

void State::orientationSet() {
    if (getOrientationIndex() >= 0) {
        Matrix_<Dimension,Dimension> T(Matrix_<Dimension,Dimension>::Identity(getDimension(),getDimension()));
        T(getOrientationIndex(X),getOrientationIndex(X)) = 0.0;
        T(getOrientationIndex(Y),getOrientationIndex(Y)) = 0.0;
        T(getOrientationIndex(Z),getOrientationIndex(Z)) = 0.0;
        T(getOrientationIndex(W),getOrientationIndex(W)) = 0.0;
        P() = T * P() * T;
    }
    system_status_ |= STATE_ROLLPITCH | STATE_YAW;
}

void State::rollpitchSet() {
    if (getOrientationIndex() >= 0) {
        Matrix_<Dimension,Dimension> T(Matrix_<Dimension,Dimension>::Identity(getDimension(),getDimension()));
        const ConstOrientationType &q = getOrientation();
        Matrix_<3,4> W; W <<
            q.w(), -q.z(),  q.y(), -q.x(),
            q.z(),  q.w(), -q.x(), -q.y(),
           -q.y(),  q.x(),  q.w(), -q.z();
        Matrix_<3,3> W_P_WT(W * P().block<4,4>(getOrientationIndex(),getOrientationIndex()) * W.transpose());
        const Eigen::internal::inverse_impl< Matrix_<3,3>::Base > W_P_WT_inv(W_P_WT.inverse());
        Matrix_<3,3> R; R(2,2) = W_P_WT(2,2);
        P() = P() - P().middleCols<4>(getOrientationIndex()) * W.transpose() * (W_P_WT_inv - W_P_WT_inv * R * W_P_WT_inv) * W * P().middleRows<4>(getOrientationIndex());
    }
    system_status_ |= STATE_ROLLPITCH;
}

void State::yawSet() {
  if (getOrientationIndex() >= 0) {
      Matrix_<Dimension,Dimension> T(Matrix_<Dimension,Dimension>::Identity(getDimension(),getDimension()));
      const ConstOrientationType &q = getOrientation();
      Matrix_<3,4> W; W <<
          q.w(), -q.z(),  q.y(), -q.x(),
          q.z(),  q.w(), -q.x(), -q.y(),
         -q.y(),  q.x(),  q.w(), -q.z();
      Matrix_<3,3> W_P_WT(W * P().block<4,4>(getOrientationIndex(),getOrientationIndex()) * W.transpose());
      const Eigen::internal::inverse_impl< Matrix_<3,3>::Base > W_P_WT_inv(W_P_WT.inverse());
      Matrix_<3,3> R; R.row(2).setZero(); R.row(2).setZero();
      P() = P() - P().middleCols<4>(getOrientationIndex()) * W.transpose() * (W_P_WT_inv - W_P_WT_inv * R * W_P_WT_inv) * W * P().middleRows<4>(getOrientationIndex());
  }
    system_status_ |= STATE_YAW;
}

void State::rateSet() {
    if (getRateIndex() >= 0) {
        Matrix_<Dimension,Dimension> T(Matrix_<Dimension,Dimension>::Identity(getDimension(),getDimension()));
        T(getRateIndex(X),getRateIndex(X)) = 0.0;
        T(getRateIndex(Y),getRateIndex(Y)) = 0.0;
        T(getRateIndex(Z),getRateIndex(Z)) = 0.0;
        P() = T * P() * T;
    }
    system_status_ |= STATE_RATE_XY | STATE_RATE_Z;
}

void State::positionSet() {
    if (getPositionIndex() >= 0) {
        Matrix_<Dimension,Dimension> T(Matrix_<Dimension,Dimension>::Identity(getDimension(),getDimension()));
        T(getPositionIndex(X),getPositionIndex(X)) = 0.0;
        T(getPositionIndex(Y),getPositionIndex(Y)) = 0.0;
        T(getPositionIndex(Z),getPositionIndex(Z)) = 0.0;
        P() = T * P() * T;
    }
    system_status_ |= STATE_POSITION_XY | STATE_POSITION_Z;
}

void State::velocitySet() {
    if (getVelocityIndex() >= 0) {
        Matrix_<Dimension,Dimension> T(Matrix_<Dimension,Dimension>::Identity(getDimension(),getDimension()));
        T(getVelocityIndex(X),getVelocityIndex(X)) = 0.0;
        T(getVelocityIndex(Y),getVelocityIndex(Y)) = 0.0;
        T(getVelocityIndex(Z),getVelocityIndex(Z)) = 0.0;
        P() = T * P() * T;
    }
    system_status_ |= STATE_VELOCITY_XY | STATE_VELOCITY_Z;
}

void State::accelerationSet() {
    if (getAccelerationIndex() >= 0) {
        Matrix_<Dimension,Dimension> T(Matrix_<Dimension,Dimension>::Identity(getDimension(),getDimension()));
        T(getAccelerationIndex(X),getAccelerationIndex(X)) = 0.0;
        T(getAccelerationIndex(Y),getAccelerationIndex(Y)) = 0.0;
        T(getAccelerationIndex(Z),getAccelerationIndex(Z)) = 0.0;
        P() = T * P() * T;
    }
}

template <>
typename SubState_<Dynamic>::ConstVectorSegment SubState_<Dynamic>::getVector() const { return state_.getVector().segment(index_,dimension_); }

template <>
typename SubState_<Dynamic>::ConstCovarianceBlock SubState_<Dynamic>::getCovariance() const { return state_.getCovariance().block(index_,index_,dimension_,dimension_); }

template <>
typename SubState_<Dynamic>::VectorSegment SubState_<Dynamic>::x() { return state_.x().segment(index_,dimension_); }

template <>
typename SubState_<Dynamic>::CovarianceBlock SubState_<Dynamic>::P() { return state_.P().block(index_,index_,dimension_,dimension_); }

template <>
typename SubState_<Dynamic>::CrossVarianceBlock SubState_<Dynamic>::P01() { return state_.P().block(index_,index_,dimension_,dimension_); }

template class SubState_<Dynamic>;

} // namespace hector_pose_estimation
