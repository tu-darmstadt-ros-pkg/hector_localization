//=================================================================================================
// Copyright (c) 2011, Johannes Meyer, TU Darmstadt
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

#include <hector_pose_estimation/measurements/magnetic.h>
#include <hector_pose_estimation/filter/set_filter.h>

#include <Eigen/Geometry>

namespace hector_pose_estimation {

template class Measurement_<MagneticModel>;

MagneticModel::MagneticModel()
  : declination_(0.0), inclination_(60.0 * M_PI/180.0), magnitude_(0.0)
  , C_full_(3,StateDimension)
{
  parameters().add("stddev", stddev_, 1.0);
  parameters().add("declination", declination_);
  parameters().add("inclination", inclination_);
  parameters().add("magnitude", magnitude_);

  C_full_.setZero();
}

MagneticModel::~MagneticModel() {}

bool MagneticModel::init(PoseEstimation &estimator, State &state)
{
  updateMagneticField();
  return true;
}

void MagneticModel::setReference(const GlobalReference::Heading &reference_heading) {
  magnetic_field_reference_.x() = reference_heading.cos * magnetic_field_north_.x() - reference_heading.sin * magnetic_field_north_.y();
  magnetic_field_reference_.y() = reference_heading.sin * magnetic_field_north_.x() + reference_heading.cos * magnetic_field_north_.y();
  magnetic_field_reference_.z() = magnetic_field_north_.z();
}

void MagneticModel::getMeasurementNoise(NoiseVariance& R, const State&, bool init)
{
  if (init) {
    R(0,0) = R(1,1) = R(2,2) = pow(stddev_, 2);
  }
}

void MagneticModel::getExpectedValue(MeasurementVector& y_pred, const State& state)
{
  State::ConstOrientationType q(state.getOrientation());

  y_pred(0) = (q.w()*q.w()+q.x()*q.x()-q.y()*q.y()-q.z()*q.z()) * magnetic_field_reference_(0) + (2.0*q.x()*q.y()+2.0*q.w()*q.z())                 * magnetic_field_reference_.y() + (2.0*q.x()*q.z()-2.0*q.w()*q.y())                 * magnetic_field_reference_.z();
  y_pred(1) = (2.0*q.x()*q.y()-2.0*q.w()*q.z())                 * magnetic_field_reference_(0) + (q.w()*q.w()-q.x()*q.x()+q.y()*q.y()-q.z()*q.z()) * magnetic_field_reference_.y() + (2.0*q.y()*q.z()+2.0*q.w()*q.x())                 * magnetic_field_reference_.z();
  y_pred(2) = (2.0*q.x()*q.z()+2.0*q.w()*q.y())                 * magnetic_field_reference_(0) + (2.0*q.y()*q.z()-2.0*q.w()*q.x())                 * magnetic_field_reference_.y() + (q.w()*q.w()-q.x()*q.x()-q.y()*q.y()+q.z()*q.z()) * magnetic_field_reference_.z();
}

void MagneticModel::getStateJacobian(MeasurementMatrix& C, const State& state, bool)
{
  State::ConstOrientationType q(state.getOrientation());

  if (state.getOrientationIndex() >= 0) {
    C_full_(0,State::QUATERNION_W) =  2.0*q.w() * magnetic_field_reference_.x() + 2.0*q.z() * magnetic_field_reference_.y() - 2.0*q.y() * magnetic_field_reference_.z();
    C_full_(0,State::QUATERNION_X) =  2.0*q.x() * magnetic_field_reference_.x() + 2.0*q.y() * magnetic_field_reference_.y() + 2.0*q.z() * magnetic_field_reference_.z();
    C_full_(0,State::QUATERNION_Y) = -2.0*q.y() * magnetic_field_reference_.x() + 2.0*q.x() * magnetic_field_reference_.y() - 2.0*q.w() * magnetic_field_reference_.z();
    C_full_(0,State::QUATERNION_Z) = -2.0*q.z() * magnetic_field_reference_.x() + 2.0*q.w() * magnetic_field_reference_.y() + 2.0*q.x() * magnetic_field_reference_.z();
    C_full_(1,State::QUATERNION_W) = -2.0*q.z() * magnetic_field_reference_.x() + 2.0*q.w() * magnetic_field_reference_.y() + 2.0*q.x() * magnetic_field_reference_.z();
    C_full_(1,State::QUATERNION_X) =  2.0*q.y() * magnetic_field_reference_.x() - 2.0*q.x() * magnetic_field_reference_.y() + 2.0*q.w() * magnetic_field_reference_.z();
    C_full_(1,State::QUATERNION_Y) =  2.0*q.x() * magnetic_field_reference_.x() + 2.0*q.y() * magnetic_field_reference_.y() + 2.0*q.z() * magnetic_field_reference_.z();
    C_full_(1,State::QUATERNION_Z) = -2.0*q.w() * magnetic_field_reference_.x() - 2.0*q.z() * magnetic_field_reference_.y() + 2.0*q.y() * magnetic_field_reference_.z();
    C_full_(2,State::QUATERNION_W) =  2.0*q.y() * magnetic_field_reference_.x() - 2.0*q.x() * magnetic_field_reference_.y() + 2.0*q.w() * magnetic_field_reference_.z();
    C_full_(2,State::QUATERNION_X) =  2.0*q.z() * magnetic_field_reference_.x() - 2.0*q.w() * magnetic_field_reference_.y() - 2.0*q.x() * magnetic_field_reference_.z();
    C_full_(2,State::QUATERNION_Y) =  2.0*q.w() * magnetic_field_reference_.x() + 2.0*q.z() * magnetic_field_reference_.y() - 2.0*q.y() * magnetic_field_reference_.z();
    C_full_(2,State::QUATERNION_Z) =  2.0*q.x() * magnetic_field_reference_.x() + 2.0*q.y() * magnetic_field_reference_.y() + 2.0*q.z() * magnetic_field_reference_.z();

    // return C_full_;

    // q = [qw qx qy qz]';
    // dq/dyaw * dyaw*dq = 1/2 * [-qz -qy qx qw]' * 2 * [-qz; -qy; qx; qw] =
    //  [ qz*qz  qz*qy -qz*qx -qz*qw ;
    //    qy*qz  qy*qy -qy*qx -qy*qw ;
    //   -qx*qz -qx*qy  qx*qx  qx*qw ;
    //   -qw*qz -qw*qy  qw*qx  qw*qw ]

    for(int i = 0; i <= 2; ++i) {
      C(i,State::QUATERNION_W) =  C_full_(i,State::QUATERNION_W) * q.z()*q.z() + C_full_(i,State::QUATERNION_X) * q.y()*q.z() - C_full_(i,State::QUATERNION_Y) * q.x()*q.z() - C_full_(i,State::QUATERNION_Z) * q.w()*q.z();
      C(i,State::QUATERNION_X) =  C_full_(i,State::QUATERNION_W) * q.z()*q.y() + C_full_(i,State::QUATERNION_X) * q.y()*q.y() - C_full_(i,State::QUATERNION_Y) * q.x()*q.y() - C_full_(i,State::QUATERNION_Z) * q.w()*q.y();
      C(i,State::QUATERNION_Y) = -C_full_(i,State::QUATERNION_W) * q.z()*q.x() - C_full_(i,State::QUATERNION_X) * q.y()*q.x() + C_full_(i,State::QUATERNION_Y) * q.x()*q.x() + C_full_(i,State::QUATERNION_Z) * q.w()*q.x();
      C(i,State::QUATERNION_Z) = -C_full_(i,State::QUATERNION_W) * q.z()*q.w() - C_full_(i,State::QUATERNION_X) * q.y()*q.w() + C_full_(i,State::QUATERNION_Y) * q.x()*q.w() + C_full_(i,State::QUATERNION_Z) * q.w()*q.w();
    }
  }
}

double MagneticModel::getMagneticHeading(const MeasurementVector &y) const {
  return -(-atan2(y.y(), y.x()));
}

double MagneticModel::getTrueHeading(const MeasurementVector &y) const {
  return getMagneticHeading(y) + declination_;
}

void MagneticModel::updateMagneticField()
{
  double cos_inclination, sin_inclination;
  sincos(inclination_, &sin_inclination, &cos_inclination);

  double cos_declination, sin_declination;
  sincos(declination_, &sin_declination, &cos_declination);

  // return normalized magnetic field if magnitude is zero
  double magnitude = magnitude_;
  if (magnitude == 0.0) magnitude = 1.0;

  magnetic_field_north_.x() = magnitude * (cos_inclination * cos_declination);
  magnetic_field_north_.y() = magnitude * (-sin_declination);
  magnetic_field_north_.z() = magnitude * (-sin_inclination * cos_declination);
}

Magnetic::Magnetic(const std::string &name)
  : Measurement_<MagneticModel>(name)
  , auto_heading_(true)
{
  parameters().add("auto_heading", auto_heading_);
}

void Magnetic::onReset() {
  reference_.reset();
}

const MagneticModel::MeasurementVector& Magnetic::getVector(const Magnetic::Update& update, const State& state) {
  if (getModel()->hasMagnitude()) return Measurement_<MagneticModel>::getVector(update, state);

  y_ = Measurement_<MagneticModel>::getVector(update, state);
  double c = 1.0 / y_.norm();
  if (isinf(c)) {
    y_ = MeasurementVector(0.0);
  } else {
    y_ = y_ * c;
  }
  return y_;
}

const MagneticModel::NoiseVariance& Magnetic::getVariance(const Magnetic::Update& update, const State& state) {
  if (getModel()->hasMagnitude()) return Measurement_<MagneticModel>::getVariance(update, state);

  R_ = Measurement_<MagneticModel>::getVariance(update, state);
  double c = 1.0 / Measurement_<MagneticModel>::getVector(update, state).norm();
  if (isinf(c)) {
    R_ = NoiseVariance(1.0);
  } else {
    R_ =  R_ * (c*c);
  }
  return R_;
}

bool Magnetic::prepareUpdate(State &state, const Update &update) {
  // reset reference position if Magnetic has not been updated for a while
  if (timedout()) reference_.reset();

  if (reference_ != GlobalReference::Instance()) {
    reference_ = GlobalReference::Instance();

    if (auto_heading_) {
      double yaw = Eigen::Quaterniond(state.getOrientation()).matrix().eulerAngles(2,1,0)[0];
      reference_->setHeading(getModel()->getTrueHeading(update.getVector()) - (-yaw));
      ROS_INFO("Set new reference heading to %.1f degress", reference_->heading() * 180.0 / M_PI);
    }
  }

  getModel()->setReference(reference_->heading());
  return true;
}

} // namespace hector_pose_estimation
