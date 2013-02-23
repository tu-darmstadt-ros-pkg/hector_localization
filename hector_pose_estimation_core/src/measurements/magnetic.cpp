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
#include <hector_pose_estimation/pose_estimation.h>

namespace hector_pose_estimation {

MagneticModel::MagneticModel()
  : MeasurementModel(MeasurementDimension)
  , declination_(0.0), inclination_(60.0 * M_PI/180.0), magnitude_(0.0)
  , C_full_(3,StateDimension)
{
  parameters().add("stddev", stddev_, 1.0);
  parameters().add("declination", declination_);
  parameters().add("inclination", inclination_);
  parameters().add("magnitude", magnitude_);

  C_full_= 0.0;
}

MagneticModel::~MagneticModel() {}

bool MagneticModel::init()
{
  NoiseCovariance noise = 0.0;
  noise(1,1) = noise(2,2) = noise(3,3) = pow(stddev_, 2);
  this->AdditiveNoiseSigmaSet(noise);

  updateMagneticField();
  return true;
}

SystemStatus MagneticModel::getStatusFlags() const {
  return STATE_YAW;
}

void MagneticModel::setReference(const GlobalReference::Heading &reference_heading) {
  magnetic_field_reference_(1) = reference_heading.cos * magnetic_field_north_(1) - reference_heading.sin * magnetic_field_north_(2);
  magnetic_field_reference_(2) = reference_heading.sin * magnetic_field_north_(1) + reference_heading.cos * magnetic_field_north_(2);
  magnetic_field_reference_(3) = magnetic_field_north_(3);
}

ColumnVector MagneticModel::ExpectedValueGet() const {
  const double qw = x_(QUATERNION_W);
  const double qx = x_(QUATERNION_X);
  const double qy = x_(QUATERNION_Y);
  const double qz = x_(QUATERNION_Z);

  y_(1) = (qw*qw+qx*qx-qy*qy-qz*qz) * magnetic_field_reference_(1) + (2.0*qx*qy+2.0*qw*qz)     * magnetic_field_reference_(2) + (2.0*qx*qz-2.0*qw*qy)     * magnetic_field_reference_(3);
  y_(2) = (2.0*qx*qy-2.0*qw*qz)     * magnetic_field_reference_(1) + (qw*qw-qx*qx+qy*qy-qz*qz) * magnetic_field_reference_(2) + (2.0*qy*qz+2.0*qw*qx)     * magnetic_field_reference_(3);
  y_(3) = (2.0*qx*qz+2.0*qw*qy)     * magnetic_field_reference_(1) + (2.0*qy*qz-2.0*qw*qx)     * magnetic_field_reference_(2) + (qw*qw-qx*qx-qy*qy+qz*qz) * magnetic_field_reference_(3);

  return y_;
}

Matrix MagneticModel::dfGet(unsigned int i) const {
  if (i != 0) return Matrix();

  const double qw = x_(QUATERNION_W);
  const double qx = x_(QUATERNION_X);
  const double qy = x_(QUATERNION_Y);
  const double qz = x_(QUATERNION_Z);

  C_full_(1,QUATERNION_W) =  2.0*qw * magnetic_field_reference_(1) + 2.0*qz * magnetic_field_reference_(2) - 2.0*qy * magnetic_field_reference_(3);
  C_full_(1,QUATERNION_X) =  2.0*qx * magnetic_field_reference_(1) + 2.0*qy * magnetic_field_reference_(2) + 2.0*qz * magnetic_field_reference_(3);
  C_full_(1,QUATERNION_Y) = -2.0*qy * magnetic_field_reference_(1) + 2.0*qx * magnetic_field_reference_(2) - 2.0*qw * magnetic_field_reference_(3);
  C_full_(1,QUATERNION_Z) = -2.0*qz * magnetic_field_reference_(1) + 2.0*qw * magnetic_field_reference_(2) + 2.0*qx * magnetic_field_reference_(3);
  C_full_(2,QUATERNION_W) = -2.0*qz * magnetic_field_reference_(1) + 2.0*qw * magnetic_field_reference_(2) + 2.0*qx * magnetic_field_reference_(3);
  C_full_(2,QUATERNION_X) =  2.0*qy * magnetic_field_reference_(1) - 2.0*qx * magnetic_field_reference_(2) + 2.0*qw * magnetic_field_reference_(3);
  C_full_(2,QUATERNION_Y) =  2.0*qx * magnetic_field_reference_(1) + 2.0*qy * magnetic_field_reference_(2) + 2.0*qz * magnetic_field_reference_(3);
  C_full_(2,QUATERNION_Z) = -2.0*qw * magnetic_field_reference_(1) - 2.0*qz * magnetic_field_reference_(2) + 2.0*qy * magnetic_field_reference_(3);
  C_full_(3,QUATERNION_W) =  2.0*qy * magnetic_field_reference_(1) - 2.0*qx * magnetic_field_reference_(2) + 2.0*qw * magnetic_field_reference_(3);
  C_full_(3,QUATERNION_X) =  2.0*qz * magnetic_field_reference_(1) - 2.0*qw * magnetic_field_reference_(2) - 2.0*qx * magnetic_field_reference_(3);
  C_full_(3,QUATERNION_Y) =  2.0*qw * magnetic_field_reference_(1) + 2.0*qz * magnetic_field_reference_(2) - 2.0*qy * magnetic_field_reference_(3);
  C_full_(3,QUATERNION_Z) =  2.0*qx * magnetic_field_reference_(1) + 2.0*qy * magnetic_field_reference_(2) + 2.0*qz * magnetic_field_reference_(3);

  // return C_full_;

  // q = [qw qx qy qz]';
  // dq/dyaw * dyaw*dq = 1/2 * [-qz -qy qx qw]' * 2 * [-qz; -qy; qx; qw] =
  //  [ qz*qz  qz*qy -qz*qx -qz*qw ;
  //    qy*qz  qy*qy -qy*qx -qy*qw ;
  //   -qx*qz -qx*qy  qx*qx  qx*qw ;
  //   -qw*qz -qw*qy  qw*qx  qw*qw ]

  for(int i = 1; i <= 3; ++i) {
    C_(i,QUATERNION_W) =  C_full_(i,QUATERNION_W) * qz*qz + C_full_(i,QUATERNION_X) * qy*qz - C_full_(i,QUATERNION_Y) * qx*qz - C_full_(i,QUATERNION_Z) * qw*qz;
    C_(i,QUATERNION_X) =  C_full_(i,QUATERNION_W) * qz*qy + C_full_(i,QUATERNION_X) * qy*qy - C_full_(i,QUATERNION_Y) * qx*qy - C_full_(i,QUATERNION_Z) * qw*qy;
    C_(i,QUATERNION_Y) = -C_full_(i,QUATERNION_W) * qz*qx - C_full_(i,QUATERNION_X) * qy*qx + C_full_(i,QUATERNION_Y) * qx*qx + C_full_(i,QUATERNION_Z) * qw*qx;
    C_(i,QUATERNION_Z) = -C_full_(i,QUATERNION_W) * qz*qw - C_full_(i,QUATERNION_X) * qy*qw + C_full_(i,QUATERNION_Y) * qx*qw + C_full_(i,QUATERNION_Z) * qw*qw;
  }

  return C_;
}

double MagneticModel::getMagneticHeading(const MeasurementVector &y) const {
  return -(-atan2(y(2), y(1)));
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

  magnetic_field_north_(1) = magnitude * (cos_inclination * cos_declination);
  magnetic_field_north_(2) = magnitude * (-sin_declination);
  magnetic_field_north_(3) = magnitude * (-sin_inclination * cos_declination);
}

Magnetic::Magnetic(const std::string &name)
  : Measurement_<MagneticModel>(name)
  , auto_heading_(true)
  , reference_(0)
{
  parameters().add("auto_heading", auto_heading_);
}

void Magnetic::onReset() {
  reference_ = 0;
}

const MagneticModel::MeasurementVector& Magnetic::getVector(const Magnetic::Update& update) {
  if (getModel()->hasMagnitude()) return Measurement_<MagneticModel>::getVector(update);

  y_ = Measurement_<MagneticModel>::getVector(update);
  double c = 1.0 / y_.norm();
  if (isinf(c)) {
    y_ = MeasurementVector(0.0);
  } else {
    y_ = y_ * c;
  }
  return y_;
}

const MagneticModel::NoiseCovariance& Magnetic::getCovariance(const Magnetic::Update& update) {
  if (getModel()->hasMagnitude()) return Measurement_<MagneticModel>::getCovariance(update);

  R_ = Measurement_<MagneticModel>::getCovariance(update);
  double c = 1.0 / Measurement_<MagneticModel>::getVector(update).norm();
  if (isinf(c)) {
    R_ = NoiseCovariance(1.0);
  } else {
    R_ =  R_ * (c*c);
  }
  return R_;
}

bool Magnetic::beforeUpdate(PoseEstimation &estimator, const Magnetic::Update &update) {
  // reset reference position if Magnetic has not been updated for a while
  if (timedout()) reference_ = 0;

  if (reference_ != estimator.globalReference()) {
    reference_ = estimator.globalReference();

    if (auto_heading_) {
      double yaw, pitch, roll;
      estimator.getOrientation(yaw, pitch, roll);
      reference_->setHeading(getModel()->getTrueHeading(update.getVector()) - (-yaw));
      ROS_INFO("Set new reference heading to %.1f degress", reference_->heading() * 180.0 / M_PI);
    }
  }

  getModel()->setReference(reference_->heading());
  return true;
}

} // namespace hector_pose_estimation
