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

#include <hector_pose_estimation/measurements/gravity.h>
#include <hector_pose_estimation/pose_estimation.h>

namespace hector_pose_estimation {

GravityModel::GravityModel()
  : MeasurementModel(MeasurementDimension)
  , gravity_(0.0)
{
  parameters().add("stddev", stddev_, 10.0);
}

bool GravityModel::init()
{
  NoiseCovariance noise = 0.0;
  noise(1,1) = noise(2,2) = noise(3,3) = pow(stddev_, 2);
  this->AdditiveNoiseSigmaSet(noise);
  return true;
}

GravityModel::~GravityModel() {}

bool GravityModel::applyStatusMask(const SystemStatus &status) const {
  if (status & STATE_ROLLPITCH) return false;
  return true;
}

SystemStatus GravityModel::getStatusFlags() const {
  return STATE_ROLLPITCH;
}

ColumnVector GravityModel::ExpectedValueGet() const {
  const double qw = x_(QUATERNION_W);
  const double qx = x_(QUATERNION_X);
  const double qy = x_(QUATERNION_Y);
  const double qz = x_(QUATERNION_Z);

  // y = q * [0 0 1] * q';
  this->y_(1) = -gravity_ * (2*qx*qz - 2*qw*qy);
  this->y_(2) = -gravity_ * (2*qw*qx + 2*qy*qz);
  this->y_(3) = -gravity_ * (qw*qw - qx*qx - qy*qy + qz*qz);

  return y_;
}

Matrix GravityModel::dfGet(unsigned int i) const {
  if (i != 0) return Matrix();

  const double qw = x_(QUATERNION_W);
  const double qx = x_(QUATERNION_X);
  const double qy = x_(QUATERNION_Y);
  const double qz = x_(QUATERNION_Z);

  C_(1,QUATERNION_W) =  gravity_*2*qy;
  C_(1,QUATERNION_X) = -gravity_*2*qz;
  C_(1,QUATERNION_Y) =  gravity_*2*qw;
  C_(1,QUATERNION_Z) = -gravity_*2*qx;
  C_(2,QUATERNION_W) = -gravity_*2*qx;
  C_(2,QUATERNION_X) = -gravity_*2*qw;
  C_(2,QUATERNION_Y) = -gravity_*2*qz;
  C_(2,QUATERNION_Z) = -gravity_*2*qy;
  C_(3,QUATERNION_W) = -gravity_*2*qw;
  C_(3,QUATERNION_X) =  gravity_*2*qx;
  C_(3,QUATERNION_Y) =  gravity_*2*qy;
  C_(3,QUATERNION_Z) = -gravity_*2*qz;

  return C_;
}

bool Gravity::beforeUpdate(PoseEstimation &estimator, const Update &update) {
  model_->setGravity(estimator.getSystemModel()->getGravity());
  return true;
}

} // namespace hector_pose_estimation
