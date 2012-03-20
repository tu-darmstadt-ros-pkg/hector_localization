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

#include <hector_pose_estimation/measurements/heading.h>

namespace hector_pose_estimation {

HeadingModel::HeadingModel()
  : MeasurementModel(1)
{
  SymmetricMatrix noise(1);
  parameters().add("stddev", stddev_, 10.0*M_PI/180.0);
  noise(1,1) = pow(stddev_, 2);
  this->AdditiveNoiseSigmaSet(noise);
}

HeadingModel::~HeadingModel() {}

bool HeadingModel::applyStatusMask(const SystemStatus &status) const {
  if (status & STATE_YAW) return false;
  return true;
}

SystemStatus HeadingModel::getStatusFlags() const {
  return STATE_YAW;
}

ColumnVector HeadingModel::ExpectedValueGet() const {
  const double qw = x_(QUATERNION_W);
  const double qx = x_(QUATERNION_X);
  const double qy = x_(QUATERNION_Y);
  const double qz = x_(QUATERNION_Z);

  y_(1) = atan2(2*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz);

  return y_;
}

Matrix HeadingModel::dfGet(unsigned int i) const {
  if (i != 0) return Matrix();

  const double qw = x_(QUATERNION_W);
  const double qx = x_(QUATERNION_X);
  const double qy = x_(QUATERNION_Y);
  const double qz = x_(QUATERNION_Z);
  const double t1 = qw*qw + qx*qx - qy*qy - qz*qz;
  const double t2 = 2*(qx*qy + qw*qz);
  const double t3 = 1.0 / (t1*t1 + t2*t2);

  C_(1,QUATERNION_W) = 2.0 * t3 * (qz * t1 - qw * t2);
  C_(1,QUATERNION_X) = 2.0 * t3 * (qy * t1 - qx * t2);
  C_(1,QUATERNION_Y) = 2.0 * t3 * (qx * t1 + qy * t2);
  C_(1,QUATERNION_Z) = 2.0 * t3 * (qw * t1 + qz * t2);

  return C_;
}

} // namespace hector_pose_estimation
