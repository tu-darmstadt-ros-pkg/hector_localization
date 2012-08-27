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

#include <hector_pose_estimation/measurements/rate.h>

namespace hector_pose_estimation {

RateModel::RateModel()
  : MeasurementModel(MeasurementDimension)
{
  parameters().add("stddev", stddev_, 1.0*M_PI/180.0);
}

bool RateModel::init()
{
  NoiseCovariance noise = 0.0;
  noise(1,1) = noise(2,2) = noise(3,3) = pow(stddev_, 2);
  this->AdditiveNoiseSigmaSet(noise);
  return true;
}

RateModel::~RateModel() {}

ColumnVector RateModel::ExpectedValueGet() const {
#ifdef USE_RATE_SYSTEM_MODEL
  const double qw = x_(QUATERNION_W);
  const double qx = x_(QUATERNION_X);
  const double qy = x_(QUATERNION_Y);
  const double qz = x_(QUATERNION_Z);

  y_(1) = (qw*qw+qx*qx-qy*qy-qz*qz) * x_(RATE_X) + (2.0*qx*qy+2.0*qw*qz)     * x_(RATE_Y) + (2.0*qx*qz-2.0*qw*qy)     * x_(RATE_Z) + x_(BIAS_GYRO_X);
  y_(2) = (2.0*qx*qy-2.0*qw*qz)     * x_(RATE_X) + (qw*qw-qx*qx+qy*qy-qz*qz) * x_(RATE_Y) + (2.0*qy*qz+2.0*qw*qx)     * x_(RATE_Z) + x_(BIAS_GYRO_Y);
  y_(3) = (2.0*qx*qz+2.0*qw*qy)     * x_(RATE_X) + (2.0*qy*qz-2.0*qw*qx)     * x_(RATE_Y) + (qw*qw-qx*qx-qy*qy+qz*qz) * x_(RATE_Z) + x_(BIAS_GYRO_Z);
#else // USE_RATE_SYSTEM_MODEL
  y_(1) = 0.0;
  y_(2) = 0.0;
  y_(3) = 0.0;
#endif // USE_RATE_SYSTEM_MODEL

  return y_;
}

Matrix RateModel::dfGet(unsigned int i) const {
  if (i != 0) return Matrix();

#ifdef USE_RATE_SYSTEM_MODEL
  const double qw = x_(QUATERNION_W);
  const double qx = x_(QUATERNION_X);
  const double qy = x_(QUATERNION_Y);
  const double qz = x_(QUATERNION_Z);

  //  C_(1,QUATERNION_W) =  2.0*qw * x_(RATE_X) + 2.0*qz * x_(RATE_Y) - 2.0*qy * x_(RATE_Z);
  //  C_(1,QUATERNION_X) =  2.0*qx * x_(RATE_X) + 2.0*qy * x_(RATE_Y) + 2.0*qz * x_(RATE_Z);
  //  C_(1,QUATERNION_Y) = -2.0*qy * x_(RATE_X) + 2.0*qx * x_(RATE_Y) - 2.0*qw * x_(RATE_Z);
  //  C_(1,QUATERNION_Z) = -2.0*qz * x_(RATE_X) + 2.0*qw * x_(RATE_Y) + 2.0*qx * x_(RATE_Z);
  //  C_(2,QUATERNION_W) = -2.0*qz * x_(RATE_X) + 2.0*qw * x_(RATE_Y) + 2.0*qx * x_(RATE_Z);
  //  C_(2,QUATERNION_X) =  2.0*qy * x_(RATE_X) - 2.0*qx * x_(RATE_Y) + 2.0*qw * x_(RATE_Z);
  //  C_(2,QUATERNION_Y) =  2.0*qx * x_(RATE_X) + 2.0*qy * x_(RATE_Y) + 2.0*qz * x_(RATE_Z);
  //  C_(2,QUATERNION_Z) = -2.0*qw * x_(RATE_X) - 2.0*qz * x_(RATE_Y) + 2.0*qy * x_(RATE_Z);
  //  C_(3,QUATERNION_W) =  2.0*qy * x_(RATE_X) - 2.0*qx * x_(RATE_Y) + 2.0*qw * x_(RATE_Z);
  //  C_(3,QUATERNION_X) =  2.0*qz * x_(RATE_X) - 2.0*qw * x_(RATE_Y) - 2.0*qx * x_(RATE_Z);
  //  C_(3,QUATERNION_Y) =  2.0*qw * x_(RATE_X) + 2.0*qz * x_(RATE_Y) - 2.0*qy * x_(RATE_Z);
  //  C_(3,QUATERNION_Z) =  2.0*qx * x_(RATE_X) + 2.0*qy * x_(RATE_Y) + 2.0*qz * x_(RATE_Z);

  C_(1,RATE_X) = (qw*qw+qx*qx-qy*qy-qz*qz);
  C_(1,RATE_Y) = (2.0*qx*qy+2.0*qw*qz);
  C_(1,RATE_Z) = (2.0*qx*qz-2.0*qw*qy);
  C_(2,RATE_X) = (2.0*qx*qy-2.0*qw*qz);
  C_(2,RATE_Y) = (qw*qw-qx*qx+qy*qy-qz*qz);
  C_(2,RATE_Z) = (2.0*qy*qz+2.0*qw*qx);
  C_(3,RATE_X) = (2.0*qx*qz+2.0*qw*qy);
  C_(3,RATE_Y) = (2.0*qy*qz-2.0*qw*qx);
  C_(3,RATE_Z) = (qw*qw-qx*qx-qy*qy+qz*qz);

  C_(1,BIAS_GYRO_X) = 1.0;
  C_(2,BIAS_GYRO_Y) = 1.0;
  C_(3,BIAS_GYRO_Z) = 1.0;
#endif // USE_RATE_SYSTEM_MODEL

  return C_;
}

} // namespace hector_pose_estimation
