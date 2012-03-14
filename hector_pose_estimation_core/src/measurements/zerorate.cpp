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

#include <hector_pose_estimation/measurements/zerorate.h>

namespace hector_pose_estimation {

ZeroRateModel::ZeroRateModel()
  : MeasurementModel(1,3)
{
  SymmetricMatrix noise(1);
  parameters().add("stddev", stddev_, 4.0*M_PI);
  noise(1,1) = pow(stddev_, 2);
  this->AdditiveNoiseSigmaSet(noise);
}

ZeroRateModel::~ZeroRateModel() {}

bool ZeroRateModel::applyStatusMask(const SystemStatus &status) const {
  if (status & STATE_YAW) return false;
  return true;
}

ColumnVector ZeroRateModel::ExpectedValueGet() const {
  y_(1) = -x_(BIAS_GYRO_Z);
  return y_;
}

Matrix ZeroRateModel::dfGet(unsigned int i) const {
  if (i != 0) return Matrix();
  C_(1,BIAS_GYRO_Z)  = -1.0;
  return C_;
}

} // namespace hector_pose_estimation
