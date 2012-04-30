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

#include <hector_pose_estimation/system_model.h>

namespace hector_pose_estimation {

SystemModel::SystemModel()
  : BFL::AnalyticConditionalGaussianAdditiveNoise(StateDimension, 2)
  , BFL::AnalyticSystemModelGaussianUncertainty(this)
  , dt_(0.0)
  , x_(static_cast<const StateVector&>(ConditionalArgumentGet(0)))
  , u_(static_cast<const InputVector&>(ConditionalArgumentGet(1)))
  , x_pred_(StateDimension)
  , A_(StateDimension,StateDimension)
  , measurement_status_(0)
{
  A_ = 0.0;
  for(unsigned int i = 1; i <= StateDimension; i++) A_(i,i) = 1.0;
  AdditiveNoiseMuSet(StateVector(0.0));
  AdditiveNoiseSigmaSet(SymmetricMatrix_<StateDimension>(0.0));
}

SystemModel::~SystemModel()
{
}

void SystemModel::getPrior(BFL::Gaussian &prior) const {
  StateVector mu = 0;
  mu(QUATERNION_W) = 1.0;
  StateCovariance cov = 0;
  cov(QUATERNION_W,QUATERNION_W) = 0.25 * 1.0;
  cov(QUATERNION_X,QUATERNION_X) = 0.25 * 1.0;
  cov(QUATERNION_Y,QUATERNION_Y) = 0.25 * 1.0;
  cov(QUATERNION_Z,QUATERNION_Z) = 0.25 * 1.0;
  cov(POSITION_X,POSITION_X) = 1.0;
  cov(POSITION_Y,POSITION_Y) = 1.0;
  cov(POSITION_Z,POSITION_Z) = 1.0;
  cov(VELOCITY_X,VELOCITY_X) = 1.0;
  cov(VELOCITY_Y,VELOCITY_Y) = 1.0;
  cov(VELOCITY_Z,VELOCITY_Z) = 1.0;
  cov(BIAS_ACCEL_X,BIAS_ACCEL_X) = 0.0;
  cov(BIAS_ACCEL_Y,BIAS_ACCEL_Y) = 0.0;
  cov(BIAS_ACCEL_Z,BIAS_ACCEL_Z) = 0.0;
  cov(BIAS_GYRO_X,BIAS_GYRO_X) = 0.0;
  cov(BIAS_GYRO_Y,BIAS_GYRO_Y) = 0.0;
  cov(BIAS_GYRO_Z,BIAS_GYRO_Z) = 0.0;

  prior.ExpectedValueSet(mu);
  prior.CovarianceSet(cov);
}

SymmetricMatrix SystemModel::CovarianceGet(double dt) const
{
//  return this->AdditiveNoiseSigmaGet() * (dt*dt);
  return this->AdditiveNoiseSigmaGet() * dt;
}

} // namespace hector_pose_estimation
