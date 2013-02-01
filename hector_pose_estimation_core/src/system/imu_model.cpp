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

#include <hector_pose_estimation/system/imu_model.h>

namespace hector_pose_estimation {

ImuModel::ImuModel()
{
  acceleration_drift_ = 1.0e-6;
  rate_drift_ = 1.0e-2 * M_PI/180.0;
  parameters().add("acceleration_drift", acceleration_drift_);
  parameters().add("rate_drift", rate_drift_);
}

ImuModel::~ImuModel()
{}

bool ImuModel::init(PoseEstimation& estimator, State& state)
{
  drift_ = state.addSubState(this);
  return drift_;
}

void ImuModel::getPrior(State &state)
{
  if (!drift_) return;
  drift_->x().setZero();

    drift_->P()(BIAS_ACCEL_X,BIAS_ACCEL_X)
  = drift_->P()(BIAS_ACCEL_Y,BIAS_ACCEL_Y)
  = drift_->P()(BIAS_ACCEL_Z,BIAS_ACCEL_Z) = 0.0;

    drift_->P()(BIAS_GYRO_X,BIAS_GYRO_X)
  = drift_->P()(BIAS_GYRO_Y,BIAS_GYRO_Y)
  = drift_->P()(BIAS_GYRO_Z,BIAS_GYRO_Z) = pow(5.0 * M_PI/180.0, 2);
}

void ImuModel::getSystemNoise(NoiseVariance& Q, const State& state, bool init)
{
  if (init) {
    Q(BIAS_ACCEL_X,BIAS_ACCEL_X) = Q(BIAS_ACCEL_Y,BIAS_ACCEL_Y) = pow(acceleration_drift_, 2);
    Q(BIAS_ACCEL_Z,BIAS_ACCEL_Z) = pow(acceleration_drift_, 2);
    Q(BIAS_GYRO_X,BIAS_GYRO_X) = Q(BIAS_GYRO_Y,BIAS_GYRO_Y) = Q(BIAS_GYRO_Z,BIAS_GYRO_Z) = pow(rate_drift_, 2);
  }
}

void ImuModel::getStateJacobian(SystemMatrix& A, const State& state, bool)
{
  const State::OrientationType& q = state.getOrientation();

  if (state.getOrientationIndex() >= 0) {
    A(State::QUATERNION_W, State::Dimension + BIAS_GYRO_X)  = -0.5*q.x();
    A(State::QUATERNION_W, State::Dimension + BIAS_GYRO_Y)  = -0.5*q.y();
    A(State::QUATERNION_W, State::Dimension + BIAS_GYRO_Z)  = -0.5*q.z();

    A(State::QUATERNION_X, State::Dimension + BIAS_GYRO_X)  =  0.5*q.w();
    A(State::QUATERNION_X, State::Dimension + BIAS_GYRO_Y)  = -0.5*q.z();
    A(State::QUATERNION_X, State::Dimension + BIAS_GYRO_Z)  = 0.5*q.y();

    A(State::QUATERNION_Y, State::Dimension + BIAS_GYRO_X)  = 0.5*q.z();
    A(State::QUATERNION_Y, State::Dimension + BIAS_GYRO_Y)  = 0.5*q.w();
    A(State::QUATERNION_Y, State::Dimension + BIAS_GYRO_Z)  = -0.5*q.x();

    A(State::QUATERNION_Z, State::Dimension + BIAS_GYRO_X)  = -0.5*q.y();
    A(State::QUATERNION_Z, State::Dimension + BIAS_GYRO_Y)  = 0.5*q.x();
    A(State::QUATERNION_Z, State::Dimension + BIAS_GYRO_Z)  = 0.5*q.w();
  }

  if (state.getVelocityIndex() >= 0) {
    if (state.getSystemStatus() & STATE_XY_VELOCITY) {
      A(State::VELOCITY_X, State::Dimension + BIAS_ACCEL_X) = (q.w()*q.w()+q.x()*q.x()-q.y()*q.y()-q.z()*q.z());
      A(State::VELOCITY_X, State::Dimension + BIAS_ACCEL_Y) = (2.0*q.x()*q.y()-2.0*q.w()*q.z());
      A(State::VELOCITY_X, State::Dimension + BIAS_ACCEL_Z) = (2.0*q.x()*q.z()+2.0*q.w()*q.y());

      A(State::VELOCITY_Y, State::Dimension + BIAS_ACCEL_X) = (2.0*q.x()*q.y()+2.0*q.w()*q.z());
      A(State::VELOCITY_Y, State::Dimension + BIAS_ACCEL_Y) = (q.w()*q.w()-q.x()*q.x()+q.y()*q.y()-q.z()*q.z());
      A(State::VELOCITY_Y, State::Dimension + BIAS_ACCEL_Z) = (2.0*q.y()*q.z()-2.0*q.w()*q.x());

    } else {
      A(State::VELOCITY_X, State::Dimension + BIAS_ACCEL_X) = 0.0;
      A(State::VELOCITY_X, State::Dimension + BIAS_ACCEL_Y) = 0.0;
      A(State::VELOCITY_X, State::Dimension + BIAS_ACCEL_Z) = 0.0;

      A(State::VELOCITY_Y, State::Dimension + BIAS_ACCEL_X) = 0.0;
      A(State::VELOCITY_Y, State::Dimension + BIAS_ACCEL_Y) = 0.0;
      A(State::VELOCITY_Y, State::Dimension + BIAS_ACCEL_Z) = 0.0;
    }

    if (state.getSystemStatus() & STATE_Z_VELOCITY) {
      A(State::VELOCITY_Z, State::Dimension + BIAS_ACCEL_X) = ( 2.0*q.x()*q.z()-2.0*q.w()*q.y());
      A(State::VELOCITY_Z, State::Dimension + BIAS_ACCEL_Y) = ( 2.0*q.y()*q.z()+2.0*q.w()*q.x());
      A(State::VELOCITY_Z, State::Dimension + BIAS_ACCEL_Z) = (q.w()*q.w()-q.x()*q.x()-q.y()*q.y()+q.z()*q.z());

    } else {
      A(State::VELOCITY_Z, State::Dimension + BIAS_ACCEL_X) = 0.0;
      A(State::VELOCITY_Z, State::Dimension + BIAS_ACCEL_Y) = 0.0;
      A(State::VELOCITY_Z, State::Dimension + BIAS_ACCEL_Z) = 0.0;
    }
  }
}

} // namespace hector_pose_estimation

