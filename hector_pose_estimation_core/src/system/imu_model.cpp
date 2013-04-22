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
#include <hector_pose_estimation/filter/set_filter.h>

namespace hector_pose_estimation {

template class System_<GyroModel>;
template class System_<AccelerometerModel>;

GyroModel::GyroModel()
{
  rate_stddev_ = 1.0 * M_PI/180.0;
  rate_drift_ = 1.0e-1 * M_PI/180.0;
  parameters().add("stddev", rate_stddev_);
  parameters().add("drift", rate_drift_);
}

GyroModel::~GyroModel()
{}

bool GyroModel::init(PoseEstimation& estimator, State& state)
{
  drift_ = state.addSubState<3>(this, "gyro");
  return drift_;
}

void GyroModel::getSystemNoise(NoiseVariance& Q, const State& state, bool init)
{
  if (init) {
    Q(BIAS_GYRO_X,BIAS_GYRO_X) = Q(BIAS_GYRO_Y,BIAS_GYRO_Y) = Q(BIAS_GYRO_Z,BIAS_GYRO_Z) = pow(rate_drift_, 2);
  }
}

void GyroModel::getStateJacobian(SystemMatrix& A1, CrossSystemMatrix& A01, const State& state, bool)
{
  if (state.getRateIndex() >= 0) return;

  State::ConstOrientationType q(state.getOrientation());
  State::ConstVelocityType v(state.getVelocity());

  if (state.getOrientationIndex() >= 0) {
    A01(State::QUATERNION_W, BIAS_GYRO_X)  = -0.5*q.x();
    A01(State::QUATERNION_W, BIAS_GYRO_Y)  = -0.5*q.y();
    A01(State::QUATERNION_W, BIAS_GYRO_Z)  = -0.5*q.z();

    A01(State::QUATERNION_X, BIAS_GYRO_X)  =  0.5*q.w();
    A01(State::QUATERNION_X, BIAS_GYRO_Y)  = -0.5*q.z();
    A01(State::QUATERNION_X, BIAS_GYRO_Z)  = 0.5*q.y();

    A01(State::QUATERNION_Y, BIAS_GYRO_X)  = 0.5*q.z();
    A01(State::QUATERNION_Y, BIAS_GYRO_Y)  = 0.5*q.w();
    A01(State::QUATERNION_Y, BIAS_GYRO_Z)  = -0.5*q.x();

    A01(State::QUATERNION_Z, BIAS_GYRO_X)  = -0.5*q.y();
    A01(State::QUATERNION_Z, BIAS_GYRO_Y)  = 0.5*q.x();
    A01(State::QUATERNION_Z, BIAS_GYRO_Z)  = 0.5*q.w();
  }

#ifdef VELOCITY_IN_BODY_FRAME
  if (state.getVelocityIndex() >= 0 && state.getSystemStatus() & STATE_VELOCITY_XY) {
    A01(State::VELOCITY_X, BIAS_GYRO_X) =  0.0;
    A01(State::VELOCITY_X, BIAS_GYRO_Y) = -v.z();
    A01(State::VELOCITY_X, BIAS_GYRO_Z) =  v.y();

    A01(State::VELOCITY_Y, BIAS_GYRO_X) =  v.z();
    A01(State::VELOCITY_Y, BIAS_GYRO_Y) =  0.0;
    A01(State::VELOCITY_Y, BIAS_GYRO_Z) = -v.x();
  }

  if (state.getVelocityIndex() >= 0 && state.getSystemStatus() & STATE_VELOCITY_Z) {
    A01(State::VELOCITY_Z, BIAS_GYRO_X) = -v.y();
    A01(State::VELOCITY_Z, BIAS_GYRO_Y) =  v.x();
    A01(State::VELOCITY_Z, BIAS_GYRO_Z) =  0.0;
  }

#endif
}

AccelerometerModel::AccelerometerModel()
{
  acceleration_stddev_ = 1.0e-2;
  acceleration_drift_ = 1.0e-3;
  parameters().add("stddev", acceleration_stddev_);
  parameters().add("drift", acceleration_drift_);
}

AccelerometerModel::~AccelerometerModel()
{}

bool AccelerometerModel::init(PoseEstimation& estimator, State& state)
{
  drift_ = state.addSubState<3>(this, "accelerometer");
  return drift_;
}

void AccelerometerModel::getSystemNoise(NoiseVariance& Q, const State& state, bool init)
{
  if (init) {
    Q(BIAS_ACCEL_X,BIAS_ACCEL_X) = Q(BIAS_ACCEL_Y,BIAS_ACCEL_Y) = pow(acceleration_drift_, 2);
    Q(BIAS_ACCEL_Z,BIAS_ACCEL_Z) = pow(acceleration_drift_, 2);
  }
}

void AccelerometerModel::getStateJacobian(SystemMatrix& A1, CrossSystemMatrix& A01, const State& state, bool)
{
  if (state.getAccelerationIndex() >= 0) return;

  A01 = 0;

#ifdef VELOCITY_IN_BODY_FRAME
  if (state.getVelocityIndex() >= 0) {
    if (state.getSystemStatus() & STATE_VELOCITY_XY) {
      A01.block(State::VELOCITY_X, BIAS_ACCEL_X, 2, 2).setIdentity();
    }

    if (state.getSystemStatus() & STATE_VELOCITY_Z) {
      A01.block(State::VELOCITY_Z, BIAS_ACCEL_Z, 1, 1).setIdentity();
    }
  }

#else
  State::ConstOrientationType q(state.getOrientation());
  if (state.getVelocityIndex() >= 0) {
    if (state.getSystemStatus() & STATE_VELOCITY_XY) {
      A01(State::VELOCITY_X, BIAS_ACCEL_X) = (q.w()*q.w()+q.x()*q.x()-q.y()*q.y()-q.z()*q.z());
      A01(State::VELOCITY_X, BIAS_ACCEL_Y) = (2.0*q.x()*q.y()-2.0*q.w()*q.z());
      A01(State::VELOCITY_X, BIAS_ACCEL_Z) = (2.0*q.x()*q.z()+2.0*q.w()*q.y());

      A01(State::VELOCITY_Y, BIAS_ACCEL_X) = (2.0*q.x()*q.y()+2.0*q.w()*q.z());
      A01(State::VELOCITY_Y, BIAS_ACCEL_Y) = (q.w()*q.w()-q.x()*q.x()+q.y()*q.y()-q.z()*q.z());
      A01(State::VELOCITY_Y, BIAS_ACCEL_Z) = (2.0*q.y()*q.z()-2.0*q.w()*q.x());
    }

    if (state.getSystemStatus() & STATE_VELOCITY_Z) {
      A01(State::VELOCITY_Z, BIAS_ACCEL_X) = ( 2.0*q.x()*q.z()-2.0*q.w()*q.y());
      A01(State::VELOCITY_Z, BIAS_ACCEL_Y) = ( 2.0*q.y()*q.z()+2.0*q.w()*q.x());
      A01(State::VELOCITY_Z, BIAS_ACCEL_Z) = (q.w()*q.w()-q.x()*q.x()-q.y()*q.y()+q.z()*q.z());
    }
  }
#endif

}

} // namespace hector_pose_estimation
