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

#include <hector_pose_estimation/system/generic_quaternion_system_model.h>
#include <hector_pose_estimation/pose_estimation.h>

namespace hector_pose_estimation {

static const double GRAVITY = -9.8065;

GenericQuaternionSystemModel::GenericQuaternionSystemModel()
{
  gravity_ = GRAVITY;
#ifdef USE_RATE_SYSTEM_MODEL
  rate_stddev_ = 0.0;
  angular_acceleration_stddev_ = 10.0 * M_PI/180.0;
#else // USE_RATE_SYSTEM_MODEL
  rate_stddev_ = 1.0 * M_PI/180.0;
#endif // USE_RATE_SYSTEM_MODEL
  acceleration_stddev_ = 1.0e-2;
  velocity_stddev_ = 0.0;
//  accelerationdrift_ = 1.0e-6;
//  rate_drift_ = 1.0e-2 * M_PI/180.0;
  parameters().add("gravity", gravity_);
  parameters().add("ratestddev", rate_stddev_);
#ifdef USE_RATE_SYSTEM_MODEL
  parameters().add("angular_accelerationstddev", angular_acceleration_stddev_);
#endif // USE_RATE_SYSTEM_MODEL
  parameters().add("accelerationstddev", acceleration_stddev_);
  parameters().add("velocity_stddev", velocity_stddev_);
//  parameters().add("accelerationdrift", accelerationdrift_);
//  parameters().add("ratedrift", rate_drift_);
}

GenericQuaternionSystemModel::~GenericQuaternionSystemModel()
{
}

bool GenericQuaternionSystemModel::init(PoseEstimation& estimator, State& state)
{
  imu_ = estimator.registerInput<InputType>("raw_imu");
  return true;
}

bool GenericQuaternionSystemModel::prepareUpdate(State& state, double dt)
{
  if (state.getAccelerationIndex() >= 0)
    acceleration = state.getAcceleration();
  else
    acceleration = imu_->getAcceleration() + accelerometer_model_->getBias();

  if (state.getRateIndex() >= 0)
    rate = state.getRate();
  else
    rate = imu_->getRate() + gyro_model_->getBias();

  return true;
}

void GenericQuaternionSystemModel::getDerivative(StateVector& x_dot, const State& state)
{
  const State::OrientationType& q = state.getOrientation();
  const State::VelocityType& v = state.getVelocity();
  // const State::PositionType& p = state.getPosition();

  if (state.getOrientationIndex() >= 0) {
    x_dot(State::QUATERNION_W) = 0.5*(                  (-rate.x())*q.x()+(-rate.y())*q.y()+(-rate.z())*q.z());
    x_dot(State::QUATERNION_X) = 0.5*(( rate.x())*q.w()                  +( rate.z())*q.y()+(-rate.y())*q.z());
    x_dot(State::QUATERNION_Y) = 0.5*(( rate.y())*q.w()+(-rate.z())*q.x()                  +( rate.x())*q.z());
    x_dot(State::QUATERNION_Z) = 0.5*(( rate.z())*q.w()+( rate.y())*q.x()+(-rate.x())*q.y()                  );
  }

  if (state.getSystemStatus() & STATE_XY_VELOCITY && state.getVelocityIndex() >= 0) {
    x_dot(State::VELOCITY_X)  = ((q.w()*q.w()+q.x()*q.x()-q.y()*q.y()-q.z()*q.z())*acceleration.x() + (2.0*q.x()*q.y()-2.0*q.w()*q.z())                *acceleration.y() + (2.0*q.x()*q.z()+2.0*q.w()*q.y())                *acceleration.z());
    x_dot(State::VELOCITY_Y)  = ((2.0*q.x()*q.y()+2.0*q.w()*q.z())                *acceleration.x() + (q.w()*q.w()-q.x()*q.x()+q.y()*q.y()-q.z()*q.z())*acceleration.y() + (2.0*q.y()*q.z()-2.0*q.w()*q.x())                *acceleration.z());
  }
  if (state.getSystemStatus() & STATE_Z_VELOCITY && state.getVelocityIndex() >= 0) {
    x_dot(State::VELOCITY_Z)  = ((2.0*q.x()*q.z()-2.0*q.w()*q.y())                *acceleration.x() + (2.0*q.y()*q.z()+2.0*q.w()*q.x())                *acceleration.y() + (q.w()*q.w()-q.x()*q.x()-q.y()*q.y()+q.z()*q.z())*acceleration.z() + gravity_);
  }

  if (state.getSystemStatus() & STATE_XY_POSITION) {
    x_dot(State::POSITION_X)  = (v.x());
    x_dot(State::POSITION_Y)  = (v.y());
  }
  if (state.getSystemStatus() & STATE_Z_POSITION) {
    x_dot(State::POSITION_Z)  = (v.z());
  }
}

void GenericQuaternionSystemModel::getSystemNoise(NoiseVariance& Q, const State& state, bool init)
{
  if (init) {
    if (state.getRateIndex() >= 0)
      Q(State::RATE_X,State::RATE_X) = Q(State::RATE_Y,State::RATE_Y) = Q(State::RATE_Z,State::RATE_Z) = pow(angular_acceleration_stddev_, 2);
    if (state.getPositionIndex() >= 0)
      Q(State::POSITION_X,State::POSITION_X) = Q(State::POSITION_Y,State::POSITION_Y) = Q(State::POSITION_Z,State::POSITION_Z) = pow(velocity_stddev_, 2);
    if (state.getRateIndex() >= 0)
      Q(State::VELOCITY_X,State::VELOCITY_X) = Q(State::VELOCITY_Y,State::VELOCITY_Y) = Q(State::VELOCITY_Z,State::VELOCITY_Z) = pow(acceleration_stddev_, 2);
//    Q(BIAS_ACCEL_X,BIAS_ACCEL_X) = Q(BIAS_ACCEL_Y,BIAS_ACCEL_Y) = pow(acceleration_drift_, 2);
//    Q(BIAS_ACCEL_Z,BIAS_ACCEL_Z) = pow(acceleration_drift_, 2);
//    Q(BIAS_GYRO_X,BIAS_GYRO_X) = Q(BIAS_GYRO_Y,BIAS_GYRO_Y) = Q(BIAS_GYRO_Z,BIAS_GYRO_Z) = pow(ratedrift_, 2);
  }

  if (state.getOrientationIndex() >= 0) {
    const State::OrientationType& q = state.getOrientation();
    double rate_variance_4 = 0.25 * pow(rate_stddev_, 2);
    Q(State::QUATERNION_W,State::QUATERNION_W) = rate_variance_4 * (q.x()*q.x()+q.y()*q.y()+q.z()*q.z());
    Q(State::QUATERNION_X,State::QUATERNION_X) = rate_variance_4 * (q.w()*q.w()+q.y()*q.y()+q.z()*q.z());
    Q(State::QUATERNION_Y,State::QUATERNION_Y) = rate_variance_4 * (q.w()*q.w()+q.x()*q.x()+q.z()*q.z());
    Q(State::QUATERNION_Z,State::QUATERNION_Z) = rate_variance_4 * (q.w()*q.w()+q.x()*q.x()+q.y()*q.y());
  }
}

void GenericQuaternionSystemModel::getStateJacobian(SystemMatrix& A, const State& state, bool)
{
  const State::OrientationType& q = state.getOrientation();

  //--> Set A-Matrix
  //----------------------------------------------------------
  if (state.getOrientationIndex() >= 0) {
    A(State::QUATERNION_W,State::QUATERNION_X) = (-0.5*rate.x());
    A(State::QUATERNION_W,State::QUATERNION_Y) = (-0.5*rate.y());
    A(State::QUATERNION_W,State::QUATERNION_Z) = (-0.5*rate.z());
    A(State::QUATERNION_X,State::QUATERNION_W) = ( 0.5*rate.x());
    A(State::QUATERNION_X,State::QUATERNION_Y) = ( 0.5*rate.z());
    A(State::QUATERNION_X,State::QUATERNION_Z) = (-0.5*rate.y());
    A(State::QUATERNION_Y,State::QUATERNION_W) = ( 0.5*rate.y());
    A(State::QUATERNION_Y,State::QUATERNION_X) = (-0.5*rate.z());
    A(State::QUATERNION_Y,State::QUATERNION_Z) = ( 0.5*rate.x());
    A(State::QUATERNION_Z,State::QUATERNION_W) = ( 0.5*rate.z());
    A(State::QUATERNION_Z,State::QUATERNION_X) = ( 0.5*rate.y());
    A(State::QUATERNION_Z,State::QUATERNION_Y) = (-0.5*rate.x());

    if (state.getRateIndex() >= 0) {
      A(State::QUATERNION_W,State::RATE_X)  = -0.5*q.x();
      A(State::QUATERNION_W,State::RATE_Y)  = -0.5*q.y();
      A(State::QUATERNION_W,State::RATE_Z)  = -0.5*q.z();

      A(State::QUATERNION_X,State::RATE_X)  =  0.5*q.w();
      A(State::QUATERNION_X,State::RATE_Y)  = -0.5*q.z();
      A(State::QUATERNION_X,State::RATE_Z)  = 0.5*q.y();

      A(State::QUATERNION_Y,State::RATE_X)  = 0.5*q.z();
      A(State::QUATERNION_Y,State::RATE_Y)  = 0.5*q.w();
      A(State::QUATERNION_Y,State::RATE_Z)  = -0.5*q.x();

      A(State::QUATERNION_Z,State::RATE_X)  = -0.5*q.y();
      A(State::QUATERNION_Z,State::RATE_Y)  = 0.5*q.x();
      A(State::QUATERNION_Z,State::RATE_Z)  = 0.5*q.w();
    }
  }

  if (state.getVelocityIndex() >= 0 && state.getOrientationIndex() >= 0) {
    if (state.getSystemStatus() & STATE_XY_VELOCITY) {
      A(State::VELOCITY_X,State::QUATERNION_W) = (-2.0*q.z()*acceleration.y()+2.0*q.y()*acceleration.z()+2.0*q.w()*acceleration.x());
      A(State::VELOCITY_X,State::QUATERNION_X) = ( 2.0*q.y()*acceleration.y()+2.0*q.z()*acceleration.z()+2.0*q.x()*acceleration.x());
      A(State::VELOCITY_X,State::QUATERNION_Y) = (-2.0*q.y()*acceleration.x()+2.0*q.x()*acceleration.y()+2.0*q.w()*acceleration.z());
      A(State::VELOCITY_X,State::QUATERNION_Z) = (-2.0*q.z()*acceleration.x()-2.0*q.w()*acceleration.y()+2.0*q.x()*acceleration.z());
//      A(State::VELOCITY_X,BIAS_ACCEL_X) = (q.w()*q.w()+q.x()*q.x()-q.y()*q.y()-q.z()*q.z());
//      A(State::VELOCITY_X,BIAS_ACCEL_Y) = (2.0*q.x()*q.y()-2.0*q.w()*q.z());
//      A(State::VELOCITY_X,BIAS_ACCEL_Z) = (2.0*q.x()*q.z()+2.0*q.w()*q.y());

      A(State::VELOCITY_Y,State::QUATERNION_W) = (2.0*q.z()*acceleration.x()-2.0*q.x()*acceleration.z()+2.0*q.w()*acceleration.y());
      A(State::VELOCITY_Y,State::QUATERNION_X) = (2.0*q.y()*acceleration.x()-2.0*q.x()*acceleration.y()-2.0*q.w()*acceleration.z());
      A(State::VELOCITY_Y,State::QUATERNION_Y) = (2.0*q.x()*acceleration.x()+2.0*q.z()*acceleration.z()+2.0*q.y()*acceleration.y());
      A(State::VELOCITY_Y,State::QUATERNION_Z) = (2.0*q.w()*acceleration.x()-2.0*q.z()*acceleration.y()+2.0*q.y()*acceleration.z());
//      A(State::VELOCITY_Y,BIAS_ACCEL_X) = (2.0*q.x()*q.y()+2.0*q.w()*q.z());
//      A(State::VELOCITY_Y,BIAS_ACCEL_Y) = (q.w()*q.w()-q.x()*q.x()+q.y()*q.y()-q.z()*q.z());
//      A(State::VELOCITY_Y,BIAS_ACCEL_Z) = (2.0*q.y()*q.z()-2.0*q.w()*q.x());

    } else {
      A(State::VELOCITY_X,State::QUATERNION_W) = 0.0;
      A(State::VELOCITY_X,State::QUATERNION_X) = 0.0;
      A(State::VELOCITY_X,State::QUATERNION_Y) = 0.0;
      A(State::VELOCITY_X,State::QUATERNION_Z) = 0.0;
//      A(State::VELOCITY_X,BIAS_ACCEL_X) = 0.0;
//      A(State::VELOCITY_X,BIAS_ACCEL_Y) = 0.0;
//      A(State::VELOCITY_X,BIAS_ACCEL_Z) = 0.0;

      A(State::VELOCITY_Y,State::QUATERNION_W) = 0.0;
      A(State::VELOCITY_Y,State::QUATERNION_X) = 0.0;
      A(State::VELOCITY_Y,State::QUATERNION_Y) = 0.0;
      A(State::VELOCITY_Y,State::QUATERNION_Z) = 0.0;
//      A(State::VELOCITY_Y,BIAS_ACCEL_X) = 0.0;
//      A(State::VELOCITY_Y,BIAS_ACCEL_Y) = 0.0;
//      A(State::VELOCITY_Y,BIAS_ACCEL_Z) = 0.0;
    }

    if (state.getSystemStatus() & STATE_Z_VELOCITY) {
      A(State::VELOCITY_Z,State::QUATERNION_W) = (-2.0*q.y()*acceleration.x()+2.0*q.x()*acceleration.y()+2.0*q.w()*acceleration.z());
      A(State::VELOCITY_Z,State::QUATERNION_X) = ( 2.0*q.z()*acceleration.x()+2.0*q.w()*acceleration.y()-2.0*q.x()*acceleration.z());
      A(State::VELOCITY_Z,State::QUATERNION_Y) = (-2.0*q.w()*acceleration.x()+2.0*q.z()*acceleration.y()-2.0*q.y()*acceleration.z());
      A(State::VELOCITY_Z,State::QUATERNION_Z) = ( 2.0*q.x()*acceleration.x()+2.0*q.y()*acceleration.y()+2.0*q.z()*acceleration.z());
//      A(State::VELOCITY_Z,BIAS_ACCEL_X) = ( 2.0*q.x()*q.z()-2.0*q.w()*q.y());
//      A(State::VELOCITY_Z,BIAS_ACCEL_Y) = ( 2.0*q.y()*q.z()+2.0*q.w()*q.x());
//      A(State::VELOCITY_Z,BIAS_ACCEL_Z) = (q.w()*q.w()-q.x()*q.x()-q.y()*q.y()+q.z()*q.z());

    } else {
      A(State::VELOCITY_Z,State::QUATERNION_W) = 0.0;
      A(State::VELOCITY_Z,State::QUATERNION_X) = 0.0;
      A(State::VELOCITY_Z,State::QUATERNION_Y) = 0.0;
      A(State::VELOCITY_Z,State::QUATERNION_Z) = 0.0;
//      A(State::VELOCITY_Z,BIAS_ACCEL_X) = 0.0;
//      A(State::VELOCITY_Z,BIAS_ACCEL_Y) = 0.0;
//      A(State::VELOCITY_Z,BIAS_ACCEL_Z) = 0.0;
    }
  }

  if (state.getPositionIndex() >= 0 && state.getVelocityIndex() >= 0) {
    if (state.getSystemStatus() & STATE_XY_POSITION) {
      A(State::POSITION_X,State::VELOCITY_X)   = 1.0;
      A(State::POSITION_Y,State::VELOCITY_Y)   = 1.0;
    } else {
      A(State::POSITION_X,State::VELOCITY_X)   = 0.0;
      A(State::POSITION_Y,State::VELOCITY_Y)   = 0.0;
    }

    if (state.getSystemStatus() & STATE_Z_POSITION) {
      A(State::POSITION_Z,State::VELOCITY_Z)   = 1.0;
    } else {
      A(State::POSITION_Z,State::VELOCITY_Z)   = 0.0;
    }
  }
}

void GenericQuaternionSystemModel::getInputJacobian(InputMatrix& B, const State& state, bool init)
{
  throw std::runtime_error("not implemented");
}

void GenericQuaternionSystemModel::afterUpdate(State& state)
{
  SystemStatus flags = state.getMeasurementStatus();
//     flags |= STATE_XY_POSITION | STATE_Z_POSITION;
  if (flags & STATE_XY_POSITION) flags |= STATE_XY_VELOCITY;
  if (flags & STATE_Z_POSITION)  flags |= STATE_Z_VELOCITY;
  if (flags & STATE_XY_VELOCITY) flags |= STATE_ROLLPITCH;
  state.updateSystemStatus(flags, 0);
}

} // namespace hector_pose_estimation
