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
#include <hector_pose_estimation/filter/set_filter.h>

namespace hector_pose_estimation {

template class System_<GenericQuaternionSystemModel>;

GenericQuaternionSystemModel::GenericQuaternionSystemModel()
{
  angular_acceleration_stddev_ = 360.0 * M_PI/180.0;
  // rate_stddev_ = 1.0 * M_PI/180.0;
  // acceleration_stddev_ = 1.0e-2;
  velocity_stddev_ = 0.0;
//  acceleration_drift_ = 1.0e-6;
//  rate_drift_ = 1.0e-2 * M_PI/180.0;
  parameters().addAlias("gravity", gravity_);
  parameters().add("angular_acceleration_stddev", angular_acceleration_stddev_);
  parameters().addAlias("rate_stddev", rate_stddev_);
  parameters().addAlias("acceleration_stddev", acceleration_stddev_);
  parameters().add("velocity_stddev", velocity_stddev_);
}

GenericQuaternionSystemModel::~GenericQuaternionSystemModel()
{
}

bool GenericQuaternionSystemModel::init(PoseEstimation& estimator, State& state)
{
  gyro_ = System::create(new GyroModel, "gyro");
  accelerometer_ = System::create(new AccelerometerModel, "accelerometer");

  gravity_ = estimator.parameters().get("gravity_magnitude");
  rate_stddev_ = gyro_->parameters().get("stddev");
  acceleration_stddev_ = accelerometer_->parameters().get("stddev");

  imu_ = estimator.registerInput<InputType>("raw_imu");
  estimator.addSystem(gyro_, "gyro");
  estimator.addSystem(accelerometer_, "accelerometer");
  return true;
}

void GenericQuaternionSystemModel::getPrior(State &state) {
  if (state.getOrientationCovarianceIndex() >= 0) {
    state.P()(state.getOrientationCovarianceIndex() + W,state.getOrientationCovarianceIndex() + W) = 0.25 * 1.0;
    state.P()(state.getOrientationCovarianceIndex() + X,state.getOrientationCovarianceIndex() + X) = 0.25 * 1.0;
    state.P()(state.getOrientationCovarianceIndex() + Y,state.getOrientationCovarianceIndex() + Y) = 0.25 * 1.0;
    state.P()(state.getOrientationCovarianceIndex() + Z,state.getOrientationCovarianceIndex() + Z) = 0.25 * 1.0;
  }

  if (state.getRateCovarianceIndex() >= 0) {
    state.P()(state.getRateCovarianceIndex() + X,state.getRateCovarianceIndex() + X) = pow(0.0 * M_PI/180.0, 2);
    state.P()(state.getRateCovarianceIndex() + Y,state.getRateCovarianceIndex() + Y) = pow(0.0 * M_PI/180.0, 2);
    state.P()(state.getRateCovarianceIndex() + Z,state.getRateCovarianceIndex() + Z) = pow(0.0 * M_PI/180.0, 2);
  }

  if (state.getPositionCovarianceIndex() >= 0) {
    state.P()(state.getPositionCovarianceIndex() + X,state.getPositionCovarianceIndex() + X) = 0.0;
    state.P()(state.getPositionCovarianceIndex() + Y,state.getPositionCovarianceIndex() + Y) = 0.0;
    state.P()(state.getPositionCovarianceIndex() + Z,state.getPositionCovarianceIndex() + Z) = 0.0;
  }

  if (state.getVelocityCovarianceIndex() >= 0) {
    state.P()(state.getVelocityCovarianceIndex() + X,state.getVelocityCovarianceIndex() + X) = 0.0;
    state.P()(state.getVelocityCovarianceIndex() + Y,state.getVelocityCovarianceIndex() + Y) = 0.0;
    state.P()(state.getVelocityCovarianceIndex() + Z,state.getVelocityCovarianceIndex() + Z) = 0.0;
  }
}

bool GenericQuaternionSystemModel::prepareUpdate(State& state, double dt)
{
  if (state.getAccelerationVectorIndex() >= 0)
    acceleration = state.getAcceleration();
  else
    acceleration = imu_->getAcceleration() + accelerometer_->getModel()->getBias();

  if (state.getRateVectorIndex() >= 0)
    rate = state.getRate();
  else
    rate = imu_->getRate() + gyro_->getModel()->getBias();

  state.getRotationMatrix(R);
  return true;
}

void GenericQuaternionSystemModel::getDerivative(StateVector& x_dot, const State& state)
{
  State::ConstOrientationType q(state.getOrientation());
  State::ConstVelocityType v(state.getVelocity());
  // State::ConstPositionType p(state.getPosition());

  x_dot = 0.0;

  if (state.getOrientationVectorIndex() >= 0) {
    x_dot(state.getOrientationVectorIndex() + W) = 0.5*(                  (-rate.x())*q.x()+(-rate.y())*q.y()+(-rate.z())*q.z());
    x_dot(state.getOrientationVectorIndex() + X) = 0.5*(( rate.x())*q.w()                  +( rate.z())*q.y()+(-rate.y())*q.z());
    x_dot(state.getOrientationVectorIndex() + Y) = 0.5*(( rate.y())*q.w()+(-rate.z())*q.x()                  +( rate.x())*q.z());
    x_dot(state.getOrientationVectorIndex() + Z) = 0.5*(( rate.z())*q.w()+( rate.y())*q.x()+(-rate.x())*q.y()                  );
  }

#ifdef VELOCITY_IN_BODY_FRAME
  ColumnVector3 wxv = rate.cross(v);

  if (state.getSystemStatus() & STATE_VELOCITY_XY && state.getVelocityVectorIndex() >= 0) {
    x_dot(state.getVelocityVectorIndex() + X)  = acceleration.x() - wxv.x() + R(2,0) * gravity_;
    x_dot(state.getVelocityVectorIndex() + Y)  = acceleration.y() - wxv.y() + R(2,1) * gravity_;
  }
  if (state.getSystemStatus() & STATE_VELOCITY_Z && state.getVelocityVectorIndex() >= 0) {
    x_dot(state.getVelocityVectorIndex() + Z)  = acceleration.z() - wxv.z() + R(2,2) * gravity_;
  }

  if (state.getSystemStatus() & STATE_POSITION_XY && state.getPositionVectorIndex() >= 0) {
    x_dot(state.getPositionVectorIndex() + X)  = R(0,0)*v.x() + R(0,1)*v.y() + R(0,2)*v.z();
    x_dot(state.getPositionVectorIndex() + Y)  = R(1,0)*v.x() + R(1,1)*v.y() + R(1,2)*v.z();
  }
  if (state.getSystemStatus() & STATE_POSITION_Z && state.getPositionVectorIndex() >= 0) {
    x_dot(state.getPositionVectorIndex() + Z)  = R(2,0)*v.x() + R(2,1)*v.y() + R(2,2)*v.z();
  }

#else
  if (state.getSystemStatus() & STATE_VELOCITY_XY && state.getVelocityVectorIndex() >= 0) {
    x_dot(state.getVelocityVectorIndex() + X)  = R(0,0)*acceleration.x() + R(0,1)*acceleration.y() + R(0,2)*acceleration.z();
    x_dot(state.getVelocityVectorIndex() + Y)  = R(1,0)*acceleration.x() + R(1,1)*acceleration.y() + R(1,2)*acceleration.z();
  }
  if (state.getSystemStatus() & STATE_VELOCITY_Z && state.getVelocityVectorIndex() >= 0) {
    x_dot(state.getVelocityVectorIndex() + Z)  = R(2,0)*acceleration.x() + R(2,1)*acceleration.y() + R(2,2)*acceleration.z() + gravity_;
  }

  if (state.getSystemStatus() & STATE_POSITION_XY && state.getPositionVectorIndex() >= 0) {
    x_dot(state.getPositionVectorIndex() + X)  = v.x();
    x_dot(state.getPositionVectorIndex() + Y)  = v.y();
  }
  if (state.getSystemStatus() & STATE_POSITION_Z && state.getPositionVectorIndex() >= 0) {
    x_dot(state.getPositionVectorIndex() + Z)  = v.z();
  }
#endif // VELOCITY_IN_BODY_FRAME
}

void GenericQuaternionSystemModel::getSystemNoise(NoiseVariance& Q, const State& state, bool init)
{
  if (init) {
    if (state.getRateCovarianceIndex() >= 0)
      Q(state.getRateCovarianceIndex() + X,state.getRateCovarianceIndex() + X) = Q(state.getRateCovarianceIndex() + Y,state.getRateCovarianceIndex() + Y) = Q(state.getRateCovarianceIndex() + Z,state.getRateCovarianceIndex() + Z) = pow(angular_acceleration_stddev_, 2);
    if (state.getPositionCovarianceIndex() >= 0)
      Q(state.getPositionCovarianceIndex() + X,state.getPositionCovarianceIndex() + X) = Q(state.getPositionCovarianceIndex() + Y,state.getPositionCovarianceIndex() + Y) = Q(state.getPositionCovarianceIndex() + Z,state.getPositionCovarianceIndex() + Z) = pow(velocity_stddev_, 2);
    if (state.getVelocityCovarianceIndex() >= 0)
      Q(state.getVelocityCovarianceIndex() + X,state.getVelocityCovarianceIndex() + X) = Q(state.getVelocityCovarianceIndex() + Y,state.getVelocityCovarianceIndex() + Y) = Q(state.getVelocityCovarianceIndex() + Z,state.getVelocityCovarianceIndex() + Z) = pow(acceleration_stddev_, 2);
//    Q(BIAS_ACCEL_X,BIAS_ACCEL_X) = Q(BIAS_ACCEL_Y,BIAS_ACCEL_Y) = pow(acceleration_drift_, 2);
//    Q(BIAS_ACCEL_Z,BIAS_ACCEL_Z) = pow(acceleration_drift_, 2);
//    Q(BIAS_GYRO_X,BIAS_GYRO_X) = Q(BIAS_GYRO_Y,BIAS_GYRO_Y) = Q(BIAS_GYRO_Z,BIAS_GYRO_Z) = pow(rate_drift_, 2);
  }

  if ((double) rate_stddev_ > 0.0 && state.getOrientationCovarianceIndex() >= 0) {
    State::ConstOrientationType q(state.getOrientation());
    double rate_variance_4 = 0.25 * pow(rate_stddev_, 2);
    Q(state.getOrientationCovarianceIndex() + W,state.getOrientationCovarianceIndex() + W) = rate_variance_4 * (q.x()*q.x()+q.y()*q.y()+q.z()*q.z());
    Q(state.getOrientationCovarianceIndex() + X,state.getOrientationCovarianceIndex() + X) = rate_variance_4 * (q.w()*q.w()+q.y()*q.y()+q.z()*q.z());
    Q(state.getOrientationCovarianceIndex() + Y,state.getOrientationCovarianceIndex() + Y) = rate_variance_4 * (q.w()*q.w()+q.x()*q.x()+q.z()*q.z());
    Q(state.getOrientationCovarianceIndex() + Z,state.getOrientationCovarianceIndex() + Z) = rate_variance_4 * (q.w()*q.w()+q.x()*q.x()+q.y()*q.y());
  }
}

void GenericQuaternionSystemModel::getStateJacobian(SystemMatrix& A, const State& state, bool)
{
  State::ConstOrientationType q(state.getOrientation());
  State::ConstVelocityType v(state.getVelocity());

  A = 0.0;

  //--> Set A-Matrix
  //----------------------------------------------------------
  if (state.getOrientationCovarianceIndex() >= 0) {
    A(state.getOrientationCovarianceIndex() + W,state.getOrientationCovarianceIndex() + X) = (-0.5*rate.x());
    A(state.getOrientationCovarianceIndex() + W,state.getOrientationCovarianceIndex() + Y) = (-0.5*rate.y());
    A(state.getOrientationCovarianceIndex() + W,state.getOrientationCovarianceIndex() + Z) = (-0.5*rate.z());
    A(state.getOrientationCovarianceIndex() + X,state.getOrientationCovarianceIndex() + W) = ( 0.5*rate.x());
    A(state.getOrientationCovarianceIndex() + X,state.getOrientationCovarianceIndex() + Y) = ( 0.5*rate.z());
    A(state.getOrientationCovarianceIndex() + X,state.getOrientationCovarianceIndex() + Z) = (-0.5*rate.y());
    A(state.getOrientationCovarianceIndex() + Y,state.getOrientationCovarianceIndex() + W) = ( 0.5*rate.y());
    A(state.getOrientationCovarianceIndex() + Y,state.getOrientationCovarianceIndex() + X) = (-0.5*rate.z());
    A(state.getOrientationCovarianceIndex() + Y,state.getOrientationCovarianceIndex() + Z) = ( 0.5*rate.x());
    A(state.getOrientationCovarianceIndex() + Z,state.getOrientationCovarianceIndex() + W) = ( 0.5*rate.z());
    A(state.getOrientationCovarianceIndex() + Z,state.getOrientationCovarianceIndex() + X) = ( 0.5*rate.y());
    A(state.getOrientationCovarianceIndex() + Z,state.getOrientationCovarianceIndex() + Y) = (-0.5*rate.x());

    if (state.getRateCovarianceIndex() >= 0) {
      A(state.getOrientationCovarianceIndex() + W,state.getRateCovarianceIndex() + X)  = -0.5*q.x();
      A(state.getOrientationCovarianceIndex() + W,state.getRateCovarianceIndex() + Y)  = -0.5*q.y();
      A(state.getOrientationCovarianceIndex() + W,state.getRateCovarianceIndex() + Z)  = -0.5*q.z();

      A(state.getOrientationCovarianceIndex() + X,state.getRateCovarianceIndex() + X)  =  0.5*q.w();
      A(state.getOrientationCovarianceIndex() + X,state.getRateCovarianceIndex() + Y)  = -0.5*q.z();
      A(state.getOrientationCovarianceIndex() + X,state.getRateCovarianceIndex() + Z)  = 0.5*q.y();

      A(state.getOrientationCovarianceIndex() + Y,state.getRateCovarianceIndex() + X)  = 0.5*q.z();
      A(state.getOrientationCovarianceIndex() + Y,state.getRateCovarianceIndex() + Y)  = 0.5*q.w();
      A(state.getOrientationCovarianceIndex() + Y,state.getRateCovarianceIndex() + Z)  = -0.5*q.x();

      A(state.getOrientationCovarianceIndex() + Z,state.getRateCovarianceIndex() + X)  = -0.5*q.y();
      A(state.getOrientationCovarianceIndex() + Z,state.getRateCovarianceIndex() + Y)  = 0.5*q.x();
      A(state.getOrientationCovarianceIndex() + Z,state.getRateCovarianceIndex() + Z)  = 0.5*q.w();
    }
  }

#ifdef VELOCITY_IN_BODY_FRAME

//  if (state.getSystemStatus() & STATE_VELOCITY_XY && state.getVelocityIndex() >= 0) {
//    x_dot(state.getVelocityCovarianceIndex() + X)  = acceleration.x() - wxv.x() + (2.0*q.x()*q.z()-2.0*q.w()*q.y()) * gravity_;
//    x_dot(state.getVelocityCovarianceIndex() + Y)  = acceleration.y() - wxv.y() + (2.0*q.y()*q.z()+2.0*q.w()*q.x()) * gravity_;
//  }
//  if (state.getSystemStatus() & STATE_VELOCITY_Z && state.getVelocityIndex() >= 0) {
//    x_dot(state.getVelocityCovarianceIndex() + Z)  = acceleration.z() - wxv.z() + (q.w()*q.w()-q.x()*q.x()-q.y()*q.y()+q.z()*q.z()) * gravity_;
//  }

  if (state.getVelocityIndex() >= 0 && state.getSystemStatus() & STATE_VELOCITY_XY) {
    if (state.getRateCovarianceIndex() >= 0) {
      A(state.getVelocityCovarianceIndex() + X,state.getRateCovarianceIndex() + X) =  0.0;
      A(state.getVelocityCovarianceIndex() + X,state.getRateCovarianceIndex() + Y) = -v.z();
      A(state.getVelocityCovarianceIndex() + X,state.getRateCovarianceIndex() + Z) =  v.y();

      A(state.getVelocityCovarianceIndex() + Y,state.getRateCovarianceIndex() + X) =  v.z();
      A(state.getVelocityCovarianceIndex() + Y,state.getRateCovarianceIndex() + Y) =  0.0;
      A(state.getVelocityCovarianceIndex() + Y,state.getRateCovarianceIndex() + Z) = -v.x();
    }

    A(state.getVelocityCovarianceIndex() + X,state.getVelocityCovarianceIndex() + X) =  0.0;
    A(state.getVelocityCovarianceIndex() + X,state.getVelocityCovarianceIndex() + Y) =  rate.z();
    A(state.getVelocityCovarianceIndex() + X,state.getVelocityCovarianceIndex() + Z) = -rate.y();

    A(state.getVelocityCovarianceIndex() + Y,state.getVelocityCovarianceIndex() + X) = -rate.z();
    A(state.getVelocityCovarianceIndex() + Y,state.getVelocityCovarianceIndex() + Y) =  0.0;
    A(state.getVelocityCovarianceIndex() + Y,state.getVelocityCovarianceIndex() + Z) =  rate.x();

    if (state.getOrientationCovarianceIndex() >= 0) {
      A(state.getVelocityCovarianceIndex() + X,state.getOrientationCovarianceIndex() + W) = -2.0*q.y()*gravity_;
      A(state.getVelocityCovarianceIndex() + X,state.getOrientationCovarianceIndex() + X) =  2.0*q.z()*gravity_;
      A(state.getVelocityCovarianceIndex() + X,state.getOrientationCovarianceIndex() + Y) = -2.0*q.w()*gravity_;
      A(state.getVelocityCovarianceIndex() + X,state.getOrientationCovarianceIndex() + Z) =  2.0*q.x()*gravity_;

      A(state.getVelocityCovarianceIndex() + Y,state.getOrientationCovarianceIndex() + W) =  2.0*q.x()*gravity_;
      A(state.getVelocityCovarianceIndex() + Y,state.getOrientationCovarianceIndex() + X) =  2.0*q.w()*gravity_;
      A(state.getVelocityCovarianceIndex() + Y,state.getOrientationCovarianceIndex() + Y) =  2.0*q.z()*gravity_;
      A(state.getVelocityCovarianceIndex() + Y,state.getOrientationCovarianceIndex() + Z) =  2.0*q.y()*gravity_;
    }
  }

  if (state.getVelocityCovarianceIndex() >= 0 && state.getSystemStatus() & STATE_VELOCITY_Z) {
    if (state.getRateCovarianceIndex() >= 0) {
      A(state.getVelocityCovarianceIndex() + Z,state.getRateCovarianceIndex() + X) = -v.y();
      A(state.getVelocityCovarianceIndex() + Z,state.getRateCovarianceIndex() + Y) =  v.x();
      A(state.getVelocityCovarianceIndex() + Z,state.getRateCovarianceIndex() + Z) =  0.0;
    }

    A(state.getVelocityCovarianceIndex() + Z,state.getVelocityCovarianceIndex() + X) =  rate.y();
    A(state.getVelocityCovarianceIndex() + Z,state.getVelocityCovarianceIndex() + Y) = -rate.x();
    A(state.getVelocityCovarianceIndex() + Z,state.getVelocityCovarianceIndex() + Z) =  0.0;

    if (state.getOrientationCovarianceIndex() >= 0) {
      A(state.getVelocityCovarianceIndex() + Z,state.getOrientationCovarianceIndex() + W) =  2.0*q.w()*gravity_;
      A(state.getVelocityCovarianceIndex() + Z,state.getOrientationCovarianceIndex() + X) = -2.0*q.x()*gravity_;
      A(state.getVelocityCovarianceIndex() + Z,state.getOrientationCovarianceIndex() + Y) = -2.0*q.y()*gravity_;
      A(state.getVelocityCovarianceIndex() + Z,state.getOrientationCovarianceIndex() + Z) =  2.0*q.z()*gravity_;
    }
  }

  //  if (state.getSystemStatus() & STATE_POSITION_XY) {
  //    x_dot(state.getPositionCovarianceIndex() + X)  = (q.w()*q.w()+q.x()*q.x()-q.y()*q.y()-q.z()*q.z())*v.x() + (2.0*q.x()*q.y()-2.0*q.w()*q.z())                *v.y() + (2.0*q.x()*q.z()+2.0*q.w()*q.y())                *v.z();
  //    x_dot(state.getPositionCovarianceIndex() + Y)  = (2.0*q.x()*q.y()+2.0*q.w()*q.z())                *v.x() + (q.w()*q.w()-q.x()*q.x()+q.y()*q.y()-q.z()*q.z())*v.y() + (2.0*q.y()*q.z()-2.0*q.w()*q.x())                *v.z();
  //  }
  //  if (state.getSystemStatus() & STATE_POSITION_Z) {
  //    x_dot(state.getPositionCovarianceIndex() + Z)  = (2.0*q.x()*q.z()-2.0*q.w()*q.y())                *v.x() + (2.0*q.y()*q.z()+2.0*q.w()*q.x())                *v.y() + (q.w()*q.w()-q.x()*q.x()-q.y()*q.y()+q.z()*q.z())*v.z();
  //  }

  if (state.getPositionCovarianceIndex() >= 0 && state.getSystemStatus() & STATE_POSITION_XY) {
    if (state.getOrientationCovarianceIndex() >= 0) {
      A(state.getPositionCovarianceIndex() + X,state.getOrientationCovarianceIndex() + W) = -2.0*q.z()*v.y()+2.0*q.y()*v.z()+2.0*q.w()*v.x();
      A(state.getPositionCovarianceIndex() + X,state.getOrientationCovarianceIndex() + X) =  2.0*q.y()*v.y()+2.0*q.z()*v.z()+2.0*q.x()*v.x();
      A(state.getPositionCovarianceIndex() + X,state.getOrientationCovarianceIndex() + Y) = -2.0*q.y()*v.x()+2.0*q.x()*v.y()+2.0*q.w()*v.z();
      A(state.getPositionCovarianceIndex() + X,state.getOrientationCovarianceIndex() + Z) = -2.0*q.z()*v.x()-2.0*q.w()*v.y()+2.0*q.x()*v.z();

      A(state.getPositionCovarianceIndex() + Y,state.getOrientationCovarianceIndex() + W) =  2.0*q.z()*v.x()-2.0*q.x()*v.z()+2.0*q.w()*v.y();
      A(state.getPositionCovarianceIndex() + Y,state.getOrientationCovarianceIndex() + X) =  2.0*q.y()*v.x()-2.0*q.x()*v.y()-2.0*q.w()*v.z();
      A(state.getPositionCovarianceIndex() + Y,state.getOrientationCovarianceIndex() + Y) =  2.0*q.x()*v.x()+2.0*q.z()*v.z()+2.0*q.y()*v.y();
      A(state.getPositionCovarianceIndex() + Y,state.getOrientationCovarianceIndex() + Z) =  2.0*q.w()*v.x()-2.0*q.z()*v.y()+2.0*q.y()*v.z();
    }

    if (state.getVelocityCovarianceIndex() >= 0) {
      A(state.getPositionCovarianceIndex() + X,state.getVelocityCovarianceIndex() + X)   =  R(0,0);
      A(state.getPositionCovarianceIndex() + X,state.getVelocityCovarianceIndex() + Y)   =  R(0,1);
      A(state.getPositionCovarianceIndex() + X,state.getVelocityCovarianceIndex() + Z)   =  R(0,2);

      A(state.getPositionCovarianceIndex() + Y,state.getVelocityCovarianceIndex() + X)   =  R(1,0);
      A(state.getPositionCovarianceIndex() + Y,state.getVelocityCovarianceIndex() + Y)   =  R(1,1);
      A(state.getPositionCovarianceIndex() + Y,state.getVelocityCovarianceIndex() + Z)   =  R(1,2);
    }
  }

  if (state.getPositionCovarianceIndex() >= 0 && state.getSystemStatus() & STATE_POSITION_Z) {
    if (state.getOrientationCovarianceIndex() >= 0) {
      A(state.getPositionCovarianceIndex() + Z,state.getOrientationCovarianceIndex() + W) = -2.0*q.y()*v.x()+2.0*q.x()*v.y()+2.0*q.w()*v.z();
      A(state.getPositionCovarianceIndex() + Z,state.getOrientationCovarianceIndex() + X) =  2.0*q.z()*v.x()+2.0*q.w()*v.y()-2.0*q.x()*v.z();
      A(state.getPositionCovarianceIndex() + Z,state.getOrientationCovarianceIndex() + Y) = -2.0*q.w()*v.x()+2.0*q.z()*v.y()-2.0*q.y()*v.z();
      A(state.getPositionCovarianceIndex() + Z,state.getOrientationCovarianceIndex() + Z) =  2.0*q.x()*v.x()+2.0*q.y()*v.y()+2.0*q.z()*v.z();
    }

    if (state.getVelocityCovarianceIndex() >= 0) {
      A(state.getPositionCovarianceIndex() + Z,state.getVelocityCovarianceIndex() + X)   =  R(2,0);
      A(state.getPositionCovarianceIndex() + Z,state.getVelocityCovarianceIndex() + Y)   =  R(2,1);
      A(state.getPositionCovarianceIndex() + Z,state.getVelocityCovarianceIndex() + Z)   =  R(2,2);
    }
  }

#else

  if (state.getVelocityCovarianceIndex() >= 0 && state.getOrientationCovarianceIndex() >= 0) {
    if (state.getSystemStatus() & STATE_VELOCITY_XY) {
      A(state.getVelocityCovarianceIndex() + X,state.getOrientationCovarianceIndex() + W) = (-2.0*q.z()*acceleration.y()+2.0*q.y()*acceleration.z()+2.0*q.w()*acceleration.x());
      A(state.getVelocityCovarianceIndex() + X,state.getOrientationCovarianceIndex() + X) = ( 2.0*q.y()*acceleration.y()+2.0*q.z()*acceleration.z()+2.0*q.x()*acceleration.x());
      A(state.getVelocityCovarianceIndex() + X,state.getOrientationCovarianceIndex() + Y) = (-2.0*q.y()*acceleration.x()+2.0*q.x()*acceleration.y()+2.0*q.w()*acceleration.z());
      A(state.getVelocityCovarianceIndex() + X,state.getOrientationCovarianceIndex() + Z) = (-2.0*q.z()*acceleration.x()-2.0*q.w()*acceleration.y()+2.0*q.x()*acceleration.z());

      A(state.getVelocityCovarianceIndex() + Y,state.getOrientationCovarianceIndex() + W) = (2.0*q.z()*acceleration.x()-2.0*q.x()*acceleration.z()+2.0*q.w()*acceleration.y());
      A(state.getVelocityCovarianceIndex() + Y,state.getOrientationCovarianceIndex() + X) = (2.0*q.y()*acceleration.x()-2.0*q.x()*acceleration.y()-2.0*q.w()*acceleration.z());
      A(state.getVelocityCovarianceIndex() + Y,state.getOrientationCovarianceIndex() + Y) = (2.0*q.x()*acceleration.x()+2.0*q.z()*acceleration.z()+2.0*q.y()*acceleration.y());
      A(state.getVelocityCovarianceIndex() + Y,state.getOrientationCovarianceIndex() + Z) = (2.0*q.w()*acceleration.x()-2.0*q.z()*acceleration.y()+2.0*q.y()*acceleration.z());
    }

    if (state.getSystemStatus() & STATE_VELOCITY_Z) {
      A(state.getVelocityCovarianceIndex() + Z,state.getOrientationCovarianceIndex() + W) = (-2.0*q.y()*acceleration.x()+2.0*q.x()*acceleration.y()+2.0*q.w()*acceleration.z());
      A(state.getVelocityCovarianceIndex() + Z,state.getOrientationCovarianceIndex() + X) = ( 2.0*q.z()*acceleration.x()+2.0*q.w()*acceleration.y()-2.0*q.x()*acceleration.z());
      A(state.getVelocityCovarianceIndex() + Z,state.getOrientationCovarianceIndex() + Y) = (-2.0*q.w()*acceleration.x()+2.0*q.z()*acceleration.y()-2.0*q.y()*acceleration.z());
      A(state.getVelocityCovarianceIndex() + Z,state.getOrientationCovarianceIndex() + Z) = ( 2.0*q.x()*acceleration.x()+2.0*q.y()*acceleration.y()+2.0*q.z()*acceleration.z());
    }
  }

  if (state.getPositionCovarianceIndex() >= 0 && state.getVelocityCovarianceIndex() >= 0) {
    if (state.getSystemStatus() & STATE_POSITION_XY) {
      A(state.getPositionCovarianceIndex() + X,state.getVelocityCovarianceIndex() + X)   = 1.0;
      A(state.getPositionCovarianceIndex() + Y,state.getVelocityCovarianceIndex() + Y)   = 1.0;
    }

    if (state.getSystemStatus() & STATE_POSITION_Z) {
      A(state.getPositionCovarianceIndex() + Z,state.getVelocityCovarianceIndex() + Z)   = 1.0;
    }
  }
#endif // VELOCITY_IN_BODY_FRAME

}

void GenericQuaternionSystemModel::getInputJacobian(InputMatrix& B, const State& state, bool init)
{
  throw std::runtime_error("not implemented");
}

SystemStatus GenericQuaternionSystemModel::getStatusFlags(const State& state)
{
  SystemStatus flags = state.getMeasurementStatus();
//     flags |= STATE_POSITION_XY | STATE_POSITION_Z;
  if (flags & STATE_POSITION_XY) flags |= STATE_VELOCITY_XY;
  if (flags & STATE_POSITION_Z)  flags |= STATE_VELOCITY_Z;
  if (flags & STATE_VELOCITY_XY) flags |= STATE_ROLLPITCH;
  if (flags & STATE_ROLLPITCH)   flags |= STATE_RATE_XY;
#ifndef USE_RATE_SYSTEM_MODEL
  flags |= STATE_RATE_XY | STATE_RATE_Z;
#endif
  return flags;
}

} // namespace hector_pose_estimation
