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
  if (state.getOrientationIndex() >= 0) {
    state.P()(state.getOrientationIndex(X),state.getOrientationIndex(X)) = 0.25 * 1.0;
    state.P()(state.getOrientationIndex(X),state.getOrientationIndex(X)) = 0.25 * 1.0;
    state.P()(state.getOrientationIndex(X),state.getOrientationIndex(X)) = 0.25 * 1.0;
    state.P()(state.getOrientationIndex(W),state.getOrientationIndex(W)) = 0.25 * 1.0;
  }

  if (state.getRateIndex() >= 0) {
    state.P()(state.getRateIndex(X),state.getRateIndex(X)) = pow(0.0 * M_PI/180.0, 2);
    state.P()(state.getRateIndex(Y),state.getRateIndex(Y)) = pow(0.0 * M_PI/180.0, 2);
    state.P()(state.getRateIndex(Z),state.getRateIndex(Z)) = pow(0.0 * M_PI/180.0, 2);
  }

  if (state.getPositionIndex() >= 0) {
    state.P()(state.getPositionIndex(X),state.getPositionIndex(X)) = 0.0;
    state.P()(state.getPositionIndex(Y),state.getPositionIndex(Y)) = 0.0;
    state.P()(state.getPositionIndex(Z),state.getPositionIndex(Z)) = 0.0;
  }

  if (state.getVelocityIndex() >= 0) {
    state.P()(state.getVelocityIndex(X),state.getVelocityIndex(X)) = 0.0;
    state.P()(state.getVelocityIndex(Y),state.getVelocityIndex(Y)) = 0.0;
    state.P()(state.getVelocityIndex(Z),state.getVelocityIndex(Z)) = 0.0;
  }
}

bool GenericQuaternionSystemModel::prepareUpdate(State& state, double dt)
{
  if (state.getAccelerationIndex() >= 0)
    acceleration = state.getAcceleration();
  else
    acceleration = imu_->getAcceleration() + accelerometer_->getModel()->getBias();

  if (state.getRateIndex() >= 0)
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

  x_dot.setZero();

  if (state.getOrientationIndex() >= 0) {
    x_dot(state.getOrientationIndex(W)) = 0.5*(                  (-rate.x())*q.x()+(-rate.y())*q.y()+(-rate.z())*q.z());
    x_dot(state.getOrientationIndex(X)) = 0.5*(( rate.x())*q.w()                  +( rate.z())*q.y()+(-rate.y())*q.z());
    x_dot(state.getOrientationIndex(Y)) = 0.5*(( rate.y())*q.w()+(-rate.z())*q.x()                  +( rate.x())*q.z());
    x_dot(state.getOrientationIndex(Z)) = 0.5*(( rate.z())*q.w()+( rate.y())*q.x()+(-rate.x())*q.y()                  );
  }

#ifdef VELOCITY_IN_BODY_FRAME
  ColumnVector3 wxv = rate.cross(v);

  if (state.getSystemStatus() & STATE_VELOCITY_XY && state.getVelocityIndex() >= 0) {
    x_dot(state.getVelocityIndex(X))  = acceleration.x() - wxv.x() + R(2,0) * gravity_;
    x_dot(state.getVelocityIndex(Y))  = acceleration.y() - wxv.y() + R(2,1) * gravity_;
  }
  if (state.getSystemStatus() & STATE_VELOCITY_Z && state.getVelocityIndex() >= 0) {
    x_dot(state.getVelocityIndex(Z))  = acceleration.z() - wxv.z() + R(2,2) * gravity_;
  }

  if (state.getSystemStatus() & STATE_POSITION_XY) {
    x_dot(state.getPositionIndex(X))  = R(0,0)*v.x() + R(0,1)*v.y() + R(0,2)*v.z();
    x_dot(state.getPositionIndex(Y))  = R(1,0)*v.x() + R(1,1)*v.y() + R(1,2)*v.z();
  }
  if (state.getSystemStatus() & STATE_POSITION_Z) {
    x_dot(state.getPositionIndex(Z))  = R(2,0)*v.x() + R(2,1)*v.y() + R(2,2)*v.z();
  }

#else
  if (state.getSystemStatus() & STATE_VELOCITY_XY && state.getVelocityIndex() >= 0) {
    x_dot(state.getVelocityIndex(X))  = R(0,0)*acceleration.x() + R(0,1)*acceleration.y() + R(0,2)*acceleration.z();
    x_dot(state.getVelocityIndex(Y))  = R(1,0)*acceleration.x() + R(1,1)*acceleration.y() + R(1,2)*acceleration.z();
  }
  if (state.getSystemStatus() & STATE_VELOCITY_Z && state.getVelocityIndex() >= 0) {
    x_dot(state.getVelocityIndex(Z))  = R(2,0)*acceleration.x() + R(2,1)*acceleration.y() + R(2,2)*acceleration.z() + gravity_;
  }

  if (state.getSystemStatus() & STATE_POSITION_XY) {
    x_dot(state.getPositionIndex(X))  = v.x();
    x_dot(state.getPositionIndex(Y))  = v.y();
  }
  if (state.getSystemStatus() & STATE_POSITION_Z) {
    x_dot(state.getPositionIndex(Z))  = v.z();
  }
#endif // VELOCITY_IN_BODY_FRAME
}

void GenericQuaternionSystemModel::getSystemNoise(NoiseVariance& Q, const State& state, bool init)
{
  if (init) {
    if (state.getRateIndex() >= 0)
      Q(state.getRateIndex(X),state.getRateIndex(X)) = Q(state.getRateIndex(Y),state.getRateIndex(Y)) = Q(state.getRateIndex(Z),state.getRateIndex(Z)) = pow(angular_acceleration_stddev_, 2);
    if (state.getPositionIndex() >= 0)
      Q(state.getPositionIndex(X),state.getPositionIndex(X)) = Q(state.getPositionIndex(Y),state.getPositionIndex(Y)) = Q(state.getPositionIndex(Z),state.getPositionIndex(Z)) = pow(velocity_stddev_, 2);
    if (state.getRateIndex() >= 0)
      Q(state.getVelocityIndex(X),state.getVelocityIndex(X)) = Q(state.getVelocityIndex(Y),state.getVelocityIndex(Y)) = Q(state.getVelocityIndex(Z),state.getVelocityIndex(Z)) = pow(acceleration_stddev_, 2);
//    Q(BIAS_ACCEL_X,BIAS_ACCEL_X) = Q(BIAS_ACCEL_Y,BIAS_ACCEL_Y) = pow(acceleration_drift_, 2);
//    Q(BIAS_ACCEL_Z,BIAS_ACCEL_Z) = pow(acceleration_drift_, 2);
//    Q(BIAS_GYRO_X,BIAS_GYRO_X) = Q(BIAS_GYRO_Y,BIAS_GYRO_Y) = Q(BIAS_GYRO_Z,BIAS_GYRO_Z) = pow(rate_drift_, 2);
  }

  if ((double) rate_stddev_ > 0.0 && state.getOrientationIndex() >= 0) {
    State::ConstOrientationType q(state.getOrientation());
    double rate_variance_4 = 0.25 * pow(rate_stddev_, 2);
    Q(state.getOrientationIndex(W),state.getOrientationIndex(W)) = rate_variance_4 * (q.x()*q.x()+q.y()*q.y()+q.z()*q.z());
    Q(state.getOrientationIndex(X),state.getOrientationIndex(X)) = rate_variance_4 * (q.w()*q.w()+q.y()*q.y()+q.z()*q.z());
    Q(state.getOrientationIndex(Y),state.getOrientationIndex(Y)) = rate_variance_4 * (q.w()*q.w()+q.x()*q.x()+q.z()*q.z());
    Q(state.getOrientationIndex(Z),state.getOrientationIndex(Z)) = rate_variance_4 * (q.w()*q.w()+q.x()*q.x()+q.y()*q.y());
  }
}

void GenericQuaternionSystemModel::getStateJacobian(SystemMatrix& A, const State& state, bool)
{
  State::ConstOrientationType q(state.getOrientation());
  State::ConstVelocityType v(state.getVelocity());

  A.setZero();

  //--> Set A-Matrix
  //----------------------------------------------------------
  if (state.getOrientationIndex() >= 0) {
    A(state.getOrientationIndex(W),state.getOrientationIndex(X)) = (-0.5*rate.x());
    A(state.getOrientationIndex(W),state.getOrientationIndex(Y)) = (-0.5*rate.y());
    A(state.getOrientationIndex(W),state.getOrientationIndex(Z)) = (-0.5*rate.z());
    A(state.getOrientationIndex(X),state.getOrientationIndex(W)) = ( 0.5*rate.x());
    A(state.getOrientationIndex(X),state.getOrientationIndex(Y)) = ( 0.5*rate.z());
    A(state.getOrientationIndex(X),state.getOrientationIndex(Z)) = (-0.5*rate.y());
    A(state.getOrientationIndex(Y),state.getOrientationIndex(W)) = ( 0.5*rate.y());
    A(state.getOrientationIndex(Y),state.getOrientationIndex(X)) = (-0.5*rate.z());
    A(state.getOrientationIndex(Y),state.getOrientationIndex(Z)) = ( 0.5*rate.x());
    A(state.getOrientationIndex(Z),state.getOrientationIndex(W)) = ( 0.5*rate.z());
    A(state.getOrientationIndex(Z),state.getOrientationIndex(X)) = ( 0.5*rate.y());
    A(state.getOrientationIndex(Z),state.getOrientationIndex(Y)) = (-0.5*rate.x());

    if (state.getRateIndex() >= 0) {
      A(state.getOrientationIndex(W),state.getRateIndex(X))  = -0.5*q.x();
      A(state.getOrientationIndex(W),state.getRateIndex(Y))  = -0.5*q.y();
      A(state.getOrientationIndex(W),state.getRateIndex(Z))  = -0.5*q.z();

      A(state.getOrientationIndex(X),state.getRateIndex(X))  =  0.5*q.w();
      A(state.getOrientationIndex(X),state.getRateIndex(Y))  = -0.5*q.z();
      A(state.getOrientationIndex(X),state.getRateIndex(Z))  = 0.5*q.y();

      A(state.getOrientationIndex(Y),state.getRateIndex(X))  = 0.5*q.z();
      A(state.getOrientationIndex(Y),state.getRateIndex(Y))  = 0.5*q.w();
      A(state.getOrientationIndex(Y),state.getRateIndex(Z))  = -0.5*q.x();

      A(state.getOrientationIndex(Z),state.getRateIndex(X))  = -0.5*q.y();
      A(state.getOrientationIndex(Z),state.getRateIndex(Y))  = 0.5*q.x();
      A(state.getOrientationIndex(Z),state.getRateIndex(Z))  = 0.5*q.w();
    }
  }

#ifdef VELOCITY_IN_BODY_FRAME

//  if (state.getSystemStatus() & STATE_VELOCITY_XY && state.getVelocityIndex() >= 0) {
//    x_dot(state.getVelocityIndex(X))  = acceleration.x() - wxv.x() + (2.0*q.x()*q.z()-2.0*q.w()*q.y()) * gravity_;
//    x_dot(state.getVelocityIndex(Y))  = acceleration.y() - wxv.y() + (2.0*q.y()*q.z()+2.0*q.w()*q.x()) * gravity_;
//  }
//  if (state.getSystemStatus() & STATE_VELOCITY_Z && state.getVelocityIndex() >= 0) {
//    x_dot(state.getVelocityIndex(Z))  = acceleration.z() - wxv.z() + (q.w()*q.w()-q.x()*q.x()-q.y()*q.y()+q.z()*q.z()) * gravity_;
//  }

  if (state.getVelocityIndex() >= 0 && state.getSystemStatus() & STATE_VELOCITY_XY) {
    if (state.getRateIndex() >= 0) {
      A(state.getVelocityIndex(X),state.getRateIndex(X)) =  0.0;
      A(state.getVelocityIndex(X),state.getRateIndex(Y)) = -v.z();
      A(state.getVelocityIndex(X),state.getRateIndex(Z)) =  v.y();

      A(state.getVelocityIndex(Y),state.getRateIndex(X)) =  v.z();
      A(state.getVelocityIndex(Y),state.getRateIndex(Y)) =  0.0;
      A(state.getVelocityIndex(Y),state.getRateIndex(Z)) = -v.x();
    }

    A(state.getVelocityIndex(X),state.getVelocityIndex(X)) =  0.0;
    A(state.getVelocityIndex(X),state.getVelocityIndex(Y)) =  rate.z();
    A(state.getVelocityIndex(X),state.getVelocityIndex(Z)) = -rate.y();

    A(state.getVelocityIndex(Y),state.getVelocityIndex(X)) = -rate.z();
    A(state.getVelocityIndex(Y),state.getVelocityIndex(Y)) =  0.0;
    A(state.getVelocityIndex(Y),state.getVelocityIndex(Z)) =  rate.x();

    if (state.getOrientationIndex() >= 0) {
      A(state.getVelocityIndex(X),state.getOrientationIndex(W)) = -2.0*q.y()*gravity_;
      A(state.getVelocityIndex(X),state.getOrientationIndex(X)) =  2.0*q.z()*gravity_;
      A(state.getVelocityIndex(X),state.getOrientationIndex(Y)) = -2.0*q.w()*gravity_;
      A(state.getVelocityIndex(X),state.getOrientationIndex(Z)) =  2.0*q.x()*gravity_;

      A(state.getVelocityIndex(Y),state.getOrientationIndex(W)) =  2.0*q.x()*gravity_;
      A(state.getVelocityIndex(Y),state.getOrientationIndex(X)) =  2.0*q.w()*gravity_;
      A(state.getVelocityIndex(Y),state.getOrientationIndex(Y)) =  2.0*q.z()*gravity_;
      A(state.getVelocityIndex(Y),state.getOrientationIndex(Z)) =  2.0*q.y()*gravity_;
    }
  }

  if (state.getVelocityIndex() >= 0 && state.getSystemStatus() & STATE_VELOCITY_Z) {
    if (state.getRateIndex() >= 0) {
      A(state.getVelocityIndex(Z),state.getRateIndex(X)) = -v.y();
      A(state.getVelocityIndex(Z),state.getRateIndex(Y)) =  v.x();
      A(state.getVelocityIndex(Z),state.getRateIndex(Z)) =  0.0;
    }

    A(state.getVelocityIndex(Z),state.getVelocityIndex(X)) =  rate.y();
    A(state.getVelocityIndex(Z),state.getVelocityIndex(Y)) = -rate.x();
    A(state.getVelocityIndex(Z),state.getVelocityIndex(Z)) =  0.0;

    if (state.getOrientationIndex() >= 0) {
      A(state.getVelocityIndex(Z),state.getOrientationIndex(W)) =  2.0*q.w()*gravity_;
      A(state.getVelocityIndex(Z),state.getOrientationIndex(X)) = -2.0*q.x()*gravity_;
      A(state.getVelocityIndex(Z),state.getOrientationIndex(Y)) = -2.0*q.y()*gravity_;
      A(state.getVelocityIndex(Z),state.getOrientationIndex(Z)) =  2.0*q.z()*gravity_;
    }
  }

  //  if (state.getSystemStatus() & STATE_POSITION_XY) {
  //    x_dot(state.getPositionIndex(X))  = (q.w()*q.w()+q.x()*q.x()-q.y()*q.y()-q.z()*q.z())*v.x() + (2.0*q.x()*q.y()-2.0*q.w()*q.z())                *v.y() + (2.0*q.x()*q.z()+2.0*q.w()*q.y())                *v.z();
  //    x_dot(state.getPositionIndex(Y))  = (2.0*q.x()*q.y()+2.0*q.w()*q.z())                *v.x() + (q.w()*q.w()-q.x()*q.x()+q.y()*q.y()-q.z()*q.z())*v.y() + (2.0*q.y()*q.z()-2.0*q.w()*q.x())                *v.z();
  //  }
  //  if (state.getSystemStatus() & STATE_POSITION_Z) {
  //    x_dot(state.getPositionIndex(Z))  = (2.0*q.x()*q.z()-2.0*q.w()*q.y())                *v.x() + (2.0*q.y()*q.z()+2.0*q.w()*q.x())                *v.y() + (q.w()*q.w()-q.x()*q.x()-q.y()*q.y()+q.z()*q.z())*v.z();
  //  }

  if (state.getPositionIndex() >= 0 && state.getSystemStatus() & STATE_POSITION_XY) {
    if (state.getOrientationIndex() >= 0) {
      A(state.getPositionIndex(X),state.getOrientationIndex(W)) = -2.0*q.z()*v.y()+2.0*q.y()*v.z()+2.0*q.w()*v.x();
      A(state.getPositionIndex(X),state.getOrientationIndex(X)) =  2.0*q.y()*v.y()+2.0*q.z()*v.z()+2.0*q.x()*v.x();
      A(state.getPositionIndex(X),state.getOrientationIndex(Y)) = -2.0*q.y()*v.x()+2.0*q.x()*v.y()+2.0*q.w()*v.z();
      A(state.getPositionIndex(X),state.getOrientationIndex(Z)) = -2.0*q.z()*v.x()-2.0*q.w()*v.y()+2.0*q.x()*v.z();

      A(state.getPositionIndex(Y),state.getOrientationIndex(W)) =  2.0*q.z()*v.x()-2.0*q.x()*v.z()+2.0*q.w()*v.y();
      A(state.getPositionIndex(Y),state.getOrientationIndex(X)) =  2.0*q.y()*v.x()-2.0*q.x()*v.y()-2.0*q.w()*v.z();
      A(state.getPositionIndex(Y),state.getOrientationIndex(Y)) =  2.0*q.x()*v.x()+2.0*q.z()*v.z()+2.0*q.y()*v.y();
      A(state.getPositionIndex(Y),state.getOrientationIndex(Z)) =  2.0*q.w()*v.x()-2.0*q.z()*v.y()+2.0*q.y()*v.z();
    }

    if (state.getVelocityIndex() >= 0) {
      A(state.getPositionIndex(X),state.getVelocityIndex(X))   =  R(0,0);
      A(state.getPositionIndex(X),state.getVelocityIndex(Y))   =  R(0,1);
      A(state.getPositionIndex(X),state.getVelocityIndex(Z))   =  R(0,2);

      A(state.getPositionIndex(Y),state.getVelocityIndex(X))   =  R(1,0);
      A(state.getPositionIndex(Y),state.getVelocityIndex(Y))   =  R(1,1);
      A(state.getPositionIndex(Y),state.getVelocityIndex(Z))   =  R(1,2);
    }
  }

  if (state.getPositionIndex() >= 0 && state.getSystemStatus() & STATE_POSITION_Z) {
    if (state.getOrientationIndex() >= 0) {
      A(state.getPositionIndex(Z),state.getOrientationIndex(W)) = -2.0*q.y()*v.x()+2.0*q.x()*v.y()+2.0*q.w()*v.z();
      A(state.getPositionIndex(Z),state.getOrientationIndex(X)) =  2.0*q.z()*v.x()+2.0*q.w()*v.y()-2.0*q.x()*v.z();
      A(state.getPositionIndex(Z),state.getOrientationIndex(Y)) = -2.0*q.w()*v.x()+2.0*q.z()*v.y()-2.0*q.y()*v.z();
      A(state.getPositionIndex(Z),state.getOrientationIndex(Z)) =  2.0*q.x()*v.x()+2.0*q.y()*v.y()+2.0*q.z()*v.z();
    }

    if (state.getVelocityIndex() >= 0) {
      A(state.getPositionIndex(Z),state.getVelocityIndex(X))   =  R(2,0);
      A(state.getPositionIndex(Z),state.getVelocityIndex(Y))   =  R(2,1);
      A(state.getPositionIndex(Z),state.getVelocityIndex(Z))   =  R(2,2);
    }
  }

#else

  if (state.getVelocityIndex() >= 0 && state.getOrientationIndex() >= 0) {
    if (state.getSystemStatus() & STATE_VELOCITY_XY) {
      A(state.getVelocityIndex(X),state.getOrientationIndex(W)) = (-2.0*q.z()*acceleration.y()+2.0*q.y()*acceleration.z()+2.0*q.w()*acceleration.x());
      A(state.getVelocityIndex(X),state.getOrientationIndex(X)) = ( 2.0*q.y()*acceleration.y()+2.0*q.z()*acceleration.z()+2.0*q.x()*acceleration.x());
      A(state.getVelocityIndex(X),state.getOrientationIndex(Y)) = (-2.0*q.y()*acceleration.x()+2.0*q.x()*acceleration.y()+2.0*q.w()*acceleration.z());
      A(state.getVelocityIndex(X),state.getOrientationIndex(Z)) = (-2.0*q.z()*acceleration.x()-2.0*q.w()*acceleration.y()+2.0*q.x()*acceleration.z());

      A(state.getVelocityIndex(Y),state.getOrientationIndex(W)) = (2.0*q.z()*acceleration.x()-2.0*q.x()*acceleration.z()+2.0*q.w()*acceleration.y());
      A(state.getVelocityIndex(Y),state.getOrientationIndex(X)) = (2.0*q.y()*acceleration.x()-2.0*q.x()*acceleration.y()-2.0*q.w()*acceleration.z());
      A(state.getVelocityIndex(Y),state.getOrientationIndex(Y)) = (2.0*q.x()*acceleration.x()+2.0*q.z()*acceleration.z()+2.0*q.y()*acceleration.y());
      A(state.getVelocityIndex(Y),state.getOrientationIndex(Z)) = (2.0*q.w()*acceleration.x()-2.0*q.z()*acceleration.y()+2.0*q.y()*acceleration.z());
    }

    if (state.getSystemStatus() & STATE_VELOCITY_Z) {
      A(state.getVelocityIndex(Z),state.getOrientationIndex(W)) = (-2.0*q.y()*acceleration.x()+2.0*q.x()*acceleration.y()+2.0*q.w()*acceleration.z());
      A(state.getVelocityIndex(Z),state.getOrientationIndex(X)) = ( 2.0*q.z()*acceleration.x()+2.0*q.w()*acceleration.y()-2.0*q.x()*acceleration.z());
      A(state.getVelocityIndex(Z),state.getOrientationIndex(Y)) = (-2.0*q.w()*acceleration.x()+2.0*q.z()*acceleration.y()-2.0*q.y()*acceleration.z());
      A(state.getVelocityIndex(Z),state.getOrientationIndex(Z)) = ( 2.0*q.x()*acceleration.x()+2.0*q.y()*acceleration.y()+2.0*q.z()*acceleration.z());
    }
  }

  if (state.getPositionIndex() >= 0 && state.getVelocityIndex() >= 0) {
    if (state.getSystemStatus() & STATE_POSITION_XY) {
      A(state.getPositionIndex(X),state.getVelocityIndex(X))   = 1.0;
      A(state.getPositionIndex(Y),state.getVelocityIndex(Y))   = 1.0;
    }

    if (state.getSystemStatus() & STATE_POSITION_Z) {
      A(state.getPositionIndex(Z),state.getVelocityIndex(Z))   = 1.0;
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
