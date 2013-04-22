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
    state.P()(State::QUATERNION_W,State::QUATERNION_W) = 0.25 * 1.0;
    state.P()(State::QUATERNION_X,State::QUATERNION_X) = 0.25 * 1.0;
    state.P()(State::QUATERNION_Y,State::QUATERNION_Y) = 0.25 * 1.0;
    state.P()(State::QUATERNION_Z,State::QUATERNION_Z) = 0.25 * 1.0;
  }

  if (state.getRateIndex() >= 0) {
    state.P()(State::RATE_X,State::RATE_X) = pow(0.0 * M_PI/180.0, 2);
    state.P()(State::RATE_Y,State::RATE_Y) = pow(0.0 * M_PI/180.0, 2);
    state.P()(State::RATE_Z,State::RATE_Z) = pow(0.0 * M_PI/180.0, 2);
  }

  if (state.getPositionIndex() >= 0) {
    state.P()(State::POSITION_X,State::POSITION_X) = 0.0;
    state.P()(State::POSITION_Y,State::POSITION_Y) = 0.0;
    state.P()(State::POSITION_Z,State::POSITION_Z) = 0.0;
  }

  if (state.getVelocityIndex() >= 0) {
    state.P()(State::VELOCITY_X,State::VELOCITY_X) = 0.0;
    state.P()(State::VELOCITY_Y,State::VELOCITY_Y) = 0.0;
    state.P()(State::VELOCITY_Z,State::VELOCITY_Z) = 0.0;
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

  x_dot = 0.0;

  if (state.getOrientationIndex() >= 0) {
    x_dot(State::QUATERNION_W) = 0.5*(                  (-rate.x())*q.x()+(-rate.y())*q.y()+(-rate.z())*q.z());
    x_dot(State::QUATERNION_X) = 0.5*(( rate.x())*q.w()                  +( rate.z())*q.y()+(-rate.y())*q.z());
    x_dot(State::QUATERNION_Y) = 0.5*(( rate.y())*q.w()+(-rate.z())*q.x()                  +( rate.x())*q.z());
    x_dot(State::QUATERNION_Z) = 0.5*(( rate.z())*q.w()+( rate.y())*q.x()+(-rate.x())*q.y()                  );
  }

#ifdef VELOCITY_IN_BODY_FRAME
  ColumnVector3 wxv = rate.cross(v);

  if (state.getSystemStatus() & STATE_VELOCITY_XY && state.getVelocityIndex() >= 0) {
    x_dot(State::VELOCITY_X)  = acceleration.x() - wxv.x() + R(2,0) * gravity_;
    x_dot(State::VELOCITY_Y)  = acceleration.y() - wxv.y() + R(2,1) * gravity_;
  }
  if (state.getSystemStatus() & STATE_VELOCITY_Z && state.getVelocityIndex() >= 0) {
    x_dot(State::VELOCITY_Z)  = acceleration.z() - wxv.z() + R(2,2) * gravity_;
  }

  if (state.getSystemStatus() & STATE_POSITION_XY) {
    x_dot(State::POSITION_X)  = R(0,0)*v.x() + R(0,1)*v.y() + R(0,2)*v.z();
    x_dot(State::POSITION_Y)  = R(1,0)*v.x() + R(1,1)*v.y() + R(1,2)*v.z();
  }
  if (state.getSystemStatus() & STATE_POSITION_Z) {
    x_dot(State::POSITION_Z)  = R(2,0)*v.x() + R(2,1)*v.y() + R(2,2)*v.z();
  }

#else
  if (state.getSystemStatus() & STATE_VELOCITY_XY && state.getVelocityIndex() >= 0) {
    x_dot(State::VELOCITY_X)  = R(0,0)*acceleration.x() + R(0,1)*acceleration.y() + R(0,2)*acceleration.z();
    x_dot(State::VELOCITY_Y)  = R(1,0)*acceleration.x() + R(1,1)*acceleration.y() + R(1,2)*acceleration.z();
  }
  if (state.getSystemStatus() & STATE_VELOCITY_Z && state.getVelocityIndex() >= 0) {
    x_dot(State::VELOCITY_Z)  = R(2,0)*acceleration.x() + R(2,1)*acceleration.y() + R(2,2)*acceleration.z() + gravity_;
  }

  if (state.getSystemStatus() & STATE_POSITION_XY) {
    x_dot(State::POSITION_X)  = v.x();
    x_dot(State::POSITION_Y)  = v.y();
  }
  if (state.getSystemStatus() & STATE_POSITION_Z) {
    x_dot(State::POSITION_Z)  = v.z();
  }
#endif // VELOCITY_IN_BODY_FRAME
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
//    Q(BIAS_GYRO_X,BIAS_GYRO_X) = Q(BIAS_GYRO_Y,BIAS_GYRO_Y) = Q(BIAS_GYRO_Z,BIAS_GYRO_Z) = pow(rate_drift_, 2);
  }

  if ((double) rate_stddev_ > 0.0 && state.getOrientationIndex() >= 0) {
    State::ConstOrientationType q(state.getOrientation());
    double rate_variance_4 = 0.25 * pow(rate_stddev_, 2);
    Q(State::QUATERNION_W,State::QUATERNION_W) = rate_variance_4 * (q.x()*q.x()+q.y()*q.y()+q.z()*q.z());
    Q(State::QUATERNION_X,State::QUATERNION_X) = rate_variance_4 * (q.w()*q.w()+q.y()*q.y()+q.z()*q.z());
    Q(State::QUATERNION_Y,State::QUATERNION_Y) = rate_variance_4 * (q.w()*q.w()+q.x()*q.x()+q.z()*q.z());
    Q(State::QUATERNION_Z,State::QUATERNION_Z) = rate_variance_4 * (q.w()*q.w()+q.x()*q.x()+q.y()*q.y());
  }
}

void GenericQuaternionSystemModel::getStateJacobian(SystemMatrix& A, const State& state, bool)
{
  State::ConstOrientationType q(state.getOrientation());
  State::ConstVelocityType v(state.getVelocity());

  A = 0.0;

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

#ifdef VELOCITY_IN_BODY_FRAME

//  if (state.getSystemStatus() & STATE_VELOCITY_XY && state.getVelocityIndex() >= 0) {
//    x_dot(State::VELOCITY_X)  = acceleration.x() - wxv.x() + (2.0*q.x()*q.z()-2.0*q.w()*q.y()) * gravity_;
//    x_dot(State::VELOCITY_Y)  = acceleration.y() - wxv.y() + (2.0*q.y()*q.z()+2.0*q.w()*q.x()) * gravity_;
//  }
//  if (state.getSystemStatus() & STATE_VELOCITY_Z && state.getVelocityIndex() >= 0) {
//    x_dot(State::VELOCITY_Z)  = acceleration.z() - wxv.z() + (q.w()*q.w()-q.x()*q.x()-q.y()*q.y()+q.z()*q.z()) * gravity_;
//  }

  if (state.getVelocityIndex() >= 0 && state.getSystemStatus() & STATE_VELOCITY_XY) {
    if (state.getRateIndex() >= 0) {
      A(State::VELOCITY_X,State::RATE_X) =  0.0;
      A(State::VELOCITY_X,State::RATE_Y) = -v.z();
      A(State::VELOCITY_X,State::RATE_Z) =  v.y();

      A(State::VELOCITY_Y,State::RATE_X) =  v.z();
      A(State::VELOCITY_Y,State::RATE_Y) =  0.0;
      A(State::VELOCITY_Y,State::RATE_Z) = -v.x();
    }

    A(State::VELOCITY_X,State::VELOCITY_X) =  0.0;
    A(State::VELOCITY_X,State::VELOCITY_Y) =  rate.z();
    A(State::VELOCITY_X,State::VELOCITY_Z) = -rate.y();

    A(State::VELOCITY_Y,State::VELOCITY_X) = -rate.z();
    A(State::VELOCITY_Y,State::VELOCITY_Y) =  0.0;
    A(State::VELOCITY_Y,State::VELOCITY_Z) =  rate.x();

    if (state.getOrientationIndex() >= 0) {
      A(State::VELOCITY_X,State::QUATERNION_W) = -2.0*q.y()*gravity_;
      A(State::VELOCITY_X,State::QUATERNION_X) =  2.0*q.z()*gravity_;
      A(State::VELOCITY_X,State::QUATERNION_Y) = -2.0*q.w()*gravity_;
      A(State::VELOCITY_X,State::QUATERNION_Z) =  2.0*q.x()*gravity_;

      A(State::VELOCITY_Y,State::QUATERNION_W) =  2.0*q.x()*gravity_;
      A(State::VELOCITY_Y,State::QUATERNION_X) =  2.0*q.w()*gravity_;
      A(State::VELOCITY_Y,State::QUATERNION_Y) =  2.0*q.z()*gravity_;
      A(State::VELOCITY_Y,State::QUATERNION_Z) =  2.0*q.y()*gravity_;
    }
  }

  if (state.getVelocityIndex() >= 0 && state.getSystemStatus() & STATE_VELOCITY_Z) {
    if (state.getRateIndex() >= 0) {
      A(State::VELOCITY_Z,State::RATE_X) = -v.y();
      A(State::VELOCITY_Z,State::RATE_Y) =  v.x();
      A(State::VELOCITY_Z,State::RATE_Z) =  0.0;
    }

    A(State::VELOCITY_Z,State::VELOCITY_X) =  rate.y();
    A(State::VELOCITY_Z,State::VELOCITY_Y) = -rate.x();
    A(State::VELOCITY_Z,State::VELOCITY_Z) =  0.0;

    if (state.getOrientationIndex() >= 0) {
      A(State::VELOCITY_Z,State::QUATERNION_W) =  2.0*q.w()*gravity_;
      A(State::VELOCITY_Z,State::QUATERNION_X) = -2.0*q.x()*gravity_;
      A(State::VELOCITY_Z,State::QUATERNION_Y) = -2.0*q.y()*gravity_;
      A(State::VELOCITY_Z,State::QUATERNION_Z) =  2.0*q.z()*gravity_;
    }
  }

  //  if (state.getSystemStatus() & STATE_POSITION_XY) {
  //    x_dot(State::POSITION_X)  = (q.w()*q.w()+q.x()*q.x()-q.y()*q.y()-q.z()*q.z())*v.x() + (2.0*q.x()*q.y()-2.0*q.w()*q.z())                *v.y() + (2.0*q.x()*q.z()+2.0*q.w()*q.y())                *v.z();
  //    x_dot(State::POSITION_Y)  = (2.0*q.x()*q.y()+2.0*q.w()*q.z())                *v.x() + (q.w()*q.w()-q.x()*q.x()+q.y()*q.y()-q.z()*q.z())*v.y() + (2.0*q.y()*q.z()-2.0*q.w()*q.x())                *v.z();
  //  }
  //  if (state.getSystemStatus() & STATE_POSITION_Z) {
  //    x_dot(State::POSITION_Z)  = (2.0*q.x()*q.z()-2.0*q.w()*q.y())                *v.x() + (2.0*q.y()*q.z()+2.0*q.w()*q.x())                *v.y() + (q.w()*q.w()-q.x()*q.x()-q.y()*q.y()+q.z()*q.z())*v.z();
  //  }

  if (state.getPositionIndex() >= 0 && state.getSystemStatus() & STATE_POSITION_XY) {
    if (state.getOrientationIndex() >= 0) {
      A(State::POSITION_X,State::QUATERNION_W) = -2.0*q.z()*v.y()+2.0*q.y()*v.z()+2.0*q.w()*v.x();
      A(State::POSITION_X,State::QUATERNION_X) =  2.0*q.y()*v.y()+2.0*q.z()*v.z()+2.0*q.x()*v.x();
      A(State::POSITION_X,State::QUATERNION_Y) = -2.0*q.y()*v.x()+2.0*q.x()*v.y()+2.0*q.w()*v.z();
      A(State::POSITION_X,State::QUATERNION_Z) = -2.0*q.z()*v.x()-2.0*q.w()*v.y()+2.0*q.x()*v.z();

      A(State::POSITION_Y,State::QUATERNION_W) =  2.0*q.z()*v.x()-2.0*q.x()*v.z()+2.0*q.w()*v.y();
      A(State::POSITION_Y,State::QUATERNION_X) =  2.0*q.y()*v.x()-2.0*q.x()*v.y()-2.0*q.w()*v.z();
      A(State::POSITION_Y,State::QUATERNION_Y) =  2.0*q.x()*v.x()+2.0*q.z()*v.z()+2.0*q.y()*v.y();
      A(State::POSITION_Y,State::QUATERNION_Z) =  2.0*q.w()*v.x()-2.0*q.z()*v.y()+2.0*q.y()*v.z();
    }

    if (state.getVelocityIndex() >= 0) {
      A(State::POSITION_X,State::VELOCITY_X)   =  R(0,0);
      A(State::POSITION_X,State::VELOCITY_Y)   =  R(0,1);
      A(State::POSITION_X,State::VELOCITY_Z)   =  R(0,2);

      A(State::POSITION_Y,State::VELOCITY_X)   =  R(1,0);
      A(State::POSITION_Y,State::VELOCITY_Y)   =  R(1,1);
      A(State::POSITION_Y,State::VELOCITY_Z)   =  R(1,2);
    }
  }

  if (state.getPositionIndex() >= 0 && state.getSystemStatus() & STATE_POSITION_Z) {
    if (state.getOrientationIndex() >= 0) {
      A(State::POSITION_Z,State::QUATERNION_W) = -2.0*q.y()*v.x()+2.0*q.x()*v.y()+2.0*q.w()*v.z();
      A(State::POSITION_Z,State::QUATERNION_X) =  2.0*q.z()*v.x()+2.0*q.w()*v.y()-2.0*q.x()*v.z();
      A(State::POSITION_Z,State::QUATERNION_Y) = -2.0*q.w()*v.x()+2.0*q.z()*v.y()-2.0*q.y()*v.z();
      A(State::POSITION_Z,State::QUATERNION_Z) =  2.0*q.x()*v.x()+2.0*q.y()*v.y()+2.0*q.z()*v.z();
    }

    if (state.getVelocityIndex() >= 0) {
      A(State::POSITION_Z,State::VELOCITY_X)   =  R(2,0);
      A(State::POSITION_Z,State::VELOCITY_Y)   =  R(2,1);
      A(State::POSITION_Z,State::VELOCITY_Z)   =  R(2,2);
    }
  }

#else

  if (state.getVelocityIndex() >= 0 && state.getOrientationIndex() >= 0) {
    if (state.getSystemStatus() & STATE_VELOCITY_XY) {
      A(State::VELOCITY_X,State::QUATERNION_W) = (-2.0*q.z()*acceleration.y()+2.0*q.y()*acceleration.z()+2.0*q.w()*acceleration.x());
      A(State::VELOCITY_X,State::QUATERNION_X) = ( 2.0*q.y()*acceleration.y()+2.0*q.z()*acceleration.z()+2.0*q.x()*acceleration.x());
      A(State::VELOCITY_X,State::QUATERNION_Y) = (-2.0*q.y()*acceleration.x()+2.0*q.x()*acceleration.y()+2.0*q.w()*acceleration.z());
      A(State::VELOCITY_X,State::QUATERNION_Z) = (-2.0*q.z()*acceleration.x()-2.0*q.w()*acceleration.y()+2.0*q.x()*acceleration.z());

      A(State::VELOCITY_Y,State::QUATERNION_W) = (2.0*q.z()*acceleration.x()-2.0*q.x()*acceleration.z()+2.0*q.w()*acceleration.y());
      A(State::VELOCITY_Y,State::QUATERNION_X) = (2.0*q.y()*acceleration.x()-2.0*q.x()*acceleration.y()-2.0*q.w()*acceleration.z());
      A(State::VELOCITY_Y,State::QUATERNION_Y) = (2.0*q.x()*acceleration.x()+2.0*q.z()*acceleration.z()+2.0*q.y()*acceleration.y());
      A(State::VELOCITY_Y,State::QUATERNION_Z) = (2.0*q.w()*acceleration.x()-2.0*q.z()*acceleration.y()+2.0*q.y()*acceleration.z());
    }

    if (state.getSystemStatus() & STATE_VELOCITY_Z) {
      A(State::VELOCITY_Z,State::QUATERNION_W) = (-2.0*q.y()*acceleration.x()+2.0*q.x()*acceleration.y()+2.0*q.w()*acceleration.z());
      A(State::VELOCITY_Z,State::QUATERNION_X) = ( 2.0*q.z()*acceleration.x()+2.0*q.w()*acceleration.y()-2.0*q.x()*acceleration.z());
      A(State::VELOCITY_Z,State::QUATERNION_Y) = (-2.0*q.w()*acceleration.x()+2.0*q.z()*acceleration.y()-2.0*q.y()*acceleration.z());
      A(State::VELOCITY_Z,State::QUATERNION_Z) = ( 2.0*q.x()*acceleration.x()+2.0*q.y()*acceleration.y()+2.0*q.z()*acceleration.z());
    }
  }

  if (state.getPositionIndex() >= 0 && state.getVelocityIndex() >= 0) {
    if (state.getSystemStatus() & STATE_POSITION_XY) {
      A(State::POSITION_X,State::VELOCITY_X)   = 1.0;
      A(State::POSITION_Y,State::VELOCITY_Y)   = 1.0;
    }

    if (state.getSystemStatus() & STATE_POSITION_Z) {
      A(State::POSITION_Z,State::VELOCITY_Z)   = 1.0;
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
