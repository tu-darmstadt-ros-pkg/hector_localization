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
#include <hector_pose_estimation/filter/set_filter.h>

#include <hector_pose_estimation/pose_estimation.h>

namespace hector_pose_estimation {

template class System_<GenericQuaternionSystemModel>;

GenericQuaternionSystemModel::GenericQuaternionSystemModel()
{
  angular_acceleration_stddev_ = 360.0 * M_PI/180.0;
  rate_stddev_ = 0.0;
  acceleration_stddev_ = 0.0;
  velocity_stddev_ = 0.0;

//  parameters().addAlias("gravity", gravity_);
  parameters().add("angular_acceleration_stddev", angular_acceleration_stddev_);
  parameters().add("rate_stddev", rate_stddev_);
  parameters().add("acceleration_stddev", acceleration_stddev_);
  parameters().add("velocity_stddev", velocity_stddev_);
}

GenericQuaternionSystemModel::~GenericQuaternionSystemModel()
{
}

bool GenericQuaternionSystemModel::init(PoseEstimation& estimator, State& state)
{
//  gyro_ = System::create(new GyroModel, "gyro");
//  accelerometer_ = System::create(new AccelerometerModel, "accelerometer");

  gravity_ = estimator.parameters().get("gravity_magnitude");
//  rate_stddev_ = gyro_->parameters().get("stddev");
//  acceleration_stddev_ = accelerometer_->parameters().get("stddev");

  imu_ = estimator.addInput<ImuInput>("raw_imu");
//  estimator.addSystem(gyro_, "gyro");
//  estimator.addSystem(accelerometer_, "accelerometer");
  return true;
}

void GenericQuaternionSystemModel::getPrior(State &state) {
  if (state.orientation()) {
    state.orientation()->P()(X,X) = 1.0;
    state.orientation()->P()(Y,Y) = 1.0;
    state.orientation()->P()(Z,Z) = 1.0;
  }

  if (state.rate()) {
    state.rate()->P()(X,X) = pow(0.0 * M_PI/180.0, 2);
    state.rate()->P()(Y,Y) = pow(0.0 * M_PI/180.0, 2);
    state.rate()->P()(Z,Z) = pow(0.0 * M_PI/180.0, 2);
  }

  if (state.position()) {
    state.position()->P()(X,X) = 0.0;
    state.position()->P()(Y,Y) = 0.0;
    state.position()->P()(Z,Z) = 0.0;
  }

  if (state.velocity()) {
    state.velocity()->P()(X,X) = 0.0;
    state.velocity()->P()(Y,Y) = 0.0;
    state.velocity()->P()(Z,Z) = 0.0;
  }
}

bool GenericQuaternionSystemModel::prepareUpdate(State& state, const Inputs& inputs, double dt)
{
  if (state.acceleration()) {
    acceleration_nav_ = state.R() * state.getAcceleration();
    ROS_DEBUG_STREAM("a_nav = [" << acceleration_nav_.transpose() << "]");
//  } else {
//    acceleration_nav_.setZero();
  }

  if (force_input_) {
    force_input_nav_ = state.R() * force_input_->getVector();
//  } else {
//    force_input_nav_.setZero();
  }

  if (imu_) {
    imu_acceleration_nav_ = state.R() * imu_->getAcceleration();
//  } else {
//    imu_acceleration_nav_.setZero();
  }

  return true;
}

void GenericQuaternionSystemModel::getDerivative(StateVector& x_dot, const State& state)
{
  const State::RotationMatrix &R = state.R();

  x_dot.setZero();

  if (state.rate()) {
    if (torque_input_) {
      state.rate()->segment(x_dot) = torque_input_->getVector();
    }
  }

  if (state.orientation()) {
    if (state.rate()) {
      state.orientation()->segment(x_dot).head(3) = R * state.getRate();
    } else if (imu_) {
      state.orientation()->segment(x_dot).head(3) = R * imu_->getRate();
    }
  }

  if (state.velocity()) {
    if (state.acceleration()) {
      if (state.getSystemStatus() & STATE_VELOCITY_XY) {
        state.velocity()->segment(x_dot)(X) = acceleration_nav_.x();
        state.velocity()->segment(x_dot)(Y) = acceleration_nav_.y();
      }
      if (state.getSystemStatus() & STATE_VELOCITY_Z) {
        state.velocity()->segment(x_dot)(Z) = acceleration_nav_.z();
      }
    } else if (force_input_) {
      state.velocity()->segment(x_dot) = R * force_input_->getVector();
    } else if (imu_) {
      if (state.getSystemStatus() & STATE_VELOCITY_XY) {
        state.velocity()->segment(x_dot)(X) = imu_acceleration_nav_.x();
        state.velocity()->segment(x_dot)(Y) = imu_acceleration_nav_.y();
      }
      if (state.getSystemStatus() & STATE_VELOCITY_Z) {
        state.velocity()->segment(x_dot)(Z) = imu_acceleration_nav_.z() + gravity_;
      }
    }
  }

  if (state.position()) {
    State::ConstVelocityType v(state.getVelocity());
    if (state.getSystemStatus() & STATE_POSITION_XY) {
      state.position()->segment(x_dot)(X) = v.x();
      state.position()->segment(x_dot)(Y) = v.y();
    }
    if (state.getSystemStatus() & STATE_POSITION_Z) {
      state.position()->segment(x_dot)(Z) = v.z();
    }
  }
}

void GenericQuaternionSystemModel::getSystemNoise(NoiseVariance& Q, const State& state, const Inputs &, bool init)
{
  if (!init) return;

  Q.setZero();
  if (state.orientation())
    state.orientation()->block(Q)(X,X) = state.orientation()->block(Q)(Y,Y) = state.orientation()->block(Q)(Z,Z) = pow(rate_stddev_, 2);
  if (state.rate())
    state.rate()->block(Q)(X,X) = state.rate()->block(Q)(Y,Y) = state.rate()->block(Q)(Z,Z) = pow(angular_acceleration_stddev_, 2);
  if (state.position())
    state.position()->block(Q)(X,X) = state.position()->block(Q)(Y,Y) = state.position()->block(Q)(Z,Z) = pow(velocity_stddev_, 2);
  if (state.velocity())
    state.velocity()->block(Q)(X,X) = state.velocity()->block(Q)(Y,Y) = state.velocity()->block(Q)(Z,Z) = pow(acceleration_stddev_, 2);
}

void GenericQuaternionSystemModel::getStateJacobian(SystemMatrix& A, const State& state)
{
  const State::RotationMatrix &R = state.R();
  A.setZero();

  if (state.orientation() && state.rate()) {
    state.orientation()->block(A, *state.rate()) = R;
  }

  if (state.velocity() && state.orientation()) {
    if (state.acceleration()) {
      if (state.getSystemStatus() & STATE_VELOCITY_XY) {
        state.velocity()->block(A, *state.orientation())(X,X) = 0.0;
        state.velocity()->block(A, *state.orientation())(X,Y) = acceleration_nav_.z();
        state.velocity()->block(A, *state.orientation())(X,Z) = -acceleration_nav_.y();

        state.velocity()->block(A, *state.orientation())(Y,X) = -acceleration_nav_.z();
        state.velocity()->block(A, *state.orientation())(Y,Y) = 0.0;
        state.velocity()->block(A, *state.orientation())(Y,Z) = acceleration_nav_.x();
      }

      if (state.getSystemStatus() & STATE_VELOCITY_Z) {
        state.velocity()->block(A, *state.orientation())(Z,X) = acceleration_nav_.y();
        state.velocity()->block(A, *state.orientation())(Z,Y) = -acceleration_nav_.x();
        state.velocity()->block(A, *state.orientation())(Z,Z) = 0.0;
      }

    } else if (force_input_) {
      if (state.getSystemStatus() & STATE_VELOCITY_XY) {
        state.velocity()->block(A, *state.orientation())(X,X) = 0.0;
        state.velocity()->block(A, *state.orientation())(X,Y) = force_input_nav_.z();
        state.velocity()->block(A, *state.orientation())(X,Z) = -force_input_nav_.y();

        state.velocity()->block(A, *state.orientation())(Y,X) = -force_input_nav_.z();
        state.velocity()->block(A, *state.orientation())(Y,Y) = 0.0;
        state.velocity()->block(A, *state.orientation())(Y,Z) = force_input_nav_.x();
      }

      if (state.getSystemStatus() & STATE_VELOCITY_Z) {
        state.velocity()->block(A, *state.orientation())(Z,X) = force_input_nav_.y();
        state.velocity()->block(A, *state.orientation())(Z,Y) = -force_input_nav_.x();
        state.velocity()->block(A, *state.orientation())(Z,Z) = 0.0;
      }

    } else if (imu_) {
      if (state.getSystemStatus() & STATE_VELOCITY_XY) {
        state.velocity()->block(A, *state.orientation())(X,X) = 0.0;
        state.velocity()->block(A, *state.orientation())(X,Y) = imu_acceleration_nav_.z();
        state.velocity()->block(A, *state.orientation())(X,Z) = -imu_acceleration_nav_.y();

        state.velocity()->block(A, *state.orientation())(Y,X) = -imu_acceleration_nav_.z();
        state.velocity()->block(A, *state.orientation())(Y,Y) = 0.0;
        state.velocity()->block(A, *state.orientation())(Y,Z) = imu_acceleration_nav_.x();
      }

      if (state.getSystemStatus() & STATE_VELOCITY_Z) {
        state.velocity()->block(A, *state.orientation())(Z,X) = imu_acceleration_nav_.y();
        state.velocity()->block(A, *state.orientation())(Z,Y) = -imu_acceleration_nav_.x();
        state.velocity()->block(A, *state.orientation())(Z,Z) = 0.0;
      }
    }
  }

  if (state.position() && state.velocity()) {
    if (state.getSystemStatus() & STATE_POSITION_XY) {
      state.position()->block(A, *state.velocity())(X,X)   = 1.0;
      state.position()->block(A, *state.velocity())(Y,Y)   = 1.0;
    }

    if (state.getSystemStatus() & STATE_POSITION_Z) {
      state.position()->block(A, *state.velocity())(Z,Z)   = 1.0;
    }
  }

}

SystemStatus GenericQuaternionSystemModel::getStatusFlags(const State& state)
{
  SystemStatus flags = state.getMeasurementStatus();
  if (flags & STATE_POSITION_XY) flags |= STATE_VELOCITY_XY;
  if (flags & STATE_POSITION_Z)  flags |= STATE_VELOCITY_Z;
  if (imu_ && (flags & STATE_VELOCITY_XY)) flags |= STATE_ROLLPITCH;
  if (flags & STATE_ROLLPITCH)   flags |= STATE_RATE_XY;
  return flags;
}

} // namespace hector_pose_estimation
