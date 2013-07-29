//=================================================================================================
// Copyright (c) 2013, Johannes Meyer and contributors, Technische Universitat Darmstadt
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

#include <hector_pose_estimation/system/ground_vehicle_model.h>
#include <hector_pose_estimation/pose_estimation.h>
#include <hector_pose_estimation/filter/set_filter.h>

#include <limits>

namespace hector_pose_estimation {

template class System_<GroundVehicleModel>;

GroundVehicleModel::GroundVehicleModel()
{
  gain_ = 1.0;
  base_height_ = 0.0;
  min_height_ = -std::numeric_limits<double>::quiet_NaN();
  max_height_ =  std::numeric_limits<double>::quiet_NaN();

  parameters().add("gain", gain_);
  parameters().add("base_height", base_height_);
  parameters().add("min_height", min_height_);
  parameters().add("max_height", max_height_);
}

GroundVehicleModel::~GroundVehicleModel()
{
}

void GroundVehicleModel::getPrior(State &state)
{
  GenericQuaternionSystemModel::getPrior(state);
  state.position().z() = base_height_;
}

void GroundVehicleModel::getDerivative(StateVector& x_dot, const State& state)
{
  // forward to GenericQuaternionSystemModel
  GenericQuaternionSystemModel::getDerivative(x_dot, state);

  // State::ConstOrientationType q(state.getOrientation());
  State::ConstVelocityType v(state.getVelocity());
  // State::ConstPositionType p(state.getPosition());

  // Update the body z velocity towards 0
#ifdef VELOCITY_IN_BODY_FRAME
  if (state.getVelocityIndex() >= 0) {
    x_dot(State::VELOCITY_Z) = -gain_ * v.z();
  }

#else
  if (state.getVelocityIndex() >= 0) {
    // v_z_body = R.row(2).dot(v)
    x_dot(State::VELOCITY_Z) = -gain_ * R(2,2) * R.row(2).dot(v);
  }
#endif // VELOCITY_IN_BODY_FRAME
}

void GroundVehicleModel::getStateJacobian(SystemMatrix& A, const State& state, bool init)
{
  GenericQuaternionSystemModel::getStateJacobian(A, state, init);

  State::ConstOrientationType q(state.getOrientation());
  State::ConstVelocityType v(state.getVelocity());

#ifdef VELOCITY_IN_BODY_FRAME
  if (state.getVelocityIndex() >= 0) {
    A(State::VELOCITY_Z,State::VELOCITY_Z) = -gain_;
  }

#else
  if (state.getVelocityIndex() >= 0) {
    A.block<1,3>(State::VELOCITY_Z,State::VELOCITY_X) = -gain_ * R(2,2) * R.row(2);

    if (state.getOrientationIndex() >= 0) {
      dr3_dq_ <<  2*q.y(),  2*q.z(),  2*q.w(), 2*q.x(),
                 -2*q.x(), -2*q.w(),  2*q.z(), 2*q.y(),
                  2*q.w(), -2*q.x(), -2*q.y(), 2*q.z();

      A.block<1,4>(State::VELOCITY_Z,state.getOrientationIndex()) = -gain_ * ((dr3_dq_.row(2) * R.row(2).dot(v)) + R(2,2) * (v.transpose() * dr3_dq_));
    }
  }
#endif // VELOCITY_IN_BODY_FRAME
}

SystemStatus GroundVehicleModel::getStatusFlags(const State& state)
{
  SystemStatus flags = GenericQuaternionSystemModel::getStatusFlags(state);
  flags |= STATE_VELOCITY_Z;
  flags |= STATE_POSITION_Z;
  return flags;
}

bool GroundVehicleModel::limitState(State& state)
{
  bool result = GenericQuaternionSystemModel::limitState(state);
  if (state.position().z() < min_height_) {
    state.position().z() = min_height_;
    result = false;
  }
  if (state.position().z() > max_height_) {
    state.position().z() = max_height_;
    result = false;
  }
  return result;
}

} // namespace hector_pose_estimation
