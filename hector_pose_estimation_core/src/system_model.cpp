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
{
}

SystemModel::~SystemModel()
{
}

void SystemModel::getPrior(State &state) {
  state.x().setZero();
  state.P().setZero();

  if (state.getOrientationIndex() >= 0) {
    state.orientation() = Eigen::Quaternion<ScalarType>::Identity().coeffs();
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

} // namespace hector_pose_estimation
