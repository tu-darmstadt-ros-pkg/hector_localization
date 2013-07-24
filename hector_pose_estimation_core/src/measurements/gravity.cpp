//=================================================================================================
// Copyright (c) 2011, Johannes Meyer, TU Darmstadt
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

#include <hector_pose_estimation/measurements/gravity.h>
#include <hector_pose_estimation/pose_estimation.h>
#include <hector_pose_estimation/filter/set_filter.h>

namespace hector_pose_estimation {

template class Measurement_<GravityModel>;

GravityModel::GravityModel()
  : gravity_(0.0)
{
  parameters().add("stddev", stddev_, 10.0);
}

GravityModel::~GravityModel() {}

bool GravityModel::init(PoseEstimation &estimator, State &state) {
  setGravity(estimator.parameters().getAs<double>("gravity_magnitude"));
  return true;
}

void GravityModel::getMeasurementNoise(NoiseVariance& R, const State&, bool init)
{
  if (init) {
    R(0,0) = R(1,1) = R(2,2) = pow(stddev_, 2);
  }
}

void GravityModel::getExpectedValue(MeasurementVector& y_pred, const State& state)
{
  State::ConstOrientationType q(state.getOrientation());

  // y = q * [0 0 1] * q';
  y_pred(0) = -gravity_.z() * (2*q.x()*q.z() - 2*q.w()*q.y());
  y_pred(1) = -gravity_.z() * (2*q.w()*q.x() + 2*q.y()*q.z());
  y_pred(2) = -gravity_.z() * (q.w()*q.w() - q.x()*q.x() - q.y()*q.y() + q.z()*q.z());
}

void GravityModel::getStateJacobian(MeasurementMatrix& C, const State& state, bool)
{
  State::ConstOrientationType q(state.getOrientation());

  if (state.getOrientationIndex() >= 0) {
    C(0,State::QUATERNION_W) =  gravity_.z()*2*q.y();
    C(0,State::QUATERNION_X) = -gravity_.z()*2*q.z();
    C(0,State::QUATERNION_Y) =  gravity_.z()*2*q.w();
    C(0,State::QUATERNION_Z) = -gravity_.z()*2*q.x();
    C(1,State::QUATERNION_W) = -gravity_.z()*2*q.x();
    C(1,State::QUATERNION_X) = -gravity_.z()*2*q.w();
    C(1,State::QUATERNION_Y) = -gravity_.z()*2*q.z();
    C(1,State::QUATERNION_Z) = -gravity_.z()*2*q.y();
    C(2,State::QUATERNION_W) = -gravity_.z()*2*q.w();
    C(2,State::QUATERNION_X) =  gravity_.z()*2*q.x();
    C(2,State::QUATERNION_Y) =  gravity_.z()*2*q.y();
    C(2,State::QUATERNION_Z) = -gravity_.z()*2*q.z();
  }
}

} // namespace hector_pose_estimation
