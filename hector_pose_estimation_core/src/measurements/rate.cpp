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

#include <hector_pose_estimation/measurements/rate.h>
#include <hector_pose_estimation/filter/set_filter.h>

namespace hector_pose_estimation {

template class Measurement_<RateModel>;

RateModel::RateModel()
{
  parameters().add("stddev", stddev_, 1.0*M_PI/180.0);
}

RateModel::~RateModel() {}

bool RateModel::init(PoseEstimation &estimator, State &state)
{
  gyro_drift_ = state.addSubState<3>(this, "gyro");
  return true;
}

void RateModel::getMeasurementNoise(NoiseVariance& R, const State&, bool init)
{
  if (init) {
    R(0,0) = R(1,1) = R(2,2) = pow(stddev_, 2);
  }
}

void RateModel::getExpectedValue(MeasurementVector& y_pred, const State& state)
{
//  const State::OrientationType& q = state.getOrientation();
//  const State::RateType& rate = state.getRate();

//  y_pred(0) = (q.w()*q.w()+q.x()*q.x()-q.y()*q.y()-q.z()*q.z()) * rate.x() + (2.0*q.x()*q.y()+2.0*q.w()*q.z())                 * rate.y() + (2.0*q.x()*q.z()-2.0*q.w()*q.y())                 * rate.z();
//  y_pred(1) = (2.0*q.x()*q.y()-2.0*q.w()*q.z())                 * rate.x() + (q.w()*q.w()-q.x()*q.x()+q.y()*q.y()-q.z()*q.z()) * rate.y() + (2.0*q.y()*q.z()+2.0*q.w()*q.x())                 * rate.z();
//  y_pred(2) = (2.0*q.x()*q.z()+2.0*q.w()*q.y())                 * rate.x() + (2.0*q.y()*q.z()-2.0*q.w()*q.x())                 * rate.y() + (q.w()*q.w()-q.x()*q.x()-q.y()*q.y()+q.z()*q.z()) * rate.z();

  y_pred = state.getRate();

  if (gyro_drift_) {
    y_pred += gyro_drift_->getVector();
  }
}

void RateModel::getStateJacobian(MeasurementMatrix &C0, SubMeasurementMatrix &C1, const State &state, bool init)
{
//  const State::OrientationType& q = state.getOrientation();
//  const State::RateType& rate = state.getRate();

//  if (state.getOrientationIndex() >= 0) {
//    C(0,State::QUATERNION_W) =  2.0*q.w() * rate.x() + 2.0*q.z() * rate.y() - 2.0*q.y() * rate.z();
//    C(0,State::QUATERNION_X) =  2.0*q.x() * rate.x() + 2.0*q.y() * rate.y() + 2.0*q.z() * rate.z();
//    C(0,State::QUATERNION_Y) = -2.0*q.y() * rate.x() + 2.0*q.x() * rate.y() - 2.0*q.w() * rate.z();
//    C(0,State::QUATERNION_Z) = -2.0*q.z() * rate.x() + 2.0*q.w() * rate.y() + 2.0*q.x() * rate.z();
//    C(1,State::QUATERNION_W) = -2.0*q.z() * rate.x() + 2.0*q.w() * rate.y() + 2.0*q.x() * rate.z();
//    C(1,State::QUATERNION_X) =  2.0*q.y() * rate.x() - 2.0*q.x() * rate.y() + 2.0*q.w() * rate.z();
//    C(1,State::QUATERNION_Y) =  2.0*q.x() * rate.x() + 2.0*q.y() * rate.y() + 2.0*q.z() * rate.z();
//    C(1,State::QUATERNION_Z) = -2.0*q.w() * rate.x() - 2.0*q.z() * rate.y() + 2.0*q.y() * rate.z();
//    C(2,State::QUATERNION_W) =  2.0*q.y() * rate.x() - 2.0*q.x() * rate.y() + 2.0*q.w() * rate.z();
//    C(2,State::QUATERNION_X) =  2.0*q.z() * rate.x() - 2.0*q.w() * rate.y() - 2.0*q.x() * rate.z();
//    C(2,State::QUATERNION_Y) =  2.0*q.w() * rate.x() + 2.0*q.z() * rate.y() - 2.0*q.y() * rate.z();
//    C(2,State::QUATERNION_Z) =  2.0*q.x() * rate.x() + 2.0*q.y() * rate.y() + 2.0*q.z() * rate.z();
//  }

//  if (state.getRateIndex() >= 0) {
//    C(0,State::RATE_X) = (q.w()*q.w()+q.x()*q.x()-q.y()*q.y()-q.z()*q.z());
//    C(0,State::RATE_Y) = (2.0*q.x()*q.y()+2.0*q.w()*q.z());
//    C(0,State::RATE_Z) = (2.0*q.x()*q.z()-2.0*q.w()*q.y());
//    C(1,State::RATE_X) = (2.0*q.x()*q.y()-2.0*q.w()*q.z());
//    C(1,State::RATE_Y) = (q.w()*q.w()-q.x()*q.x()+q.y()*q.y()-q.z()*q.z());
//    C(1,State::RATE_Z) = (2.0*q.y()*q.z()+2.0*q.w()*q.x());
//    C(2,State::RATE_X) = (2.0*q.x()*q.z()+2.0*q.w()*q.y());
//    C(2,State::RATE_Y) = (2.0*q.y()*q.z()-2.0*q.w()*q.x());
//    C(2,State::RATE_Z) = (q.w()*q.w()-q.x()*q.x()-q.y()*q.y()+q.z()*q.z());
//  }

  if (!init) return;

  if (state.getRateIndex() >= 0) {
   C0(0,State::RATE_X) = 1.0;
   C0(1,State::RATE_Y) = 1.0;
   C0(2,State::RATE_Z) = 1.0;
  }

  if (gyro_drift_) {
    C1.setIdentity();
  }
}

} // namespace hector_pose_estimation
