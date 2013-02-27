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

#include <hector_pose_estimation/measurements/heading.h>
#include <hector_pose_estimation/filter/set_filter.h>

namespace hector_pose_estimation {

template class Measurement_<HeadingModel>;

HeadingModel::HeadingModel()
{
  parameters().add("stddev", stddev_, 10.0*M_PI/180.0);
}

HeadingModel::~HeadingModel() {}

void HeadingModel::getMeasurementNoise(NoiseVariance& R, const State&, bool init)
{
  if (init) {
    R(0,0) = pow(stddev_, 2);
  }
}

void HeadingModel::getExpectedValue(MeasurementVector& y_pred, const State& state)
{
  State::ConstOrientationType q(state.getOrientation());
  y_pred(0) = atan2(2*(q.x()*q.y() + q.w()*q.z()), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());
}

void HeadingModel::getStateJacobian(MeasurementMatrix& C, const State& state, bool)
{
  State::ConstOrientationType q(state.getOrientation());
  const double t1 = q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z();
  const double t2 = 2*(q.x()*q.y() + q.w()*q.z());
  const double t3 = 1.0 / (t1*t1 + t2*t2);

  if (state.getOrientationIndex() >= 0) {
    C(0,State::QUATERNION_W) = 2.0 * t3 * (q.z() * t1 - q.w() * t2);
    C(0,State::QUATERNION_X) = 2.0 * t3 * (q.y() * t1 - q.x() * t2);
    C(0,State::QUATERNION_Y) = 2.0 * t3 * (q.x() * t1 + q.y() * t2);
    C(0,State::QUATERNION_Z) = 2.0 * t3 * (q.w() * t1 + q.z() * t2);
  }
}

void HeadingModel::limitError(MeasurementVector &error) {
  error(0) = remainder(error(0), 2 * M_PI);
}

} // namespace hector_pose_estimation
