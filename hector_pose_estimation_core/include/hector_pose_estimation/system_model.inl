//=================================================================================================
// Copyright (c) 2013, Johannes Meyer, TU Darmstadt
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

#ifndef HECTOR_POSE_ESTIMATION_SYSTEM_MODEL_INL
#define HECTOR_POSE_ESTIMATION_SYSTEM_MODEL_INL

#include <hector_pose_estimation/system_model.h>

namespace hector_pose_estimation {

template <class ConcreteModel, int _SubDimension>
void SystemModel_<ConcreteModel, _SubDimension>::getPrior(State &state)
{
  const double dt = 10.0;
  NoiseVarianceBlock P0(sub(state).P());
  getSystemNoise(P0, state, dt, true);
}

template <class ConcreteModel, int _SubDimension>
struct TimeContinuousSystemModel_<ConcreteModel, _SubDimension>::internal {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  StateVector x_dot;
  SystemMatrix A;
  CrossSystemMatrix A01;
  InputMatrix B;
  NoiseVariance Q;
};

template <class ConcreteModel, int _SubDimension>
TimeContinuousSystemModel_<ConcreteModel, _SubDimension>::TimeContinuousSystemModel_()
  : internal_(new internal)
{}

template <class ConcreteModel, int _SubDimension>
TimeContinuousSystemModel_<ConcreteModel, _SubDimension>::~TimeContinuousSystemModel_()
{
  delete internal_;
}

template <class ConcreteModel, int _SubDimension>
void TimeContinuousSystemModel_<ConcreteModel, _SubDimension>::getExpectedValue(StateVectorSegment& x_pred, const State& state, double dt) {
  getDerivative(internal_->x_dot, state);
  x_pred = this->sub(state).getVector() + dt * internal_->x_dot;
}

template <class ConcreteModel, int _SubDimension>
void TimeContinuousSystemModel_<ConcreteModel, _SubDimension>::getStateJacobian(SystemMatrixBlock& A, const State& state, double dt, bool init) {
  if (init) internal_->A.setZero();
  getStateJacobian(internal_->A, state, init);
  A = SystemMatrix::Identity() + dt * internal_->A;
}

template <class ConcreteModel, int _SubDimension>
void TimeContinuousSystemModel_<ConcreteModel, _SubDimension>::getStateJacobian(SystemMatrixBlock& A1, CrossSystemMatrixBlock& A01, const State& state, double dt, bool init) {
  if (init) {
    internal_->A.setZero();
    internal_->A01.setZero();
  }
  getStateJacobian(internal_->A, internal_->A01, state, init);
  A1 = SystemMatrix::Identity() + dt * internal_->A;
  A01 = dt * internal_->A01;
}

template <class ConcreteModel, int _SubDimension>
void TimeContinuousSystemModel_<ConcreteModel, _SubDimension>::getInputJacobian(InputMatrixBlock& B, const State& state, double dt, bool init) {
  if (init) internal_->B.setZero();
  getInputJacobian(internal_->B, state, init);
  B = dt * internal_->B;
}

template <class ConcreteModel, int _SubDimension>
void TimeContinuousSystemModel_<ConcreteModel, _SubDimension>::getSystemNoise(NoiseVarianceBlock& Q, const State& state, double dt, bool init) {
  if (init) internal_->Q.setZero();
  getSystemNoise(internal_->Q, state, init);
  Q = dt * internal_->Q;
}

} // namespace hector_pose_estimation

#endif // HECTOR_POSE_ESTIMATION_SYSTEM_MODEL_INL
