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

#ifndef HECTOR_POSE_ESTIMATION_FILTER_EKF_INL
#define HECTOR_POSE_ESTIMATION_FILTER_EKF_INL

#include <hector_pose_estimation/filter/ekf.h>
#include <boost/utility/enable_if.hpp>

namespace hector_pose_estimation {
namespace filter {

template <typename ConcreteModel, typename Enabled>
bool EKF::PredictorImpl_<ConcreteModel, Enabled>::predict(State &state, double dt) {
  this->model_->getExpectedValue(filter_->x_pred, state, dt);
  this->model_->getStateJacobian(filter_->A, state, dt, this->init_);
  this->model_->getSystemNoise(Q, state, dt, this->init_);

  state.P() = filter_->A * state.P() * filter_->A.transpose() + Q;
  state.x() = filter_->x_pred;

  this->init_ = false;
  return true;
}

template <typename ConcreteModel>
bool EKF::PredictorImpl_<ConcreteModel, typename boost::enable_if< typename ConcreteModel::IsSubSystem >::type>::predict(State &state, double dt) {
  typename ConcreteModel::StateType& sub = this->model_->state(state);

  this->model_->getExpectedValue(x_pred, state, dt);
  this->model_->getStateJacobian(A1, A01, state, dt, this->init_);
  this->model_->getSystemNoise(Q, state, dt, this->init_);

  // Attention: do not reorder the following lines as each statement overwrites inputs of previous ones
  state.P() += (filter_->A * sub.P01() + A01 * sub.P()) * A01.transpose() + A01 * sub.P01().transpose() * filter_->A.transpose();
  sub.P01()  = (filter_->A * sub.P01() + A01 * sub.P()) * A1.transpose();
  sub.P()    = A1 * sub.P() * A1.transpose() + Q;
  sub.x()    = x_pred;

  this->init_ = false;
  return true;
}

template <typename ConcreteModel, typename Enabled>
bool EKF::CorrectorImpl_<ConcreteModel, Enabled>::correct(State &state, const typename ConcreteModel::MeasurementVector &y, const typename ConcreteModel::NoiseVariance &R) {
  this->model_->getExpectedValue(y_pred, state);
  this->model_->getStateJacobian(C, state, this->init_);

  K = state.P() * C.transpose() * (C * state.P() * C.transpose() + R).inverse();
  state.P() = state.P() - K * C * state.P();

  error = y - y_pred;
  this->model_->limitError(error);
  state.x() = state.x() + K * error;

  this->init_ = false;
  return true;
}

template <typename ConcreteModel>
bool EKF::CorrectorImpl_<ConcreteModel, typename boost::enable_if< typename ConcreteModel::HasSubSystem >::type>::correct(State &state, const typename ConcreteModel::MeasurementVector &y, const typename ConcreteModel::NoiseVariance &R) {
  ROS_WARN("EKF corrections with sub systems are currently not implemented");
  return false;
}

} // namespace filter
} // namespace hector_pose_estimation

#endif // HECTOR_POSE_ESTIMATION_FILTER_EKF_INL
