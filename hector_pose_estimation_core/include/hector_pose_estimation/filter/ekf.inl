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

#include <ros/console.h>

namespace hector_pose_estimation {
namespace filter {

template <class ConcreteModel, typename Enabled>
bool EKF::PredictorImpl_<ConcreteModel, Enabled>::predict(double dt) {
  this->model_->getExpectedValue(x_pred, state(), dt);
  this->model_->getStateJacobian(A, state(), dt, this->init_);
  this->model_->getSystemNoise(Q, state(), dt, this->init_);

  ROS_DEBUG_STREAM("f(x)      = [" << x_pred.transpose() << "]");
  ROS_DEBUG_STREAM("Q         = [" << Q  << "]");
  ROS_DEBUG_STREAM("A = df/dx = [" << A << "]");

//  state().P0() = A * state().P() * A.transpose() + Q;
//  state().x0() = x_pred;

  this->init_ = false;
  return true;
}

template <class ConcreteModel>
bool EKF::PredictorImpl_<ConcreteModel, typename boost::enable_if< typename ConcreteModel::IsSubSystem >::type>::predict(double dt) {
  this->model_->getExpectedValue(x_pred, state(), dt);
  this->model_->getStateJacobian(A11, A01, state(), dt, this->init_);
  this->model_->getSystemNoise(Q1, state(), dt, this->init_);

  ROS_DEBUG_STREAM("f(x)          = [" << x_pred.transpose() << "]");
  ROS_DEBUG_STREAM("Q1            = [" << Q1 << "]");
  ROS_DEBUG_STREAM("A01 = df0/dx1 = [" << A01 << "]");
  ROS_DEBUG_STREAM("A11 = df1/dx1 = [" << A11 << "]");

//  // Attention: do not reorder the following lines as each statement overwrites inputs of previous ones
//  state().P() += (A * sub().P01() + A01 * sub().P()) * A01.transpose() + A01 * sub().P01().transpose() * A.transpose();
//  sub().P01()  = (A * sub().P01() + A01 * sub().P()) * A11.transpose();
//  sub().P()    = A11 * sub().P() * A11.transpose() + Q1;
//  sub().x()    = x_pred;

  this->init_ = false;
  return true;
}

template <class ConcreteModel, typename Enabled>
bool EKF::CorrectorImpl_<ConcreteModel, Enabled>::correct(const typename ConcreteModel::MeasurementVector &y, const typename ConcreteModel::NoiseVariance &R) {
  this->model_->getExpectedValue(y_pred, state());
  this->model_->getStateJacobian(C, state(), this->init_);

  ROS_DEBUG_STREAM("x_prior   = [" << state().getVector().transpose() << "]");
  ROS_DEBUG_STREAM("P_prior   = [" << state().getCovariance() << "]");
  ROS_DEBUG_STREAM("y         = [" << y.transpose() << "]");
  ROS_DEBUG_STREAM("R         = [" << R << "]");

  ROS_DEBUG_STREAM("h(x)      = [" << y_pred.transpose() << "]");
  ROS_DEBUG_STREAM("C = dh/dx = [" << C << "]");

  // S = state().P0().quadratic(C) + R;
  S  = C * state().P0().template selfadjointView<Upper>() * C.transpose() + R;
  CP = C * state().P().template topRows<State::Dimension>();
  K = CP.transpose() * S.inverse();
  state().P() -= K * CP;

  error = y - y_pred;
  this->model_->limitError(error);
  state().x() += K * error;

  ROS_DEBUG_STREAM("S             = [" << S << "]");
  ROS_DEBUG_STREAM("K             = [" << K << "]");

  ROS_DEBUG_STREAM("x_post    = [" << state().getVector().transpose() << "]");
  ROS_DEBUG_STREAM("P_post    = [" << state().getCovariance() << "]");

  this->init_ = false;
  return true;
}

template <class ConcreteModel>
bool EKF::CorrectorImpl_<ConcreteModel, typename boost::enable_if< typename ConcreteModel::HasSubSystem >::type>::correct(const typename ConcreteModel::MeasurementVector &y, const typename ConcreteModel::NoiseVariance &R) {
  this->model_->getExpectedValue(y_pred, state());
  this->model_->getStateJacobian(C0, C1, state(), this->init_);

  ROS_DEBUG_STREAM("x_prior       = [" << state().getVector().transpose() << "]");
  ROS_DEBUG_STREAM("P_prior       = [" << state().getCovariance() << "]");
  ROS_DEBUG_STREAM("y             = [" << y.transpose() << "]");
  ROS_DEBUG_STREAM("R             = [" << R << "]");

  ROS_DEBUG_STREAM("h(x_0,x_i)    = [" << y_pred.transpose() << "]");
  ROS_DEBUG_STREAM("C_0 = dh/dx_0 = [" << C0 << "]");
  ROS_DEBUG_STREAM("C_i = dh/dx_i = [" << C1 << "]");

  S =   C0 * (state().P0().template selfadjointView<Upper>() * C0.transpose() + sub().P01() * C1.transpose())
      + C1 * (sub().P01().transpose() * C0.transpose()                        + sub().P().template selfadjointView<Upper>() * C1.transpose())
      + R;
  CP = C0 * state().P().template topRows<State::Dimension>() + C1 * state().P().template middleRows<ConcreteModel::SubDimension>(sub().getIndex());
  K = CP.transpose() * S.inverse();
  state().P() -= K * CP;

  error = y - y_pred;
  this->model_->limitError(error);
  state().x() += K * error;

  ROS_DEBUG_STREAM("S             = [" << S << "]");
  ROS_DEBUG_STREAM("K             = [" << K << "]");

  ROS_DEBUG_STREAM("x_post        = [" << state().getVector().transpose() << "]");
  ROS_DEBUG_STREAM("P_post        = [" << state().getCovariance() << "]");

  this->init_ = false;
  return true;
}

} // namespace filter
} // namespace hector_pose_estimation

#endif // HECTOR_POSE_ESTIMATION_FILTER_EKF_INL
