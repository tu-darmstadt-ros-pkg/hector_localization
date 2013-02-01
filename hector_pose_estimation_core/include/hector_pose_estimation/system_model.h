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

#ifndef HECTOR_POSE_ESTIMATION_SYSTEM_MODEL_H
#define HECTOR_POSE_ESTIMATION_SYSTEM_MODEL_H

#include <hector_pose_estimation/types.h>
#include <hector_pose_estimation/state.h>
#include <hector_pose_estimation/input.h>
#include <hector_pose_estimation/parameters.h>

#include <string>

namespace hector_pose_estimation {

class PoseEstimation;

class SystemModel {
public:
  SystemModel();
  virtual ~SystemModel();

  virtual int getDimension() const = 0;
  virtual int getStateDimension() const = 0;

  enum SystemTypeEnum { TIME_DISCRETE, TIME_CONTINUOUS };
  virtual SystemTypeEnum getSystemType() const = 0;

  virtual bool init(PoseEstimation& estimator, State& state) { return true; }
  virtual void cleanup() { }
  virtual void reset(State& state) { }

  ParameterList& parameters() { return parameters_; }
  const ParameterList& parameters() const { return parameters_; }

  virtual void getPrior(State &state);

  virtual bool prepareUpdate(State& state, double dt) { return true; }
  virtual void afterUpdate(State& state) {}

  virtual bool limitState(State& x) const { return true; }

private:
  ParameterList parameters_;
};

template <class Derived, int _SubDimension = 0>
class SystemModel_ : public SystemModel {
public:
  // static const int StateDimension = SystemModel::traits<Derived>::Dimension;
//  typedef typename traits<Derived>::StateVector StateVector;
//  typedef typename traits<Derived>::NoiseVariance NoiseVariance;
//  typedef typename traits<Derived>::SystemMatrix SystemMatrix;
//  typedef typename traits<Derived>::InputMatrix InputMatrix;
  static const int SubDimension   = (_SubDimension == 0) ? State::Dimension : _SubDimension;
  static const int StateDimension = (_SubDimension == 0) ? State::Dimension : (State::Dimension + SubDimension);
  typedef ColumnVector_<SubDimension> StateVector;
  typedef SymmetricMatrix_<SubDimension> NoiseVariance;
  typedef Matrix_<StateDimension,StateDimension> SystemMatrix;

  static const int InputDimension = Input::traits<Derived>::Dimension;
  typedef typename Input::traits<Derived>::Type InputType;
  typedef typename Input::traits<Derived>::Vector InputVector;
  typedef Matrix_<StateDimension,InputDimension> InputMatrix;

  SystemModel_() {}
  virtual ~SystemModel_() {}

  virtual int getDimension() const { return SubDimension; }
  virtual int getStateDimension() const { return StateDimension; }

  // time discrete models should overwrite the following virtual methods if required:
  virtual void getExpectedValue(StateVector& x_pred, const State& state, double dt) {}
  virtual void getStateJacobian(SystemMatrix& A, const State& state, double dt, bool init) {}
  virtual void getInputJacobian(InputMatrix& B, const State& state, double dt, bool init) {}
  virtual void getSystemNoise(NoiseVariance& Q, const State& state, double dt, bool init) {}
};

template <class Derived, int _SubDimension = 0>
class TimeDiscreteSystemModel_ : public SystemModel_<Derived, _SubDimension> {
public:
//  using SystemModel_<Derived, _SubDimension>::StateVector;
//  using SystemModel_<Derived, _SubDimension>::SystemMatrix;
//  using SystemModel_<Derived, _SubDimension>::InputMatrix;
//  using SystemModel_<Derived, _SubDimension>::NoiseVariance;
  typedef typename SystemModel_<Derived, _SubDimension>::StateVector StateVector;
  typedef typename SystemModel_<Derived, _SubDimension>::SystemMatrix SystemMatrix;
  typedef typename SystemModel_<Derived, _SubDimension>::InputMatrix InputMatrix;
  typedef typename SystemModel_<Derived, _SubDimension>::NoiseVariance NoiseVariance;

  TimeDiscreteSystemModel_() {}
  virtual ~TimeDiscreteSystemModel_() {}

  virtual SystemModel::SystemTypeEnum getSystemType() const { return SystemModel::TIME_DISCRETE; }
};

template <class Derived, int _SubDimension = 0>
class TimeContinuousSystemModel_ : public SystemModel_<Derived, _SubDimension> {
public:
//  using SystemModel_<Derived, _SubDimension>::StateVector;
//  using SystemModel_<Derived, _SubDimension>::SystemMatrix;
//  using SystemModel_<Derived, _SubDimension>::InputMatrix;
//  using SystemModel_<Derived, _SubDimension>::NoiseVariance;
  typedef typename SystemModel_<Derived, _SubDimension>::StateVector StateVector;
  typedef typename SystemModel_<Derived, _SubDimension>::SystemMatrix SystemMatrix;
  typedef typename SystemModel_<Derived, _SubDimension>::InputMatrix InputMatrix;
  typedef typename SystemModel_<Derived, _SubDimension>::NoiseVariance NoiseVariance;

  TimeContinuousSystemModel_() {}
  virtual ~TimeContinuousSystemModel_() {}

  virtual SystemModel::SystemTypeEnum getSystemType() const { return SystemModel::TIME_CONTINUOUS; }

  // time continuous models should overwrite the following virtual methods if required:
  virtual void getDerivative(typename SystemModel_<Derived, _SubDimension>::StateVector& x_dot, const State& state) { internal_.x_dot.setZero(); }
  virtual void getStateJacobian(SystemMatrix& A, const State& state, bool init) {}
  virtual void getInputJacobian(InputMatrix& B, const State& state, bool init) {}
  virtual void getSystemNoise(NoiseVariance& Q, const State& state, bool init) {}

  virtual void getExpectedValue(StateVector& x_pred, const State& state, double dt) {
    getDerivative(internal_.x_dot, state);
    x_pred = state.getVector() + dt * internal_.x_dot;
  }

  virtual void getStateJacobian(SystemMatrix& A, const State& state, double dt, bool init) {
    if (init) internal_.A.setZero();
    getStateJacobian(internal_.A, state, init);
    A = SystemMatrix::Identity() + dt * internal_.A;
  }

  virtual void getInputJacobian(InputMatrix& B, const State& state, double dt, bool init) {
    if (init) internal_.B.setZero();
    getInputJacobian(internal_.B, state, init);
    B = dt * internal_.B;
  }

  virtual void getSystemNoise(NoiseVariance& Q, const State& state, double dt, bool init) {
    if (init) internal_.Q.setZero();
    getSystemNoise(internal_.Q, state, init);
    Q = dt * internal_.Q;
  }

private:
  struct {
    StateVector x_dot;
    SystemMatrix A;
    InputMatrix B;
    NoiseVariance Q;
  } internal_;
};

} // namespace hector_pose_estimation

#endif // HECTOR_POSE_ESTIMATION_SYSTEM_MODEL_H
