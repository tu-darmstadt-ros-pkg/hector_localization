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
  virtual bool isSubSystem() const { return false; }
  virtual bool hasSubSystem() const { return false; }

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

class SubSystemModel : public SystemModel {
public:
  SubSystemModel();
  virtual ~SubSystemModel();

  virtual bool isSubSystem() const { return true; }
};

template <class Derived, class Base = SystemModel, int _SubDimension = 0>
class SystemModel_ : public Base {
public:
  static const int StateDimension = State::Dimension;
  static const int SubDimension = _SubDimension;
  static const int Dimension = (SubDimension > 0 ? SubDimension : StateDimension);

  typedef ColumnVector_<Dimension> StateVector;
  typedef SymmetricMatrix_<Dimension> NoiseVariance;
  typedef Matrix_<Dimension,Dimension> SystemMatrix;
  typedef Matrix_<StateDimension,SubDimension> CrossSystemMatrix;

  static const int InputDimension = Input::traits<Derived>::Dimension;
  typedef typename Input::traits<Derived>::Type InputType;
  typedef typename Input::traits<Derived>::Vector InputVector;
  typedef Matrix_<Dimension,InputDimension> InputMatrix;

  struct IsSubSystem : boost::is_base_of<SubSystemModel,Derived> {};

  SystemModel_() {}
  virtual ~SystemModel_() {}

  virtual int getDimension() const { return Dimension; }

  // time discrete models should overwrite the following virtual methods if required:
  virtual void getExpectedValue(StateVector& x_pred, const State& state, double dt) {}
  virtual void getStateJacobian(SystemMatrix& A, const State& state, double dt) {}
  virtual void getInputJacobian(InputMatrix& B, const State& state, double dt) {}
  virtual void getSystemNoise(NoiseVariance& Q, const State& state, double dt) {}

  // variants with boolean init argument for time invariant systems
  virtual void getStateJacobian(SystemMatrix& A, const State& state, double dt, bool init) { getStateJacobian(A, state, dt); }
  virtual void getInputJacobian(InputMatrix& B, const State& state, double dt, bool init)  { getInputJacobian(B, state, dt); }
  virtual void getSystemNoise(NoiseVariance& Q, const State& state, double dt, bool init)  { getSystemNoise(Q, state, dt); }

  // variants for SubSystems
  virtual void getStateJacobian(SystemMatrix& Asub, CrossSystemMatrix& Across, const State& state, double dt) {}
  virtual void getStateJacobian(SystemMatrix& Asub, CrossSystemMatrix& Across, const State& state, double dt, bool init) { getStateJacobian(Asub, Across, state, dt); }
};

template <class Derived, class Base = SystemModel, int _SubDimension = 0>
class TimeDiscreteSystemModel_ : public SystemModel_<Derived,Base,_SubDimension> {
public:
  typedef typename SystemModel_<Derived,Base,_SubDimension>::StateVector StateVector;
  typedef typename SystemModel_<Derived,Base,_SubDimension>::SystemMatrix SystemMatrix;
  typedef typename SystemModel_<Derived,Base,_SubDimension>::CrossSystemMatrix CrossSystemMatrix;
  typedef typename SystemModel_<Derived,Base,_SubDimension>::InputMatrix InputMatrix;
  typedef typename SystemModel_<Derived,Base,_SubDimension>::NoiseVariance NoiseVariance;

  TimeDiscreteSystemModel_() {}
  virtual ~TimeDiscreteSystemModel_() {}

  virtual SystemModel::SystemTypeEnum getSystemType() const { return SystemModel::TIME_DISCRETE; }
};

template <class Derived, class Base = SystemModel, int _SubDimension = 0>
class TimeContinuousSystemModel_ : public SystemModel_<Derived,Base,_SubDimension> {
public:
  typedef typename SystemModel_<Derived,Base,_SubDimension>::StateVector StateVector;
  typedef typename SystemModel_<Derived,Base,_SubDimension>::SystemMatrix SystemMatrix;
  typedef typename SystemModel_<Derived,Base,_SubDimension>::CrossSystemMatrix CrossSystemMatrix;
  typedef typename SystemModel_<Derived,Base,_SubDimension>::InputMatrix InputMatrix;
  typedef typename SystemModel_<Derived,Base,_SubDimension>::NoiseVariance NoiseVariance;

  TimeContinuousSystemModel_() {}
  virtual ~TimeContinuousSystemModel_() {}

  virtual SystemModel::SystemTypeEnum getSystemType() const { return SystemModel::TIME_CONTINUOUS; }

  // time continuous models should overwrite the following virtual methods if required:
  virtual void getDerivative(StateVector& x_dot, const State& state) { internal_.x_dot.setZero(); }
  virtual void getStateJacobian(SystemMatrix& A, const State& state) {}
  virtual void getInputJacobian(InputMatrix& B, const State& state) {}
  virtual void getSystemNoise(NoiseVariance& Q, const State& state) {}

  // variants with boolean init argument for time invariant systems
  virtual void getStateJacobian(SystemMatrix& A, const State& state, bool init) { getStateJacobian(A, state); }
  virtual void getInputJacobian(InputMatrix& B, const State& state, bool init)  { getInputJacobian(B, state); }
  virtual void getSystemNoise(NoiseVariance& Q, const State& state, bool init)  { getSystemNoise(Q, state); }

  // variants for SubSystems
  virtual void getStateJacobian(SystemMatrix& Asub, CrossSystemMatrix& Across, const State& state) {}
  virtual void getStateJacobian(SystemMatrix& Asub, CrossSystemMatrix& Across, const State& state, bool init) { getStateJacobian(Asub, Across, state); }

private:
  virtual void getExpectedValue(StateVector& x_pred, const State& state, double dt) {
    getDerivative(internal_.x_dot, state);
    x_pred = state.getVector() + dt * internal_.x_dot;
  }

  virtual void getStateJacobian(SystemMatrix& A, const State& state, double dt, bool init) {
    if (init) internal_.A.setZero();
    getStateJacobian(internal_.A, state, init);
    A = SystemMatrix::Identity() + dt * internal_.A;
  }

  virtual void getStateJacobian(SystemMatrix& Asub, CrossSystemMatrix& Across, const State& state, double dt, bool init) {
    if (init) {
      internal_.A.setZero();
      internal_.Across.setZero();
    }
    getStateJacobian(internal_.A, internal_.Across, state, init);
    Asub = SystemMatrix::Identity() + dt * internal_.A;
    Across = dt * internal_.Across;
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
    CrossSystemMatrix Across;
    InputMatrix B;
    NoiseVariance Q;
  } internal_;
};

} // namespace hector_pose_estimation

#endif // HECTOR_POSE_ESTIMATION_SYSTEM_MODEL_H
