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

#include <ros/assert.h>

#include <string>

namespace hector_pose_estimation {

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

  virtual SystemStatus getStatusFlags(const State& state) const { return SystemStatus(0); }
  virtual bool applyStatusMask(const SystemStatus& status) { return true; }

  ParameterList& parameters() { return parameters_; }
  const ParameterList& parameters() const { return parameters_; }

  virtual void getPrior(State &state);

  virtual bool prepareUpdate(State& state, double dt) { return true; }
  virtual void afterUpdate(State& state) {}

  virtual bool limitState(State& x) const { return true; }

private:
  ParameterList parameters_;
};

template <int _SubDimension>
class SubSystemModel_ : public SystemModel {
public:
  enum { SubDimension = _SubDimension };

  SubSystemModel_() {}
  virtual ~SubSystemModel_() {}

  virtual bool isSubSystem() const { return true; }
};

namespace traits {

  template <int _SubDimension>
  struct SystemModel {
    enum { StateDimension = State::Dimension };
    enum { SubDimension = _SubDimension };
    enum { Dimension = _SubDimension };

    typedef SubSystemModel_<SubDimension> Base;

    typedef SubState_<Dimension> StateType;
//    typedef typename StateType::Vector StateVector;
//    typedef typename StateType::Covariance StateVariance;
    typedef ColumnVector_<Dimension> StateVector;
    typedef SymmetricMatrix_<Dimension> StateVariance;
    typedef SymmetricMatrix_<Dimension> NoiseVariance;
    typedef Matrix_<Dimension,Dimension> SystemMatrix;

    typedef boost::true_type IsSubSystem;
    typedef typename SubState_<SubDimension>::type SubState;
    typedef typename SubState::Ptr SubStatePtr;
    typedef ColumnVector_<SubDimension> SubStateVector;
    typedef SymmetricMatrix_<SubDimension> SubStateVariance;
    typedef Matrix_<StateDimension,SubDimension> CrossSystemMatrix;
  };

  template <>
  struct SystemModel<0> {
    enum { StateDimension = State::Dimension };
    enum { SubDimension = 0 };
    enum { Dimension = State::Dimension };

    typedef hector_pose_estimation::SystemModel Base;

    typedef State StateType;
//    typedef typename StateType::Vector StateVector;
//    typedef typename StateType::Covariance StateVariance;
    typedef ColumnVector_<Dimension> StateVector;
    typedef SymmetricMatrix_<Dimension> StateVariance;
    typedef Matrix_<Dimension,Dimension> NoiseVariance;
    typedef Matrix_<Dimension,Dimension> SystemMatrix;

    typedef boost::false_type IsSubSystem;
    typedef typename SubState_<SubDimension>::type SubState;
    typedef typename SubState::Ptr SubStatePtr;
    typedef ColumnVector_<SubDimension> SubStateVector;
    typedef SymmetricMatrix_<SubDimension> SubStateVariance;
    typedef Matrix_<StateDimension,SubDimension> CrossSystemMatrix;
  };

  template <class Derived, typename Enabled = void>
  struct StateInspector {
    static State& state(const Derived *model, State &state) { return state; }
    static const State& state(const Derived *model, const State &state) { return state; }
  };

  template <class Derived>
  struct StateInspector<Derived, typename boost::enable_if< typename Derived::IsSubSystem >::type> {
  };

} // namespace traits

template <class Derived, int _SubDimension>
class SystemModel_ : public traits::template SystemModel<_SubDimension>::Base {
public:
  typedef traits::SystemModel<_SubDimension> trait;
  enum { Dimension = trait::Dimension };

  enum { StateDimension = trait::StateDimension };
  typedef typename trait::StateType         StateType;
  typedef typename trait::StateVector       StateVector;
  typedef typename trait::StateVariance     StateVariance;
  typedef typename trait::NoiseVariance     NoiseVariance;
  typedef typename trait::SystemMatrix      SystemMatrix;

  enum { InputDimension = traits::Input<Derived>::Dimension };
  typedef typename traits::Input<Derived>::Type InputType;
  typedef typename traits::Input<Derived>::Vector InputVector;
  typedef Matrix_<Dimension,InputDimension> InputMatrix;

  typedef typename trait::IsSubSystem       IsSubSystem;
  enum { SubDimension = _SubDimension };
  typedef typename trait::SubState          SubState;
  typedef typename trait::SubStatePtr       SubStatePtr;
  typedef typename trait::SubStateVector    SubStateVector;
  typedef typename trait::SubStateVariance  SubStateVariance;
  typedef typename trait::CrossSystemMatrix CrossSystemMatrix;

  SystemModel_() {}
  virtual ~SystemModel_() {}

  int getDimension() const { return trait::Dimension; }

  Derived *derived() { return static_cast<Derived *>(this); }
  const Derived *derived() const { return static_cast<const Derived *>(this); }

  StateType& state(State& state) const { return traits::StateInspector<Derived>::state(derived(), state); }
  const StateType& state(const State& state) const { return traits::StateInspector<Derived>::state(derived(), state); }

  // time discrete models should overwrite the following virtual methods if required:
  virtual void getExpectedValue(StateVector& x_pred, const State& state, double dt) { x_pred = this->state(state).getVector(); }
  virtual void getStateJacobian(SystemMatrix& A, const State& state, double dt) {}
  virtual void getInputJacobian(InputMatrix& B, const State& state, double dt) {}
  virtual void getSystemNoise(NoiseVariance& Q, const State& state, double dt) {}

  // variants with boolean init argument for time invariant systems
  virtual void getStateJacobian(SystemMatrix& A, const State& state, double dt, bool init) { getStateJacobian(A, state, dt); }
  virtual void getInputJacobian(InputMatrix& B, const State& state, double dt, bool init)  { getInputJacobian(B, state, dt); }
  virtual void getSystemNoise(NoiseVariance& Q, const State& state, double dt, bool init)  { getSystemNoise(Q, state, dt); }

  // variants for SubSystems
  virtual void getStateJacobian(SystemMatrix& A1, CrossSystemMatrix& A01, const State& state, double dt) {}
  virtual void getStateJacobian(SystemMatrix& A1, CrossSystemMatrix& A01, const State& state, double dt, bool init) { getStateJacobian(A1, A01, state, dt); }
};

template <class Derived, int _SubDimension = 0>
class TimeDiscreteSystemModel_ : public SystemModel_<Derived, _SubDimension> {
public:
  // unfortunately we have to repeat the typedefs here as they are not inherited from base class SystemModel_
  typedef traits::SystemModel<_SubDimension> trait;
  enum { Dimension = trait::Dimension };

  enum { StateDimension = trait::StateDimension };
  typedef typename trait::StateType         StateType;
  typedef typename trait::StateVector       StateVector;
  typedef typename trait::StateVariance     StateVariance;
  typedef typename trait::NoiseVariance     NoiseVariance;
  typedef typename trait::SystemMatrix      SystemMatrix;

  enum { InputDimension = traits::Input<Derived>::Dimension };
  typedef typename traits::Input<Derived>::Type InputType;
  typedef typename traits::Input<Derived>::Vector InputVector;
  typedef Matrix_<Dimension,InputDimension> InputMatrix;

  typedef typename trait::IsSubSystem       IsSubSystem;
  enum { SubDimension = _SubDimension };
  typedef typename trait::SubState          SubState;
  typedef typename trait::SubStatePtr       SubStatePtr;
  typedef typename trait::SubStateVector    SubStateVector;
  typedef typename trait::SubStateVariance  SubStateVariance;
  typedef typename trait::CrossSystemMatrix CrossSystemMatrix;

  TimeDiscreteSystemModel_() {}
  virtual ~TimeDiscreteSystemModel_() {}

  virtual SystemModel::SystemTypeEnum getSystemType() const { return SystemModel::TIME_DISCRETE; }
};

template <class Derived, int _SubDimension = 0>
class TimeContinuousSystemModel_ : public SystemModel_<Derived, _SubDimension> {
public:
  // unfortunately we have to repeat the typedefs here as they are not inherited from base class SystemModel_
  typedef traits::SystemModel<_SubDimension> trait;
  enum { Dimension = trait::Dimension };

  enum { StateDimension = trait::StateDimension };
  typedef typename trait::StateType         StateType;
  typedef typename trait::StateVector       StateVector;
  typedef typename trait::StateVariance     StateVariance;
  typedef typename trait::NoiseVariance     NoiseVariance;
  typedef typename trait::SystemMatrix      SystemMatrix;

  enum { InputDimension = traits::Input<Derived>::Dimension };
  typedef typename traits::Input<Derived>::Type InputType;
  typedef typename traits::Input<Derived>::Vector InputVector;
  typedef Matrix_<Dimension,InputDimension> InputMatrix;

  typedef typename trait::IsSubSystem       IsSubSystem;
  enum { SubDimension = _SubDimension };
  typedef typename trait::SubState          SubState;
  typedef typename trait::SubStatePtr       SubStatePtr;
  typedef typename trait::SubStateVector    SubStateVector;
  typedef typename trait::SubStateVariance  SubStateVariance;
  typedef typename trait::CrossSystemMatrix CrossSystemMatrix;

  TimeContinuousSystemModel_() {}
//  {
//    internal_.x_dot.resize(Dimension);
//  }
  virtual ~TimeContinuousSystemModel_() {}

  virtual SystemModel::SystemTypeEnum getSystemType() const { return SystemModel::TIME_CONTINUOUS; }

  // time continuous models should overwrite the following virtual methods if required:
  virtual void getDerivative(StateVector& x_dot, const State& state) { x_dot.setZero(); }
  virtual void getStateJacobian(SystemMatrix& A, const State& state) {}
  virtual void getInputJacobian(InputMatrix& B, const State& state) {}
  virtual void getSystemNoise(NoiseVariance& Q, const State& state) {}

  // variants with boolean init argument for time invariant systems
  virtual void getStateJacobian(SystemMatrix& A, const State& state, bool init) { getStateJacobian(A, state); }
  virtual void getInputJacobian(InputMatrix& B, const State& state, bool init)  { getInputJacobian(B, state); }
  virtual void getSystemNoise(NoiseVariance& Q, const State& state, bool init)  { getSystemNoise(Q, state); }

  // variants for SubSystems
  virtual void getStateJacobian(SystemMatrix& A1, CrossSystemMatrix& A01, const State& state) {}
  virtual void getStateJacobian(SystemMatrix& A1, CrossSystemMatrix& A01, const State& state, bool init) { getStateJacobian(A1, A01, state); }

  void getExpectedValue(StateVector& x_pred, const State& state, double dt) {
    getDerivative(internal_.x_dot, state);
    x_pred = this->state(state).getVector() + dt * internal_.x_dot;
  }

  void getStateJacobian(SystemMatrix& A, const State& state, double dt, bool init) {
    if (init) internal_.A.setZero();
    getStateJacobian(internal_.A, state, init);
    A = SystemMatrix::Identity() + dt * internal_.A;
  }

  void getStateJacobian(SystemMatrix& A1, CrossSystemMatrix& A01, const State& state, double dt, bool init) {
    if (init) {
      internal_.A.setZero();
      internal_.A01.setZero();
    }
    getStateJacobian(internal_.A, internal_.A01, state, init);
    A1 = SystemMatrix::Identity() + dt * internal_.A;
    A01 = dt * internal_.A01;
  }

  void getInputJacobian(InputMatrix& B, const State& state, double dt, bool init) {
    if (init) internal_.B.setZero();
    getInputJacobian(internal_.B, state, init);
    B = dt * internal_.B;
  }

  void getSystemNoise(NoiseVariance& Q, const State& state, double dt, bool init) {
    if (init) internal_.Q.setZero();
    getSystemNoise(internal_.Q, state, init);
    Q = dt * internal_.Q;
  }

  virtual void reset(State &state) {
    this->internal_ = internal();
  }

private:
  struct internal {
    StateVector x_dot;
    SystemMatrix A;
    CrossSystemMatrix A01;
    InputMatrix B;
    NoiseVariance Q;
  } internal_;
};

} // namespace hector_pose_estimation

#endif // HECTOR_POSE_ESTIMATION_SYSTEM_MODEL_H
