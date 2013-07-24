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

#include <hector_pose_estimation/model.h>
#include <hector_pose_estimation/substate.h>
#include <hector_pose_estimation/input.h>

namespace hector_pose_estimation {

class SystemModel : public Model {
public:
  virtual ~SystemModel() {}

  virtual int getDimension() const = 0;
  virtual bool isSubSystem() const { return false; }
  virtual IndexType getStateIndex(const State&) const { return 0; }

  enum SystemTypeEnum { UNKNOWN_SYSTEM_TYPE, TIME_DISCRETE, TIME_CONTINUOUS };
  virtual SystemTypeEnum getSystemType() const { return UNKNOWN_SYSTEM_TYPE; }

  virtual SystemStatus getStatusFlags(const State& state) { return SystemStatus(0); }
  virtual bool active(const State& state) { return true; }

  virtual void getPrior(State &state) {}

  virtual bool prepareUpdate(State& state, double dt) { return true; }
  virtual void afterUpdate(State& state) {}

  virtual bool limitState(State& state) { return true; }
};

template <int _SubDimension>
class SubSystemModel_ : public SystemModel {
public:
  enum { SubDimension = _SubDimension };

  virtual ~SubSystemModel_() {}

  virtual bool isSubSystem() const { return true; }
  virtual IndexType getStateIndex(const State& state) const { return state.getSubState<SubDimension>(this)->getIndex(); }
};

template <>
class SubSystemModel_<0> : public SystemModel {};

namespace traits {

  template <int _SubDimension>
  struct SystemModel {
    typedef SubState_<_SubDimension> StateType;
    enum { BaseDimension = State::Dimension };
    enum { SubDimension = _SubDimension };
    enum { Dimension = StateType::Dimension };

    typedef typename StateType::Vector StateVector;
    typedef typename StateType::VectorSegment StateVectorSegment;
    typedef typename StateType::CovarianceBlock StateCovarianceBlock;
    typedef typename StateType::ConstVectorSegment ConstStateVectorSegment;
    typedef typename StateType::ConstCovarianceBlock ConstStateCovarianceBlock;

    typedef SymmetricMatrix_<Dimension> NoiseVariance;
    typedef Block<typename State::Covariance::Base,Dimension,Dimension> NoiseVarianceBlock;
    typedef Block<const typename State::Covariance::Base,Dimension,Dimension> ConstNoiseVarianceBlock;

    typedef Matrix_<Dimension,Dimension> SystemMatrix;
    typedef Block<Matrix,Dimension,Dimension> SystemMatrixBlock;
    typedef Block<const Matrix,Dimension,Dimension> ConstSystemMatrixBlock;

    typedef boost::integral_constant<bool,(_SubDimension > 0)> IsSubSystem;
    typedef SubState_<SubDimension> SubState;
    typedef typename SubState::Ptr SubStatePtr;
    typedef Matrix_<BaseDimension,SubDimension> CrossSystemMatrix;
    typedef Block<Matrix,BaseDimension,SubDimension> CrossSystemMatrixBlock;
    typedef Block<const Matrix,BaseDimension,SubDimension> ConstCrossSystemMatrixBlock;
  };

  #define SYSTEM_MODEL_TRAIT(Derived, _SubDimension) \
    typedef traits::SystemModel<_SubDimension> trait; \
    typedef typename trait::StateType StateType; \
    enum { BaseDimension = trait::BaseDimension }; \
    enum { SubDimension = trait::SubDimension }; \
    enum { Dimension = trait::Dimension }; \
    \
    typedef typename trait::StateVector               StateVector; \
    typedef typename trait::StateVectorSegment        StateVectorSegment; \
    typedef typename trait::StateCovarianceBlock      StateCovarianceBlock; \
    typedef typename trait::ConstStateVectorSegment   ConstStateVectorSegment; \
    typedef typename trait::ConstStateCovarianceBlock ConstStateCovarianceBlock; \
    \
    typedef typename trait::NoiseVariance           NoiseVariance; \
    typedef typename trait::NoiseVarianceBlock      NoiseVarianceBlock; \
    typedef typename trait::ConstNoiseVarianceBlock ConstNoiseVarianceBlock; \
    \
    typedef typename trait::SystemMatrix           SystemMatrix; \
    typedef typename trait::SystemMatrixBlock      SystemMatrixBlock; \
    typedef typename trait::ConstSystemMatrixBlock ConstSystemMatrixBlock; \
    \
    enum { InputDimension = traits::Input<Derived>::Dimension }; \
    typedef typename traits::Input<Derived>::Type   InputType; \
    typedef typename traits::Input<Derived>::Vector InputVector; \
    typedef Matrix_<Dynamic,InputDimension>         InputMatrix; \
    typedef Block<typename InputMatrix::Base,Dimension,InputDimension>       InputMatrixBlock; \
    typedef Block<const typename InputMatrix::Base,Dimension,InputDimension> ConstInputMatrixBlock; \
    \
    typedef typename trait::IsSubSystem IsSubSystem; \
    typedef typename trait::SubState                    SubState; \
    typedef typename trait::SubStatePtr                 SubStatePtr; \
    typedef typename trait::CrossSystemMatrix           CrossSystemMatrix; \
    typedef typename trait::CrossSystemMatrixBlock      CrossSystemMatrixBlock; \
    typedef typename trait::ConstCrossSystemMatrixBlock ConstCrossSystemMatrixBlock; \

} // namespace traits

template <class Derived, int _SubDimension>
class SystemModel_ : public SubSystemModel_<_SubDimension> {
public:
  SYSTEM_MODEL_TRAIT(Derived, _SubDimension)
  virtual ~SystemModel_() {}

  int getDimension() const { return trait::Dimension; }
  virtual SystemModel::SystemTypeEnum getSystemType() const { return SystemModel::TIME_DISCRETE; }

  virtual void getPrior(State &state);

  Derived *derived() { return static_cast<Derived *>(this); }
  const Derived *derived() const { return static_cast<const Derived *>(this); }

  SubState& sub(State& state) const { return *state.getSubState<SubDimension>(this); }
  const SubState& sub(const State& state) const { return *state.getSubState<SubDimension>(this); }

  // time discrete models should overwrite the following virtual methods if required:
  virtual void getExpectedValue(StateVectorSegment& x_pred, const State& state, double dt) { x_pred = this->sub(state).getVector(); }
  virtual void getStateJacobian(SystemMatrixBlock& A, const State& state, double dt) {}
  virtual void getInputJacobian(InputMatrixBlock& B, const State& state, double dt) {}
  virtual void getSystemNoise(NoiseVarianceBlock& Q, const State& state, double dt) {}

  // variants with boolean init argument for time invariant systems
  virtual void getStateJacobian(SystemMatrixBlock& A, const State& state, double dt, bool init) { getStateJacobian(A, state, dt); }
  virtual void getInputJacobian(InputMatrixBlock& B, const State& state, double dt, bool init)  { getInputJacobian(B, state, dt); }
  virtual void getSystemNoise(NoiseVarianceBlock& Q, const State& state, double dt, bool init)  { getSystemNoise(Q, state, dt); }

  // variants for SubSystems
  virtual void getStateJacobian(SystemMatrixBlock& A1, CrossSystemMatrixBlock& A01, const State& state, double dt) {}
  virtual void getStateJacobian(SystemMatrixBlock& A1, CrossSystemMatrixBlock& A01, const State& state, double dt, bool init) { getStateJacobian(A1, A01, state, dt); }
};

template <class Derived, int _SubDimension = 0>
class TimeDiscreteSystemModel_ : public SystemModel_<Derived, _SubDimension> {
public:
  SYSTEM_MODEL_TRAIT(Derived, _SubDimension)
  virtual ~TimeDiscreteSystemModel_() {}
};

template <class Derived, int _SubDimension = 0>
class TimeContinuousSystemModel_ : public SystemModel_<Derived, _SubDimension> {
public:
  SYSTEM_MODEL_TRAIT(Derived, _SubDimension)

  TimeContinuousSystemModel_();
  virtual ~TimeContinuousSystemModel_();

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

  // the time discrete model functions are implemented in system_model.inl
  void getExpectedValue(StateVectorSegment& x_pred, const State& state, double dt);
  void getStateJacobian(SystemMatrixBlock& A, const State& state, double dt, bool init);
  void getStateJacobian(SystemMatrixBlock& A1, CrossSystemMatrixBlock& A01, const State& state, double dt, bool init);
  void getInputJacobian(InputMatrixBlock& B, const State& state, double dt, bool init);
  void getSystemNoise(NoiseVarianceBlock& Q, const State& state, double dt, bool init);

private:
  struct internal;
  struct internal *internal_;
};

} // namespace hector_pose_estimation

#include "system_model.inl"

#endif // HECTOR_POSE_ESTIMATION_SYSTEM_MODEL_H
