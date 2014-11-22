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

  virtual bool init(PoseEstimation& estimator, System &system, State& state) { return true; }

  enum SystemTypeEnum { UNKNOWN_SYSTEM_TYPE, TIME_DISCRETE, TIME_CONTINUOUS };
  virtual SystemTypeEnum getSystemType() const { return UNKNOWN_SYSTEM_TYPE; }

  virtual SystemStatus getStatusFlags(const State& state) { return SystemStatus(0); }
  virtual bool active(const State& state) { return true; }

  virtual void getPrior(State &state) {}

  virtual bool prepareUpdate(State& state, double dt) { return true; }
  virtual bool prepareUpdate(State& state, const Inputs& inputs, double dt) { return prepareUpdate(state, dt); }
  virtual void afterUpdate(State& state) {}
  virtual void afterUpdate(State& state, const Inputs& inputs) { afterUpdate(state); }

  virtual bool limitState(State& state) { return true; }
};

namespace traits {

  template <class Derived, int _VectorDimension = Derived::VectorDimension, int _CovarianceDimension = _VectorDimension>
  struct SystemModel {
    enum { VectorDimension = _VectorDimension };
    enum { CovarianceDimension = _CovarianceDimension };

    typedef State::Vector StateVector;
    typedef SymmetricMatrix NoiseVariance;
    typedef Matrix SystemMatrix;

    enum { InputDimension = traits::Input<Derived>::Dimension };
    typedef typename traits::Input<Derived>::Type   InputType;
    typedef typename traits::Input<Derived>::Vector InputVector;
    typedef Matrix_<State::Covariance::RowsAtCompileTime,InputDimension> InputMatrix;

    typedef SubState_<VectorDimension,CovarianceDimension> SubState;
    typedef ColumnVector_<VectorDimension> Vector;

    typedef typename SubState::VectorSegment VectorSegment;
    typedef typename SubState::CovarianceBlock CovarianceBlock;
    typedef typename SubState::CrossVarianceBlock CrossVarianceBlock;
    typedef Block<SystemMatrix,VectorDimension,SystemMatrix::ColsAtCompileTime> SystemMatrixBlock;

    typedef typename SubState::ConstVectorSegment ConstVectorSegment;
    typedef typename SubState::ConstCovarianceBlock ConstCovarianceBlock;
    typedef typename SubState::ConstCrossVarianceBlock ConstCrossVarianceBlock;
    typedef Block<const SystemMatrix,VectorDimension,SystemMatrix::ColsAtCompileTime> ConstSystemMatrixBlock;
  };

  #define SYSTEM_MODEL_TRAIT(Derived, _VectorDimension, _CovarianceDimension) \
    typedef traits::SystemModel<Derived, _VectorDimension, _CovarianceDimension> trait; \
    \
    enum { VectorDimension = trait::VectorDimension }; \
    enum { CovarianceDimension = trait::CovarianceDimension }; \
    \
    typedef typename trait::StateVector   StateVector; \
    typedef typename trait::NoiseVariance NoiseVariance; \
    typedef typename trait::SystemMatrix  SystemMatrix; \
    \
    enum { InputDimension = trait::InputDimension }; \
    typedef typename trait::InputType     InputType; \
    typedef typename trait::InputVector   InputVector; \
    typedef typename trait::InputMatrix   InputMatrix; \
    \
    typedef typename trait::SubState SubState; \
    typedef typename trait::Vector Vector; \
    \
    typedef typename trait::VectorSegment VectorSegment; \
    typedef typename trait::CovarianceBlock CovarianceBlock; \
    typedef typename trait::CrossVarianceBlock CrossVarianceBlock; \
    typedef typename trait::SystemMatrixBlock SystemMatrixBlock; \
    \
    typedef typename trait::ConstVectorSegment ConstVectorSegment; \
    typedef typename trait::ConstCovarianceBlock ConstCovarianceBlock; \
    typedef typename trait::ConstCrossVarianceBlock ConstCrossVarianceBlock; \
    typedef typename trait::ConstSystemMatrixBlock ConstSystemMatrixBlock; \

} // namespace traits

template <class Derived, int _VectorDimension = Dynamic, int _CovarianceDimension = _VectorDimension>
class SystemModel_ : public SystemModel {
public:
  SYSTEM_MODEL_TRAIT(Derived, _VectorDimension, _CovarianceDimension)
  virtual ~SystemModel_() {}

  virtual SystemModel::SystemTypeEnum getSystemType() const { return SystemModel::TIME_DISCRETE; }

  virtual void getPrior(State &state);

  Derived *derived() { return static_cast<Derived *>(this); }
  const Derived *derived() const { return static_cast<const Derived *>(this); }

  // time discrete models should overwrite the following virtual methods, as required:
protected:
  virtual void getExpectedDiff(StateVector& x_diff, double dt) {}
  virtual void getStateJacobian(SystemMatrix& A, double dt) {}
  virtual void getInputJacobian(InputMatrix& B, double dt) {}
  virtual void getSystemNoise(NoiseVariance& Q, double dt) {}
  virtual void getExpectedDiff(StateVector& x_diff, const State& state, double dt) { getExpectedDiff(x_diff, dt); }
  virtual void getStateJacobian(SystemMatrix& A, const State& state, double dt)    { getStateJacobian(A, dt); }
  virtual void getInputJacobian(InputMatrix& B, const State& state, double dt)     { getInputJacobian(B, dt); }
  virtual void getSystemNoise(NoiseVariance& Q, const State& state, double dt)     { getSystemNoise(Q, dt); }

public:
  virtual void getExpectedDiff(StateVector& x_diff, const State& state, const Inputs& inputs, double dt) { getExpectedDiff(x_diff, state, dt); }
  virtual void getStateJacobian(SystemMatrix& A, const State& state, const Inputs& inputs, double dt)    { getStateJacobian(A, state, dt); }
  virtual void getInputJacobian(InputMatrix& B, const State& state, const Inputs& inputs, double dt)     { getInputJacobian(B, state, dt); }
  virtual void getSystemNoise(NoiseVariance& Q, const State& state, const Inputs& inputs, double dt)     { getSystemNoise(Q, state, dt); }

  // variants with boolean init argument for time invariant systems
  virtual void getStateJacobian(SystemMatrix& A, const State& state, const Inputs& inputs, double dt, bool init) { getStateJacobian(A, state, inputs, dt); }
  virtual void getInputJacobian(InputMatrix& B, const State& state, const Inputs& inputs, double dt, bool init)  { getInputJacobian(B, state, inputs, dt); }
  virtual void getSystemNoise(NoiseVariance& Q, const State& state, const Inputs& inputs, double dt, bool init)  { getSystemNoise(Q, state, inputs, dt); }
};

template <class Derived, int _VectorDimension = Dynamic, int _CovarianceDimension = _VectorDimension>
class TimeContinuousSystemModel_ : public SystemModel_<Derived, _VectorDimension, _CovarianceDimension> {
public:
  SYSTEM_MODEL_TRAIT(Derived, _VectorDimension, _CovarianceDimension)

  TimeContinuousSystemModel_();
  virtual ~TimeContinuousSystemModel_();

  virtual SystemModel::SystemTypeEnum getSystemType() const { return SystemModel::TIME_CONTINUOUS; }

public:
  // the overwritten time discrete model functions are implemented in system_model.inl
  void getExpectedDiff(StateVector& x_diff, const State& state, const Inputs& inputs, double dt);
  void getStateJacobian(SystemMatrix& A, const State& state, const Inputs& inputs, double dt, bool init);
  void getInputJacobian(InputMatrix& B, const State& state, const Inputs& inputs, double dt, bool init);
  void getSystemNoise(NoiseVariance& Q, const State& state, const Inputs& inputs, double dt, bool init);

  // time continuous models should overwrite the following virtual methods, as required:
protected:
  virtual void getDerivative(StateVector& x_dot) {}
  virtual void getStateJacobian(SystemMatrix& A) {}
  virtual void getInputJacobian(InputMatrix& B) {}
  virtual void getSystemNoise(NoiseVariance& Q) {}
  virtual void getDerivative(StateVector& x_dot, const State& state) { getDerivative(x_dot); }
  virtual void getStateJacobian(SystemMatrix& A, const State& state) { getStateJacobian(A); }
  virtual void getInputJacobian(InputMatrix& B, const State& state)  { getInputJacobian(B); }
  virtual void getSystemNoise(NoiseVariance& Q, const State& state)  { getSystemNoise(Q); }

public:
  virtual void getDerivative(StateVector& x_dot, const State& state, const Inputs& inputs) { getDerivative(x_dot, state); }
  virtual void getStateJacobian(SystemMatrix& A, const State& state, const Inputs& inputs) { getStateJacobian(A, state); }
  virtual void getInputJacobian(InputMatrix& B, const State& state, const Inputs& inputs)  { getInputJacobian(B, state); }
  virtual void getSystemNoise(NoiseVariance& Q, const State& state, const Inputs& inputs)  { getSystemNoise(Q, state); }

  // variants with boolean init argument for time invariant systems
  virtual void getStateJacobian(SystemMatrix& A, const State& state, const Inputs& inputs, bool init) { getStateJacobian(A, state, inputs); }
  virtual void getInputJacobian(InputMatrix& B, const State& state, const Inputs& inputs, bool init)  { getInputJacobian(B, state, inputs); }
  virtual void getSystemNoise(NoiseVariance& Q, const State& state, const Inputs& inputs, bool init)  { getSystemNoise(Q, state, inputs); }

private:
  struct internal;
  struct internal *internal_;
};

} // namespace hector_pose_estimation

#include "system_model.inl"

#endif // HECTOR_POSE_ESTIMATION_SYSTEM_MODEL_H
