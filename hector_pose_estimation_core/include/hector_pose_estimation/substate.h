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

#ifndef HECTOR_POSE_ESTIMATION_SUBSTATE_H
#define HECTOR_POSE_ESTIMATION_SUBSTATE_H

#include <hector_pose_estimation/state.h>

namespace hector_pose_estimation {

class SubState
{
public:
  SubState(State& state) : state_(state) {}
  virtual ~SubState() {}

  virtual int getDimension() const = 0;
  virtual int getIndex() const = 0;

  virtual void reset() {}
  virtual void updated() {}
  virtual void normalize() {}

protected:
  State& state_;
  template <int _Dimension> class initializer;
};

template <int _Dimension> class SubState::initializer {
public:
  enum { Dimension = _Dimension };
  initializer(State& state) : index_(state.getDimension()) {
    IndexType newDimension = index_ + Dimension;
    state.x().conservativeResize(newDimension);
    state.P().conservativeResize(newDimension);
  }
protected:
  const IndexType index_;
};

template <> class SubState::initializer<0> {
public:
  enum { Dimension = State::Dimension };
  initializer(State&) : index_(0) {}
protected:
  const IndexType index_;
};

template <int _Dimension>
class SubState_ : public SubState, public SubState::initializer<_Dimension>
{
public:
  enum { Dimension = SubState::initializer<_Dimension>::Dimension };
  typedef ColumnVector_<Dimension> Vector;

  typedef VectorBlock<State::Vector,Dimension> VectorSegment;
  typedef Block<State::Covariance::Base,Dimension,Dimension> CovarianceBlock;
  typedef Block<State::Covariance::Base,State::Dimension,Dimension> CrossVarianceBlock;

  typedef VectorBlock<const State::Vector,Dimension> ConstVectorSegment;
  typedef Block<const State::Covariance::Base,Dimension,Dimension> ConstCovarianceBlock;
  typedef Block<const State::Covariance::Base,State::Dimension,Dimension> ConstCrossVarianceBlock;

  typedef boost::shared_ptr<SubState_<_Dimension> > Ptr;

  using SubState::initializer<_Dimension>::index_;

public:
  SubState_(State& state)
    : SubState(state)
    , SubState::initializer<_Dimension>(state)
//    , vector_(state.x().template segment<Dimension>(index_))
//    , covariance_(state.P().template block<Dimension,Dimension>(index_, index_))
//    , cross_variance_(state.P().template block<State::Dimension,Dimension>(0, index_))
  {
  }
  virtual ~SubState_() {}

  int getDimension() const { return Dimension; }
  int getIndex() const { return index_; }

  ConstVectorSegment getVector() const { return state_.getVector().segment<Dimension>(index_); }
  ConstCovarianceBlock getCovariance() const { return state_.getCovariance().block<Dimension,Dimension>(index_,index_); }
  template <int Size> VectorBlock<const typename Vector::Base, Size> getSegment(IndexType start) const { return state_.getVector().segment<Dimension>(index_ + start); }

  VectorSegment x() { return state_.x().segment<Dimension>(index_); }
  CovarianceBlock P() { return state_.P().block<Dimension,Dimension>(index_,index_); }
  CrossVarianceBlock P01() { return state_.P().block<State::Dimension,Dimension>(0,index_); }
};

extern template class SubState_<0>;

template <int SubDimension>
boost::shared_ptr<SubState_<SubDimension> > State::getSubState(const Model *model) const {
  return boost::shared_dynamic_cast<SubState_<SubDimension> >(substates_by_model_.count(model) ? substates_by_model_.at(model).lock() : SubStatePtr());
}

template <>
inline boost::shared_ptr<BaseState> State::getSubState<0>(const Model *) const {
  return base_;
}

template <int SubDimension>
boost::shared_ptr<SubState_<SubDimension> > State::getSubState(const std::string& name) const {
  return boost::shared_dynamic_cast<SubState_<SubDimension> >(substates_by_name_.count(name) ? substates_by_name_.at(name).lock() : SubStatePtr());
}

template <int SubDimension>
boost::shared_ptr<SubState_<SubDimension> > State::addSubState(const Model *model, const std::string& name) {
  boost::shared_ptr<SubState_<SubDimension> > substate;
  if (!name.empty()) substate = getSubState<SubDimension>(name);
  if (!substate) {
    substate.reset(new SubState_<SubDimension>(*this));
    substates_.push_back(boost::shared_static_cast<SubState>(substate));
    if (!name.empty()) substates_by_name_[name] = SubStateWPtr(boost::shared_static_cast<SubState>(substate));
  }
  substates_by_model_[model] = SubStateWPtr(boost::shared_static_cast<SubState>(substate));
  return substate;
}

} // namespace hector_pose_estimation

#endif // HECTOR_POSE_ESTIMATION_SUBSTATE_H
