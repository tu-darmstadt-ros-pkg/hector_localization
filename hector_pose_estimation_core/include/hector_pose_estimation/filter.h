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

#ifndef HECTOR_POSE_ESTIMATION_FILTER_H
#define HECTOR_POSE_ESTIMATION_FILTER_H

#include <hector_pose_estimation/state.h>
#include <hector_pose_estimation/types.h>

#include <map>

namespace hector_pose_estimation {

class Filter {
public:
  Filter(State &state);
  virtual ~Filter();

  virtual std::string getType() const = 0;

  virtual bool reset();

  virtual const State& state() const { return state_; }
  virtual State& state() { return state_; }

  virtual void predict(State& state, const Systems& systems, double dt);
  virtual void predict(State& state, const SystemPtr& system, double dt);
  template <typename ConcreteModel> bool predict(ConcreteModel *model, State& state, double dt);

  virtual void update(State& state, const Measurements& measurements);
  virtual void update(State& state, const MeasurementPtr& measurement);
  template <typename ConcreteModel> bool update(ConcreteModel *model, State& state, const typename ConcreteModel::MeasurementVector& y, const typename ConcreteModel::NoiseVariance& R);

  // struct Predictor {};
  template <typename ConcreteModel> struct Predictor_ /* : public Predictor */ {
    Predictor_(Filter *filter, ConcreteModel *model) : model_(model) { reset(); }
    virtual bool predict(State& state, double dt) = 0;
    virtual bool reset() { init_ = true; return true; }

    template <typename Derived> typename Derived::template Predictor<ConcreteModel> *derived() { return dynamic_cast<typename Derived::template Predictor<ConcreteModel> *>(this); }
    template <typename Derived> const typename Derived::template Predictor<ConcreteModel> *derived() const { return dynamic_cast<const typename Derived::template Predictor<ConcreteModel> *>(this); }

  protected:
    ConcreteModel *model_;
    bool init_;
  };

  // class Corrector {};
  template <typename ConcreteModel> struct Corrector_ /* : public Corrector */ {
    Corrector_(Filter *filter, ConcreteModel *model) : model_(model) { reset(); }
    virtual bool correct(State& state, const typename ConcreteModel::MeasurementVector& y, const typename ConcreteModel::NoiseVariance& R) = 0;
    virtual bool reset() { init_ = true; return true; }

    template <typename Derived> typename Derived::template Corrector<ConcreteModel> *derived() { return dynamic_cast<typename Derived::template Corrector<ConcreteModel> *>(this); }
    template <typename Derived> const typename Derived::template Corrector<ConcreteModel> *derived() const { return dynamic_cast<const typename Derived::template Corrector<ConcreteModel> *>(this); }

  protected:
    ConcreteModel *model_;
    bool init_;
  };

  template <typename Derived> Derived *derived() { return dynamic_cast<Derived *>(this); }
  template <typename Derived> const Derived *derived() const { return dynamic_cast<const Derived *>(this); }

  template <typename Derived>
  struct Factory {
    Factory(Derived *filter) : filter_(filter) {}
//    template <typename ConcreteModel> boost::shared_ptr<typename Derived::template Predictor<ConcreteModel> > addPredictor(ConcreteModel *model) { return boost::make_shared<typename Derived::template Predictor<ConcreteModel> >(filter_, model); }
//    template <typename ConcreteModel> boost::shared_ptr<typename Derived::template Corrector<ConcreteModel> > addCorrector(ConcreteModel *model) { return boost::make_shared<typename Derived::template Corrector<ConcreteModel> >(filter_, model); }
    template <typename ConcreteModel> boost::shared_ptr<Predictor_<ConcreteModel> > addPredictor(ConcreteModel *model) { return boost::shared_static_cast<Predictor_<ConcreteModel> >(boost::make_shared<typename Derived::template Predictor<ConcreteModel> >(filter_, model)); }
    template <typename ConcreteModel> boost::shared_ptr<Corrector_<ConcreteModel> > addCorrector(ConcreteModel *model) { return boost::shared_static_cast<Corrector_<ConcreteModel> >(boost::make_shared<typename Derived::template Corrector<ConcreteModel> >(filter_, model)); }

  private:
    Derived *filter_;
  };
  template <typename Derived> static Factory<Derived> factory(Derived *filter) { return Factory<Derived>(filter); }

private:
  State& state_;
};

} // namespace hector_pose_estimation

#endif // HECTOR_POSE_ESTIMATION_FILTER_H
