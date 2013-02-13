//=================================================================================================
// Copyright (c) 2011, Johannes Meyer, TU Darmstadt
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

#ifndef HECTOR_POSE_ESTIMATION_MEASUREMENT_H
#define HECTOR_POSE_ESTIMATION_MEASUREMENT_H

#include <hector_pose_estimation/measurement_model.h>
#include <hector_pose_estimation/measurement_update.h>
#include <hector_pose_estimation/state.h>
#include <hector_pose_estimation/queue.h>
#include <hector_pose_estimation/collection.h>

namespace hector_pose_estimation {

class Filter;

class Measurement
{
public:
  Measurement(const std::string& name);
  virtual ~Measurement();

  template <typename ConcreteModel> static boost::shared_ptr<Measurement> create(ConcreteModel *model, const std::string& name);

  virtual const std::string& getName() const { return name_; }
  void setName(const std::string& name) { name_ = name; }

  virtual MeasurementModel* getModel() const { return 0; }

  virtual bool init(PoseEstimation& estimator, State& state);
  virtual void cleanup();
  virtual void reset(State& state);

  virtual SystemStatus getStatusFlags() const { return status_flags_; }
  virtual bool active(const SystemStatus& status) { return enabled(); }

  virtual ParameterList& parameters() { return parameters_; }
  virtual const ParameterList& parameters() const { return parameters_; }

  virtual void add(const MeasurementUpdate &update);
  virtual bool update(Filter &filter, State &state, const MeasurementUpdate &update) = 0;
  virtual void process(Filter &filter, State &state);

  bool enabled() const { return enabled_; }
  void enable() { enabled_ = true; }
  void disable() { enabled_ = false; }

  void increase_timer(double dt);
  void updated();
  bool timedout() const;

protected:
  virtual Queue& queue() = 0;
  void updateInternal(Filter &filter, State &state, ColumnVector const& y, const SymmetricMatrix &R);

  virtual bool onInit(PoseEstimation& estimator, State& state) { return true; }
  virtual void onReset() { }
  virtual void onCleanup() { }

protected:
  std::string name_;
  ParameterList parameters_;
  SystemStatus status_flags_;

  bool enabled_;
  double min_interval_;

  double timeout_;
  double timer_;
};

typedef boost::shared_ptr<Measurement> MeasurementPtr;
typedef boost::weak_ptr<Measurement> MeasurementWPtr;
typedef Collection<Measurement> Measurements;

template <class ConcreteModel, class ConcreteUpdate = typename Update_<ConcreteModel>::Type >
class Measurement_ : public Measurement {
public:
  typedef ConcreteModel Model;
  typedef ConcreteUpdate Update;
  static const int MeasurementDimension = Model::MeasurementDimension;
  typedef typename Model::MeasurementVector MeasurementVector;
  typedef typename Model::NoiseVariance NoiseVariance;

  Measurement_(const std::string& name)
    : Measurement(name)
    , model_(new Model)
  {
    parameters_.add(model_->parameters());
  }

  Measurement_(Model *model, const std::string& name)
    : Measurement(name)
    , model_(model)
  {
    parameters_.add(model_->parameters());
  }

  virtual ~Measurement_() {
  }

  virtual Model* getModel() const { return model_.get(); }
  virtual bool active(const SystemStatus& status) { return enabled() && model_->applyStatusMask(status); }

  virtual MeasurementVector const& getVector(const Update &update, const State &state) {
    return internal::UpdateInspector<ConcreteModel,ConcreteUpdate>::getVector(update, state, getModel());
  }

  virtual NoiseVariance const& getVariance(const Update &update, const State &state) {
    if (update.hasVariance()) return internal::UpdateInspector<ConcreteModel,ConcreteUpdate>::getVariance(update, state, getModel());
    model_->getMeasurementNoise(R_, state, false);
    return R_;
  }

  virtual void setNoiseVariance(NoiseVariance const& R) {
    R_ = R;
  }

  virtual bool update(Filter &filter, State &state, const MeasurementUpdate &update);

protected:
  boost::shared_ptr<Model> model_;
  NoiseVariance R_;

  Queue_<Update> queue_;
  virtual Queue& queue() { return queue_; }

  virtual bool beforeUpdate(State &state, const Update &update) { return true; }
  virtual void afterUpdate(State &state) { }
};

template <typename ConcreteModel>
MeasurementPtr Measurement::create(ConcreteModel *model, const std::string& name)
{
  return MeasurementPtr(new Measurement_<ConcreteModel>(model, name));
}

template <class ConcreteModel, class ConcreteUpdate>
bool Measurement_<ConcreteModel, ConcreteUpdate>::update(Filter &filter, State &state, const MeasurementUpdate &update_)
{
  if (!enabled()) return false;
  if (min_interval_ > 0.0 && timer_ < min_interval_) return false;

  Update const &update = dynamic_cast<Update const &>(update_);
  if (!beforeUpdate(state, update)) return false;

  updateInternal(filter, state, getVector(update, state), getVariance(update, state));

  afterUpdate(state);
  return true;
}

} // namespace hector_pose_estimation

#endif // HECTOR_POSE_ESTIMATION_MEASUREMENT_H
