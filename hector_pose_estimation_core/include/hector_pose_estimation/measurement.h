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

#include <bfl/filter/kalmanfilter.h>
#include "measurement_model.h"
#include "measurement_update.h"
#include "queue.h"

namespace hector_pose_estimation {

class PoseEstimation;

class Measurement
{
public:
  Measurement(const std::string& name);
  virtual ~Measurement();

  virtual const std::string& getName() const { return name_; }
  void setName(const std::string& name) { name_ = name; }

  virtual MeasurementModel* getModel() const { return 0; }

  virtual bool init();
  virtual void cleanup();
  virtual void reset();
  virtual void reset(const StateVector& state) { reset(); }

  virtual SystemStatus getStatusFlags() const { return status_flags_; }
  virtual bool active(const SystemStatus& status) { return enabled(); }

  virtual ParameterList& parameters() { return parameters_; }
  virtual const ParameterList& parameters() const { return parameters_; }

  virtual void add(const MeasurementUpdate &update);
  virtual bool update(PoseEstimation &estimator, const MeasurementUpdate &update) = 0;
  virtual void process(PoseEstimation &estimator);

  bool enabled() const { return enabled_; }
  void enable() { enabled_ = true; }
  void disable() { enabled_ = false; }

  void increase_timer(double dt);
  void updated();
  bool timedout() const;

protected:
  virtual Queue& queue() = 0;
  void updateInternal(PoseEstimation &estimator, ColumnVector const& y);
  virtual void onReset() { }

protected:
  std::string name_;
  ParameterList parameters_;
  SystemStatus status_flags_;

  bool enabled_;
  double min_interval_;

  double timeout_;
  double timer_;
};

template <class ConcreteModel, class ConcreteUpdate> class Measurement_;

template <class ConcreteModel, class ConcreteUpdate = Update_<ConcreteModel> >
class Measurement_ : public Measurement {
public:
  typedef ConcreteModel Model;
  typedef ConcreteUpdate Update;
  static const unsigned int MeasurementDimension = Model::MeasurementDimension;
  typedef typename Model::MeasurementVector MeasurementVector;
  typedef typename Model::NoiseCovariance NoiseCovariance;

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
    delete model_;
  }

  virtual bool init() { return model_->init() && Measurement::init(); }
  virtual void cleanup() { model_->cleanup(); Measurement::cleanup(); }
  virtual void reset() { model_->reset(); Measurement::reset(); }
  virtual void reset(const StateVector& state) { model_->reset(state); Measurement::reset(state); }

  virtual Model* getModel() const { return model_; }
  virtual bool active(const SystemStatus& status) { return enabled() && model_->applyStatusMask(status); }

  virtual MeasurementVector const& getValue(const Update &update) { return internal::UpdateInspector<ConcreteModel,ConcreteUpdate>::getValue(update); }
  virtual NoiseCovariance const& getCovariance(const Update &update) { return update.hasCovariance() ? internal::UpdateInspector<ConcreteModel,ConcreteUpdate>::getCovariance(update) : static_cast<NoiseCovariance const&>(model_->AdditiveNoiseSigmaGet()); }
  virtual void setNoiseCovariance(NoiseCovariance const& sigma);

  virtual bool update(PoseEstimation &estimator, const MeasurementUpdate &update);

protected:
  Model *model_;

  Queue_<Update> queue_;
  virtual Queue& queue() { return queue_; }

  virtual bool beforeUpdate(PoseEstimation &estimator, const Update &update) { return true; }
  virtual void afterUpdate(PoseEstimation &estimator) { }
};

template <class ConcreteModel, class ConcreteUpdate>
bool Measurement_<ConcreteModel, ConcreteUpdate>::update(PoseEstimation &estimator, const MeasurementUpdate &update_)
{
  if (!enabled()) return false;
  if (min_interval_ > 0.0 && timer_ < min_interval_) return false;

  Update const &update = dynamic_cast<Update const &>(update_);
  if (!beforeUpdate(estimator, update)) return false;

  if (update.hasCovariance()) setNoiseCovariance(getCovariance(update));
  updateInternal(estimator, getValue(update));

  afterUpdate(estimator);
  return true;
}

template <class ConcreteModel, class ConcreteUpdate>
void Measurement_<ConcreteModel, ConcreteUpdate>::setNoiseCovariance(Measurement_<ConcreteModel, ConcreteUpdate>::NoiseCovariance const& sigma)
{
  model_->AdditiveNoiseSigmaSet(sigma);
}

} // namespace hector_pose_estimation

#endif // HECTOR_POSE_ESTIMATION_MEASUREMENT_H
