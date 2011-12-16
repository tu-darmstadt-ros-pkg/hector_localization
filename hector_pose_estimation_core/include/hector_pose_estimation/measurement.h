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

  virtual SystemStatus getStatusFlags() const { return status_flags_; }
  virtual bool active(const SystemStatus& status) { return enabled(); }

  virtual ParameterList& parameters() { return parameters_; }
  virtual const ParameterList& parameters() const { return parameters_; }

  virtual void add(const MeasurementUpdate &update);
  virtual void update(BFL::KalmanFilter &filter, const SystemStatus& status, const MeasurementUpdate &update) = 0;
  virtual void process(BFL::KalmanFilter &filter, const SystemStatus& status);

  bool enabled() const { return enabled_; }
  void enable() { enabled_ = true; }
  void disable() { enabled_ = false; }

  void increase_timer(double dt);
  void updated();
  bool timedout() const;

protected:
  virtual Queue& queue() = 0;

protected:
  std::string name_;
  ParameterList parameters_;

  bool enabled_;
  SystemStatus status_flags_;

  unsigned int timeout_;
  unsigned int timer_;
};

template <class ConcreteMeasurementModel>
class Measurement_ : public Measurement {
public:
  typedef ConcreteMeasurementModel Model;
  typedef Update_<Measurement_<ConcreteMeasurementModel> > Update;

  Measurement_(Model *model, const std::string& name)
    : Measurement(name)
    , model_(model)
  {}

  virtual ~Measurement_() {
    delete model_;
  }

  virtual Model* getModel() const { return model_; }
  virtual bool active(const SystemStatus& status) { return enabled() && getModel()->applyStatusMask(status); }

  virtual ParameterList& parameters() { return getModel()->parameters(); }
  virtual const ParameterList& parameters() const { return getModel()->parameters(); }

  virtual void update(BFL::KalmanFilter &filter, const SystemStatus& status, const MeasurementUpdate &update);
  virtual void update(BFL::KalmanFilter &filter, const SystemStatus& status, typename Model::MeasurementVector const& y);
  virtual void update(BFL::KalmanFilter &filter, const SystemStatus& status, typename Model::MeasurementVector const& y, typename Model::NoiseCovariance const& sigma);

private:
  Model *model_;

protected:
  typedef Queue_<Update> Queue;
  Queue queue_;
  virtual Queue& queue() { return queue_; }
};

template <class ConcreteMeasurementModel>
void Measurement_<ConcreteMeasurementModel>::update(BFL::KalmanFilter &filter, const SystemStatus& status, const MeasurementUpdate &update_)
{
  Update const &update = static_cast<Update const &>(update_);
  if (update.getSigma())
    this->update(filter, status, update.getY(), *(update.getSigma()));
  else
    this->update(filter, status, update.getY());
}

template <class ConcreteMeasurementModel>
void Measurement_<ConcreteMeasurementModel>::update(BFL::KalmanFilter &filter, const SystemStatus& status, typename Measurement_<ConcreteMeasurementModel>::Model::MeasurementVector const& y)
{
  filter.Update(getModel(), y);
  updated();
}

template <class ConcreteMeasurementModel>
void Measurement_<ConcreteMeasurementModel>::update(BFL::KalmanFilter &filter, const SystemStatus& status, typename Measurement_<ConcreteMeasurementModel>::Model::MeasurementVector const& y, typename Measurement_<ConcreteMeasurementModel>::Model::NoiseCovariance const& sigma)
{
  if (&sigma) dynamic_cast<BFL::AnalyticConditionalGaussianAdditiveNoise *>(model_)->AdditiveNoiseSigmaSet(sigma);
  this->update(filter, status, y);
}

} // namespace hector_pose_estimation

#endif // HECTOR_POSE_ESTIMATION_MEASUREMENT_H
