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
#include <hector_pose_estimation/types.h>
#include <hector_pose_estimation/state.h>
#include <hector_pose_estimation/queue.h>
#include <hector_pose_estimation/filter.h>

#include <ros/console.h>

namespace hector_pose_estimation {

template <typename ConcreteModel> class Measurement_;

class Measurement
{
public:
  Measurement(const std::string& name);
  virtual ~Measurement();

  template <typename ConcreteModel> static boost::shared_ptr<Measurement_<ConcreteModel> > create(ConcreteModel *model, const std::string& name);

  virtual const std::string& getName() const { return name_; }
  void setName(const std::string& name) { name_ = name; }

  virtual MeasurementModel* getModel() const { return 0; }
  virtual int getDimension() const { return 0; }

  virtual void setFilter(Filter *filter) {}

  virtual bool init(PoseEstimation& estimator, Filter& filter, State& state);
  virtual void cleanup();
  virtual void reset(State& state);

  virtual SystemStatus getStatusFlags() const { return status_flags_; }
  virtual bool active(const SystemStatus& status) { return enabled(); }

  virtual ParameterList& parameters() { return parameters_; }
  virtual const ParameterList& parameters() const { return parameters_; }

  virtual void add(const MeasurementUpdate &update);
  virtual void process(State &state);
  virtual bool update(State &state, const MeasurementUpdate &update);

  bool enabled() const { return enabled_; }
  void enable() { enabled_ = true; }
  void disable() { enabled_ = false; }

  void increase_timer(double dt);
  void updated();
  bool timedout() const;

protected:
  virtual Queue& queue() = 0;
  virtual bool updateImpl(State &state, const MeasurementUpdate &update) { return false; }

  virtual bool onInit(PoseEstimation& estimator, State& state) { return true; } // currently unsed...
  virtual void onReset() { }
  virtual void onCleanup() { } // currently unsed...

protected:
  std::string name_;
  ParameterList parameters_;
  SystemStatus status_flags_;

  bool enabled_;
  double min_interval_;

  double timeout_;
  double timer_;
};

template <class ConcreteModel>
class Measurement_ : public Measurement {
public:
  typedef ConcreteModel Model;
  typedef typename traits::Update<ConcreteModel>::type Update;

  enum { MeasurementDimension = Model::MeasurementDimension };
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
  virtual int getDimension() const { return MeasurementDimension; }

  virtual bool active(const SystemStatus& status) { return enabled() && model_->applyStatusMask(status); }

  virtual MeasurementVector const& getVector(const Update &update, const State &state) {
    return internal::UpdateInspector<ConcreteModel>::getVector(update, state, getModel());
  }

  virtual NoiseVariance const& getVariance(const Update &update, const State &state) {
    if (update.hasVariance()) return internal::UpdateInspector<ConcreteModel>::getVariance(update, state, getModel());
    model_->getMeasurementNoise(R_, state, false);
    return R_;
  }

  virtual void setNoiseVariance(NoiseVariance const& R) {
    R_ = R;
  }

  const boost::shared_ptr< Filter::Corrector_<Model> >& filter() const { return filter_; }
  void setFilter(Filter *filter = 0); // implemented in filter/set_filter.h

protected:
  virtual bool updateImpl(State &state, const MeasurementUpdate &update);
  virtual bool prepareUpdate(State &state, const Update &update) { return getModel()->prepareUpdate(state, update); }
  virtual void afterUpdate(State &state) { getModel()->afterUpdate(state); }

protected:
  boost::shared_ptr<Model> model_;
  NoiseVariance R_;

  Queue_<Update> queue_;
  virtual Queue& queue() { return queue_; }

  boost::shared_ptr< Filter::Corrector_<Model> > filter_;
};

template <typename ConcreteModel>
boost::shared_ptr<Measurement_<ConcreteModel> > Measurement::create(ConcreteModel *model, const std::string& name)
{
  return boost::make_shared<Measurement_<ConcreteModel> >(model, name);
}

} // namespace hector_pose_estimation

#include <hector_pose_estimation/filter.h>
//#include <hector_pose_estimation/filter/ekf.h>

namespace hector_pose_estimation {

template <class ConcreteModel>
  bool Measurement_<ConcreteModel>::updateImpl(State &state, const MeasurementUpdate &update_)
  {
    Update const &update = dynamic_cast<Update const &>(update_);
    if (!prepareUpdate(state, update)) return false;

    ROS_DEBUG_NAMED(getName(), "Updating with measurement %s", getName().c_str());
    const MeasurementVector &y = getVector(update, state);
    const NoiseVariance &R = getVariance(update, state);

    ROS_DEBUG_STREAM_NAMED(getName(), "x_prior    = [" << state.getVector().transpose() << "]");
    ROS_DEBUG_STREAM_NAMED(getName(), "P_prior    = [" << state.getCovariance() << "]");
    ROS_DEBUG_STREAM_NAMED(getName(), "y          = [" << y.transpose() << "]");
    ROS_DEBUG_STREAM_NAMED(getName(), "R          = [" << R << "]");

    if (!this->filter() || !this->filter()->correct(state, y, R)) return false;

//    if (this->filter()->derived<filter::EKF>()) {
//      ROS_DEBUG_STREAM_NAMED(getName(), "h(x)      = [" << this->filter()->derived<filter::EKF>()->y_pred.transpose() << "]");
//      ROS_DEBUG_STREAM_NAMED(getName(), "H = dh/dx = [" << this->filter()->derived<filter::EKF>()->C << "]");
//    }
    ROS_DEBUG_STREAM_NAMED(getName(), "x_post    = [" << state.getVector().transpose() << "]");
    ROS_DEBUG_STREAM_NAMED(getName(), "P_post    = [" << state.getCovariance() << "]");

    afterUpdate(state);
    return true;
  }

} // namespace hector_pose_estimation

#endif // HECTOR_POSE_ESTIMATION_MEASUREMENT_H
