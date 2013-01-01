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

#ifndef HECTOR_POSE_ESTIMATION_SYSTEM_H
#define HECTOR_POSE_ESTIMATION_SYSTEM_H

#include "system_model.h"
#include "system_input.h"

namespace BFL { class KalmanFilter; }

namespace hector_pose_estimation {

class PoseEstimation;

class System
{
public:
  System(const std::string& name);
  virtual ~System();

  template <typename ConcreteModel> static boost::shared_ptr<System> create(ConcreteModel *model, const std::string& name = "system");

  virtual const std::string& getName() const { return name_; }
  virtual void setName(const std::string& name) { name_ = name; }

  virtual SystemModel *getModel() const = 0;

  virtual bool init();
  virtual void cleanup();
  virtual void reset();

  virtual SystemStatus getStatusFlags() const { return status_flags_; }

  virtual ParameterList& parameters() { return parameters_; }
  virtual const ParameterList& parameters() const { return parameters_; }

  BFL::Gaussian *getPrior();
  virtual const SystemInput& getInput() const = 0;
  virtual void setInput(const SystemInput& input) = 0;

  virtual bool update(PoseEstimation &estimator, double dt) = 0;
  virtual void updated();
  virtual StateVector limitState(StateVector state) const;

protected:
  void updateInternal(PoseEstimation &estimator, double dt, ColumnVector const& u);

protected:
  std::string name_;
  ParameterList parameters_;
  SystemStatus status_flags_;
  BFL::Gaussian prior_;
};

typedef boost::shared_ptr<System> SystemPtr;

template <typename ConcreteModel, typename ConcreteInput = typename Input_<ConcreteModel>::Type >
class System_ : public System
{
public:
  typedef ConcreteModel Model;
  typedef ConcreteInput Input;
  static const unsigned int InputDimension = Model::InputDimension;
  typedef typename Model::InputVector InputVector;

  System_(const std::string& name = "system")
    : System(name)
    , model_(new Model)
  {
    parameters_.add(model_->parameters());
  }

  System_(Model *model, const std::string& name)
    : System(name)
    , model_(model)
  {
    parameters_.add(model_->parameters());
  }
  virtual SystemModel *getModel() const { return model_.get(); }

  virtual const Input& getInput() const { return input_; }
  virtual void setInput(const SystemInput& input) { input_ = dynamic_cast<const Input&>(input); }
  virtual void setInput(const Input& input) { input_ = input; }

  virtual bool update(PoseEstimation &estimator, double dt);

private:
  boost::shared_ptr<Model> model_;
  Input input_;
};

template <class ConcreteModel, class ConcreteInput>
bool System_<ConcreteModel, ConcreteInput>::update(PoseEstimation &estimator, double dt)
{
  updateInternal(estimator, dt, input_.getVector());
  return true;
}

template <typename ConcreteModel>
SystemPtr System::create(ConcreteModel *model, const std::string& name)
{
  return SystemPtr(new System_<ConcreteModel>(model, name));
}

} // namespace hector_pose_estimation

#endif // HECTOR_POSE_ESTIMATION_SYSTEM_H
