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

#include <hector_pose_estimation/system_model.h>
#include <hector_pose_estimation/state.h>
#include <hector_pose_estimation/input.h>
#include <hector_pose_estimation/collection.h>

namespace hector_pose_estimation {

class Filter;

class System
{
public:
  System(const std::string& name);
  virtual ~System();

  template <typename ConcreteModel> static boost::shared_ptr<System> create(ConcreteModel *model, const std::string& name = "system");

  virtual const std::string& getName() const { return name_; }
  virtual void setName(const std::string& name) { name_ = name; }

  virtual SystemModel *getModel() const = 0;

  virtual bool init(PoseEstimation& estimator, State& state);
  virtual void cleanup();
  virtual void reset(State& state);

  virtual SystemStatus getStatusFlags() const { return status_flags_; }

  virtual ParameterList& parameters() { return parameters_; }
  virtual const ParameterList& parameters() const { return parameters_; }

  virtual void getPrior(State &state) const;

  virtual bool update(Filter &filter, State &state, double dt);
  virtual void updated();
  virtual bool limitState(State& state) const;

protected:
  std::string name_;
  ParameterList parameters_;
  SystemStatus status_flags_;
};

typedef boost::shared_ptr<System> SystemPtr;
typedef boost::weak_ptr<System> SystemWPtr;
typedef Collection<System> Systems;

template <typename ConcreteModel>
class System_ : public System
{
public:
  typedef ConcreteModel Model;
  typedef typename Input::traits<ConcreteModel>::Type InputType;
  typedef typename Input::traits<ConcreteModel>::Vector InputVector;

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

  virtual ~System_() {}

  virtual SystemModel *getModel() const { return model_.get(); }
  virtual int getDimension() const { return model_->getDimension(); }

  virtual const Input& getInput() const { return input_; }
  virtual void setInput(const Input& input) { input_ = dynamic_cast<const InputType&>(input); }
  virtual void setInput(const InputType& input) { input_ = input; }

private:
  boost::shared_ptr<Model> model_;
  InputType input_;
};

template <typename ConcreteModel>
SystemPtr System::create(ConcreteModel *model, const std::string& name)
{
  return SystemPtr(new System_<ConcreteModel>(model, name));
}

} // namespace hector_pose_estimation

#endif // HECTOR_POSE_ESTIMATION_SYSTEM_H
