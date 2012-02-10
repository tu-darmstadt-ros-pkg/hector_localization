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

namespace BFL { class KalmanFilter; }

namespace hector_pose_estimation {

class PoseEstimation;

class System
{
public:
  System(SystemModel *model, const std::string& name = "system");
  virtual ~System();

  virtual const std::string& getName() const { return name_; }
  virtual void setName(const std::string& name) { name_ = name; }

	virtual void setModel(SystemModel *system_model) { model_ = system_model; }
	virtual SystemModel *getModel() const { return model_; }

  virtual bool init();
  virtual void cleanup();
  virtual void reset();
  virtual void reset(const StateVector& state) { reset(); }

  virtual SystemStatus getStatusFlags() const { return status_flags_; }

  virtual ParameterList& parameters() { return parameters_; }
  virtual const ParameterList& parameters() const { return parameters_; }

	BFL::Gaussian *getPrior() { return &prior_; }
	const InputVector& getInput() const { return input_; }
	void setInput(const InputVector& input) { input_ = input; }

	virtual bool update(PoseEstimation &estimator, double dt);
	virtual void updated();
	StateVector limitState(StateVector state) const;

protected:
	SystemModel *model_;
	std::string name_;
	ParameterList parameters_;
	SystemStatus status_flags_;

	BFL::Gaussian prior_;
	InputVector input_;
};

} // namespace hector_pose_estimation

#endif // HECTOR_POSE_ESTIMATION_SYSTEM_H
