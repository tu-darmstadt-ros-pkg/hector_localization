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

#include <hector_pose_estimation/measurement.h>

namespace hector_pose_estimation {

Measurement::Measurement(const std::string& name)
  : name_(name)
  , status_flags_(0)
  , enabled_(true)
  , min_interval_(0.0)
  , timeout_(1.0)
  , timer_(0.0)
{
  parameters().add("enabled", enabled_);
  parameters().add("min_interval", min_interval_);
}

Measurement::~Measurement()
{
}

bool Measurement::init(PoseEstimation& estimator, Filter& filter, State& state)
{
  if (!getModel()) return false;
  setFilter(&filter);
  if (!onInit(estimator, state)) return false;
  return getModel()->init(estimator, state);
}

void Measurement::cleanup()
{
  onCleanup();
  if (getModel()) getModel()->cleanup();
}

void Measurement::reset(State& state)
{
  queue().clear();
  timer_ = 0;
  onReset();
  if (getModel()) getModel()->reset(state);
}

void Measurement::increase_timer(double dt) {
  timer_ += dt;
}

void Measurement::updated() {
  timer_ = 0.0;
  if (getModel()) status_flags_ = getModel()->getStatusFlags();
}

bool Measurement::timedout() const {
  if (timer_ > timeout_) {
    if (status_flags_ > 0) ROS_WARN("Measurement %s timed out.", getName().c_str());
    return true;
  }
  return false;
}

void Measurement::add(const MeasurementUpdate& update) {
  queue().push(update);
}

void Measurement::process(State &state) {
  while(!(queue().empty())) {
    update(state, queue().pop());
  }

  // check for timeout
  if (timedout()) status_flags_ = 0;
}

bool Measurement::update(State &state, const MeasurementUpdate &update)
{
  if (!enabled()) return false;
  if (min_interval_ > 0.0 && timer_ < min_interval_) return false;

  if (!updateImpl(state, update)) return false;

  updated();
  return true;
}

} // namespace hector_pose_estimation
