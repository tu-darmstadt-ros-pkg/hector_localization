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

#include <hector_pose_estimation/filter.h>
#include <hector_pose_estimation/pose_estimation.h>

namespace hector_pose_estimation {

Filter::Filter(State& state)
  : state_(state)
{
}

Filter::~Filter()
{
}

bool Filter::reset()
{
  state_.reset();
  return true;
}

void Filter::predict(State& state, const Systems& systems, double dt) {
  SystemStatus system_status = 0;

  for(Systems::iterator it = systems.begin(); it != systems.end(); it++) {
    const SystemPtr& system = *it;
    predict(state, system, dt);
    system_status |= system->getStatusFlags();
  }

  state.setSystemStatus(system_status);
}

void Filter::predict(State& state, const SystemPtr& system, double dt) {
  system->update(*this, state, dt);
}

void Filter::update(State& state, const Measurements& measurements) {
  SystemStatus measurement_status = 0;

  for(Measurements::iterator it = measurements.begin(); it != measurements.end(); it++) {
    const MeasurementPtr& measurement = *it;
    update(state, measurement);
    measurement_status |= measurement->getStatusFlags();
  }

  state.setMeasurementStatus(measurement_status);
}

void Filter::update(State& state, const MeasurementPtr& measurement) {
  // measurement->process(*this, state);
  measurement->process(state);
}

} // namespace hector_pose_estimation
