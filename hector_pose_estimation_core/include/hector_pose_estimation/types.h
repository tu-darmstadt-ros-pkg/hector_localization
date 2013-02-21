//=================================================================================================
// Copyright (c) 2011, Johannes Meyer and Martin Nowara, TU Darmstadt
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

#ifndef HECTOR_POSE_ESTIMATION_TYPES_H
#define HECTOR_POSE_ESTIMATION_TYPES_H

#include <hector_pose_estimation/matrix.h>
#include <hector_pose_estimation/collection.h>

// Use system model with angular rates.
// #define USE_RATE_SYSTEM_MODEL

namespace hector_pose_estimation {

  enum {
    STATE_ALIGNMENT = 1,
    STATE_DEGRADED = 2,
    STATE_READY = 4,

    STATE_ROLLPITCH = 8,
    STATE_YAW = 16,
    STATE_XY_VELOCITY = 32,
    STATE_XY_POSITION = 64,
    STATE_Z_VELOCITY = 128,
    STATE_Z_POSITION = 256,

    STATE_ALL = -1
  };
  typedef int SystemStatus;

  static const char* const SystemStatusStrings[] = {
    "ALIGNMENT", "DEGRADED", "READY", "ROLLPITCH", "YAW", "XY_VELOCITY", "XY_POSITION", "Z_VELOCITY", "Z_POSITION"
  };
  std::string getSystemStatusString(const SystemStatus& status);
  static inline std::ostream& operator<<(std::ostream& os, const SystemStatus& status) {
    return os << getSystemStatusString(status);
  }

  class SystemModel;
  class System;
  typedef boost::shared_ptr<System> SystemPtr;
  typedef boost::weak_ptr<System> SystemWPtr;
  typedef Collection<System> Systems;

  class MeasurementModel;
  class MeasurementUpdate;
  class Measurement;
  typedef boost::shared_ptr<Measurement> MeasurementPtr;
  typedef boost::weak_ptr<Measurement> MeasurementWPtr;
  typedef Collection<Measurement> Measurements;

  class Input;
  typedef boost::shared_ptr<Input> InputPtr;
  typedef boost::weak_ptr<Input> InputWPtr;
  typedef Collection<Input> Inputs;

  class PoseEstimation;
  class Filter;
  class State;

} // namespace hector_pose_estimation

#endif // HECTOR_POSE_ESTIMATION_TYPES_H
