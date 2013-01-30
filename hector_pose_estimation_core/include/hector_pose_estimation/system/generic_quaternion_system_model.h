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

#ifndef HECTOR_POSE_ESTIMATION_GENERIC_QUATERNION_SYSTEM_MODEL_H
#define HECTOR_POSE_ESTIMATION_GENERIC_QUATERNION_SYSTEM_MODEL_H

#include <hector_pose_estimation/system_model.h>
#include <hector_pose_estimation/system/imu_input.h>

namespace hector_pose_estimation {

class GenericQuaternionSystemModel;
template <> struct Input_<GenericQuaternionSystemModel> {
  typedef ImuInput Type;
};

class GenericQuaternionSystemModel : public SystemModel
{
public:
  static const unsigned int InputDimension = ImuInput::InputDimension;
  typedef ImuInput::InputVector InputVector;

  GenericQuaternionSystemModel();
  virtual ~GenericQuaternionSystemModel();

  virtual std::string getName() const { return "GenericQuaternionSystemModel"; }
  virtual bool init();

  virtual SystemStatus getStatusFlags() const;

  virtual ColumnVector ExpectedValueGet(double dt) const;
  virtual SymmetricMatrix CovarianceGet(double dt) const;
  virtual Matrix dfGet(unsigned int i, double dt) const;

  virtual bool limitState(State &x) const;

  void setGravity(double gravity) { gravity_ = gravity; }
  double getGravity() const { return gravity_; }

protected:
  static void normalize(State &x);

protected:
  double gravity_;
  double rate_stddev_;
#ifdef USE_RATE_SYSTEM_MODEL
  double angular_acceleration_stddev_;
#endif // USE_RATE_SYSTEM_MODEL
  double acceleration_stddev_;
  double velocity_stddev_;
  double acceleration_drift_;
  double rate_drift_;

  mutable double q0,q1,q2,q3;
  mutable SymmetricMatrix_<State::StateDimension> noise_;
};

} // namespace hector_pose_estimation

#endif // HECTOR_POSE_ESTIMATION_GENERIC_QUATERNION_SYSTEM_MODEL_H
