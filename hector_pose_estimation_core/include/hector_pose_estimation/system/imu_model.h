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

#ifndef HECTOR_POSE_ESTIMATION_IMU_MODEL_H
#define HECTOR_POSE_ESTIMATION_IMU_MODEL_H

#include <hector_pose_estimation/system_model.h>
#include <hector_pose_estimation/system.h>

namespace hector_pose_estimation {

class GyroModel : public TimeContinuousSystemModel_<GyroModel, SubSystemModel, 3>
{
public:
  enum StateIndex {
    BIAS_GYRO_X = 0,
    BIAS_GYRO_Y,
    BIAS_GYRO_Z
  };

  GyroModel();
  virtual ~GyroModel();

  virtual bool init(PoseEstimation& estimator, State& state);
  virtual void getPrior(State &state);

  virtual void getSystemNoise(NoiseVariance& Q, const State& state, bool init);
  virtual void getStateJacobian(SystemMatrix& Asub, CrossSystemMatrix& Across, const State& state, bool init);

  ConstVectorBlock3 getBias() const { return drift_->getSegment<3>(BIAS_GYRO_X); }

private:
  SubStatePtr drift_;
  double rate_drift_;
};

class AccelerometerModel : public TimeContinuousSystemModel_<AccelerometerModel, SubSystemModel, 3>
{
public:
  enum StateIndex {
    BIAS_ACCEL_X = 0,
    BIAS_ACCEL_Y,
    BIAS_ACCEL_Z,
  };

  AccelerometerModel();
  virtual ~AccelerometerModel();

  virtual bool init(PoseEstimation& estimator, State& state);
  virtual void getPrior(State &state);

  virtual void getSystemNoise(NoiseVariance& Q, const State& state, bool init);
  virtual void getStateJacobian(SystemMatrix& Asub, CrossSystemMatrix& Across, const State& state, bool init);

  ConstVectorBlock3 getBias() const { return drift_->getSegment<3>(BIAS_ACCEL_X); }

private:
  SubStatePtr drift_;
  double acceleration_drift_;
};

typedef System_<GyroModel> Gyro;
typedef System_<AccelerometerModel> Accelerometer;

}

#endif // HECTOR_POSE_ESTIMATION_IMU_MODEL_H
