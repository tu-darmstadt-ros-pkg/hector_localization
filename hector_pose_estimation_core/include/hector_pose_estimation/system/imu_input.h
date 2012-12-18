//=================================================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
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

#include <hector_pose_estimation/system_input.h>
#include <hector_pose_estimation/matrix.h>
#include <sensor_msgs/Imu.h>

#ifndef HECTOR_POSE_ESTIMATION_IMU_INPUT_H
#define HECTOR_POSE_ESTIMATION_IMU_INPUT_H

namespace hector_pose_estimation {

class ImuInput : public SystemInput
{
public:
  enum InputIndex {
    ACCEL_X = 1,
    ACCEL_Y,
    ACCEL_Z,
    GYRO_X,
    GYRO_Y,
    GYRO_Z
  };
  static const unsigned int InputDimension = GYRO_Z;
  typedef ColumnVector_<InputDimension> InputVector;

  ImuInput()
    : u_(InputDimension)
  {}
  ImuInput(InputVector const& u)
    : u_(InputDimension)
  {
    setValue(u);
  }
  ImuInput(const sensor_msgs::Imu& imu)
    : u_(InputDimension)
  {
    setValue(imu);
  }
  virtual ~ImuInput() {}

  virtual void setValue(InputVector const& u) { u_ = u; }
  virtual void setValue(const sensor_msgs::Imu& imu) {
    u_(ACCEL_X) = imu.linear_acceleration.x;
    u_(ACCEL_Y) = imu.linear_acceleration.y;
    u_(ACCEL_Z) = imu.linear_acceleration.z;
    u_(GYRO_X)  = imu.angular_velocity.x;
    u_(GYRO_Y)  = imu.angular_velocity.y;
    u_(GYRO_Z)  = imu.angular_velocity.z;
  }

  virtual InputVector const &getVector() const { return u_; }
  virtual ColumnVector_<3> getAccel() const { return u_.sub(ACCEL_X, ACCEL_Z); }
  virtual ColumnVector_<3> getRate() const { return u_.sub(GYRO_X, GYRO_Z); }

  virtual InputVector &operator=(InputVector const& u) { setValue(u); return u_; }
  virtual InputVector &operator=(const sensor_msgs::Imu& imu) { setValue(imu); return u_; }

protected:
  InputVector u_;
};

} // namespace hector_pose_estimation

#endif // HECTOR_POSE_ESTIMATION_IMU_INPUT_H
