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

#include <hector_pose_estimation/measurements/gps.h>
#include <hector_pose_estimation/pose_estimation.h>

namespace hector_pose_estimation {

GPSModel::GPSModel()
  : MeasurementModel(4)
{
  SymmetricMatrix noise(4);
  position_stddev_ = 10.0;
  velocity_stddev_ = 1.0;
  parameters().add("position_stddev", position_stddev_);
  parameters().add("velocity_stddev", velocity_stddev_);
  noise(1,1) = noise(2,2) = pow(position_stddev_, 2);
  noise(3,3) = noise(4,4) = pow(velocity_stddev_, 2);
  this->AdditiveNoiseSigmaSet(noise);
}

GPSModel::~GPSModel() {}

SystemStatus GPSModel::getStatusFlags() const {
  return STATE_XY_VELOCITY | STATE_XY_POSITION;
}

ColumnVector GPSModel::ExpectedValueGet() const {
  this->y_(1) = x_(POSITION_X);
  this->y_(2) = x_(POSITION_Y);
  this->y_(3) = x_(VELOCITY_X);
  this->y_(4) = x_(VELOCITY_Y);
  return y_;
}

Matrix GPSModel::dfGet(unsigned int i) const {
  if (i != 0) return Matrix();
  C_(1,POSITION_X) = 1.0;
  C_(2,POSITION_Y) = 1.0;
  C_(3,VELOCITY_X) = 1.0;
  C_(4,VELOCITY_Y) = 1.0;
  return C_;
}

GPS::GPS(const std::string &name)
  : Measurement_<GPSModel,GPSUpdate>(name)
  , reference_latitude_(0.0)
  , reference_longitude_(0.0)
  , reference_heading_(0.0)
  , has_reference_(false)
  , y_(4)
{
  parameters().add("reference_latitude", reference_latitude_);
  parameters().add("reference_longitude", reference_longitude_);
  parameters().add("reference_heading", reference_heading_);
}

GPS::~GPS()
{}

void GPS::onReset() {
  has_reference_ = false;
}

typename GPSModel::MeasurementVector const& GPS::getValue(const GPSUpdate &update) {
  double north = radius_north_ * (update.latitude  - reference_latitude_);
  double east  = radius_east_  * (update.longitude - reference_longitude_);

  y_(1) = north * cos_reference_heading_ + east * sin_reference_heading_;
  y_(2) = north * sin_reference_heading_ - east * cos_reference_heading_;
  y_(3) = update.velocity_north * cos_reference_heading_ + update.velocity_east * sin_reference_heading_;
  y_(4) = update.velocity_north * sin_reference_heading_ - update.velocity_east * cos_reference_heading_;

  last_ = update;
  return y_;
}

bool GPS::beforeUpdate(PoseEstimation &estimator, const GPSUpdate &update) {
  // reset reference position if GPS has not been updated for a while
  if (timedout()) has_reference_ = false;

  // find new reference position
  if (!has_reference_) {
    reference_latitude_ = update.latitude;
    reference_longitude_ = update.longitude;
    updateReference();

    StateVector state = estimator.getState();
    double north =  state(POSITION_X) * cos_reference_heading_ + state(POSITION_Y) * sin_reference_heading_;
    double east  =  state(POSITION_X) * sin_reference_heading_ - state(POSITION_Y) * cos_reference_heading_;
    reference_latitude_  = update.latitude  - north / radius_north_;
    reference_longitude_ = update.longitude - east  / radius_east_;
    has_reference_ = true;

    ROS_INFO("Set new GPS reference position: %f/%f", reference_latitude_ * 180.0/M_PI, reference_longitude_ * 180.0/M_PI);
  }

  return true;
}

void GPS::updateReference() {
  static const double radius_earth = 6371e3;
  radius_north_ = radius_earth;
  radius_east_  = radius_earth * cos(reference_latitude_);
  cos_reference_heading_ = cos(reference_heading_);
  sin_reference_heading_ = sin(reference_heading_);
}

} // namespace hector_pose_estimation
