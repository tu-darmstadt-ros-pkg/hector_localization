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
#include <hector_pose_estimation/global_reference.h>
#include <hector_pose_estimation/filter/set_filter.h>

namespace hector_pose_estimation {

template class Measurement_<GPSModel>;

GPSModel::GPSModel()
{
  position_stddev_ = 10.0;
  velocity_stddev_ = 1.0;
  parameters().add("position_stddev", position_stddev_);
  parameters().add("velocity_stddev", velocity_stddev_);
}

GPSModel::~GPSModel() {}

void GPSModel::getMeasurementNoise(NoiseVariance& R, const State&, bool init)
{
  if (init) {
    R(0,0) = R(1,1) = pow(position_stddev_, 2);
    R(2,2) = R(3,3) = pow(velocity_stddev_, 2);
  }
}

bool GPSModel::prepareUpdate(State &state, const MeasurementUpdate &update)
{
  state.getRotationMatrix(R);
  return true;
}

void GPSModel::getExpectedValue(MeasurementVector& y_pred, const State& state)
{
  y_pred(0) = state.getPosition().x();
  y_pred(1) = state.getPosition().y();
#ifdef VELOCITY_IN_BODY_FRAME
  y_pred(2) = R.row(0) * state.getVelocity();
  y_pred(3) = R.row(1) * state.getVelocity();
#else
  y_pred(2) = state.getVelocity().x();
  y_pred(3) = state.getVelocity().y();
#endif
}

void GPSModel::getStateJacobian(MeasurementMatrix& C, const State& state, bool init)
{
  if (state.getPositionIndex() >= 0) {
    C(0,State::POSITION_X) = 1.0;
    C(1,State::POSITION_Y) = 1.0;
  }

#ifdef VELOCITY_IN_BODY_FRAME
  State::ConstOrientationType q(state.getOrientation());
  State::ConstVelocityType v(state.getVelocity());

  if (state.getOrientationIndex() >= 0) {
    C(2,State::QUATERNION_W) = -2.0*q.z()*v.y()+2.0*q.y()*v.z()+2.0*q.w()*v.x();
    C(2,State::QUATERNION_X) =  2.0*q.y()*v.y()+2.0*q.z()*v.z()+2.0*q.x()*v.x();
    C(2,State::QUATERNION_Y) = -2.0*q.y()*v.x()+2.0*q.x()*v.y()+2.0*q.w()*v.z();
    C(2,State::QUATERNION_Z) = -2.0*q.z()*v.x()-2.0*q.w()*v.y()+2.0*q.x()*v.z();

    C(3,State::QUATERNION_W) =  2.0*q.z()*v.x()-2.0*q.x()*v.z()+2.0*q.w()*v.y();
    C(3,State::QUATERNION_X) =  2.0*q.y()*v.x()-2.0*q.x()*v.y()-2.0*q.w()*v.z();
    C(3,State::QUATERNION_Y) =  2.0*q.x()*v.x()+2.0*q.z()*v.z()+2.0*q.y()*v.y();
    C(3,State::QUATERNION_Z) =  2.0*q.w()*v.x()-2.0*q.z()*v.y()+2.0*q.y()*v.z();
  }

  if (state.getVelocityIndex() >= 0) {
    C(2,State::VELOCITY_X)   =  R(0,0);
    C(2,State::VELOCITY_Y)   =  R(0,1);
    C(2,State::VELOCITY_Z)   =  R(0,2);

    C(3,State::VELOCITY_X)   =  R(1,0);
    C(3,State::VELOCITY_Y)   =  R(1,1);
    C(3,State::VELOCITY_Z)   =  R(1,2);
  }

#else
  if (!init) return; // C is time-constant

  if (state.getVelocityIndex() >= 0) {
    C(2,State::VELOCITY_X) = 1.0;
    C(3,State::VELOCITY_Y) = 1.0;
  }
#endif
}

GPS::GPS(const std::string &name)
  : Measurement_<GPSModel>(name)
  , y_(4)
{
}

GPS::~GPS()
{}

void GPS::onReset() {
  reference_.reset();
}

GPSModel::MeasurementVector const& GPS::getVector(const GPSUpdate &update, const State&) {
  if (!reference_) {
    y_ = 0.0/0.0;
    return y_;
  }

  reference_->fromWGS84(update.latitude, update.longitude, y_(0), y_(1));
  reference_->fromNorthEast(update.velocity_north, update.velocity_east, y_(2), y_(3));

  last_ = update;
  return y_;
}

bool GPS::prepareUpdate(State &state, const Update &update) {
  // reset reference position if GPS has not been updated for a while
  if (timedout()) reference_.reset();

  // find new reference position
  if (reference_ != GlobalReference::Instance()) {
    reference_ = GlobalReference::Instance();
    reference_->setCurrentPosition(state, update.latitude, update.longitude);
  }

  return true;
}

} // namespace hector_pose_estimation
