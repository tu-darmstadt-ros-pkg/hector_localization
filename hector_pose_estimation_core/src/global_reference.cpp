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

#include <hector_pose_estimation/global_reference.h>
#include <hector_pose_estimation/state.h>
#include <cmath>

using namespace std;

namespace hector_pose_estimation {

GlobalReference::GlobalReference()
{
  parameters().add("reference_latitude",  position_.latitude);
  parameters().add("reference_longitude", position_.longitude);
  parameters().add("reference_altitude",  position_.altitude);
  parameters().add("reference_heading",   heading_.value);

  reset();
}

const GlobalReferencePtr &GlobalReference::Instance()
{
  static GlobalReferencePtr instance;
  if (!instance) { instance.reset(new GlobalReference); }
  return instance;
}

void GlobalReference::reset()
{
  position_ = Position();
  heading_ = Heading();
  radius_ = Radius();

  updated();
}

ParameterList& GlobalReference::parameters() {
  return parameters_;
}

void GlobalReference::updated() {
  // WGS84 constants
  static const double equatorial_radius = 6378137.0;
  static const double flattening = 1.0/298.257223563;
  static const double excentrity2 = 2*flattening - flattening*flattening;

  // calculate earth radii
  if (hasPosition()) {
    double temp = 1.0 / (1.0 - excentrity2 * sin(position_.latitude) * sin(position_.latitude));
    double prime_vertical_radius = equatorial_radius * sqrt(temp);
    radius_.north = prime_vertical_radius * (1 - excentrity2) * temp;
    radius_.east  = prime_vertical_radius * cos(position_.latitude);
  }

  // calculate sin and cos of the heading reference
  if (hasHeading()) {
    sincos(heading_.value, &heading_.sin, &heading_.cos);
  }
}

void GlobalReference::fromWGS84(double latitude, double longitude, double &x, double &y) {
  if (!hasPosition()) {
    x = 0.0;
    y = 0.0;
    return;
  }
  double north = radius_.north * (latitude  - position_.latitude);
  double east  = radius_.east  * (longitude - position_.longitude);
  fromNorthEast(north, east, x, y);
}

void GlobalReference::toWGS84(double x, double y, double &latitude, double &longitude) {
  if (!hasPosition()) {
    latitude  = 0.0;
    longitude = 0.0;
    return;
  }

  double north, east;
  toNorthEast(x, y, north, east);
  latitude  = position_.latitude  + north / radius_.north;
  longitude = position_.longitude + east  / radius_.east;
}

void GlobalReference::fromNorthEast(double north, double east, double &x, double &y) {
  if (!hasHeading()) {
    x = 0.0;
    y = 0.0;
    return;
  }
  x = north * heading_.cos + east * heading_.sin;
  y = north * heading_.sin - east * heading_.cos;
}

void GlobalReference::toNorthEast(double x, double y, double &north, double &east) {
  if (!hasHeading()) {
    north = 0.0;
    east = 0.0;
    return;
  }
  north = x * heading_.cos + y * heading_.sin;
  east  = x * heading_.sin - y * heading_.cos;
}

GlobalReference& GlobalReference::setPosition(double latitude, double longitude, bool quiet /* = false */) {
  position_.latitude = latitude;
  position_.longitude = longitude;
  updated();
  if (!quiet) ROS_INFO("Set new reference position to %f deg N / %f deg E", this->position().latitude * 180.0/M_PI, this->position().longitude * 180.0/M_PI);
  return *this;
}

GlobalReference& GlobalReference::setHeading(double heading, bool quiet /* = false */) {
  heading_.value = heading;
  updated();
  if (!quiet) ROS_INFO("Set new reference heading to %.1f degress", this->heading() * 180.0 / M_PI);
  return *this;
}

GlobalReference& GlobalReference::setAltitude(double altitude, bool quiet /* = false */) {
  position_.altitude = altitude;
  updated();
  if (!quiet) ROS_INFO("Set new reference altitude to %.2f m", this->position().altitude);
  return *this;
}

GlobalReference& GlobalReference::setCurrentPosition(const State& state, double new_latitude, double new_longitude) {
  State::ConstPositionType position = state.getPosition();

  // set reference to new latitude/longitude first (intermediate reference)
  setPosition(new_latitude, new_longitude, true);

  // convert current position back to WGS84 using the new reference position
  // and reset the reference position so that current position in x/y coordinates remains the same
  // This will work unless the radii at the origin and the x/y position of the robot differ too much
  toWGS84(-position.x(), -position.y(), new_latitude, new_longitude);
  setPosition(new_latitude, new_longitude);

  return *this;
}

GlobalReference& GlobalReference::setCurrentHeading(const State& state, double new_heading) {
  // get current yaw angle
  double current_yaw = state.getYaw();
  State::ConstPositionType position = state.getPosition();

  // get current position in WGS84
  double current_latitude, current_longitude;
  if (hasPosition()) {
    toWGS84(position.x(), position.y(), current_latitude, current_longitude);
  }

  // set the new reference heading
  setHeading(new_heading - (-current_yaw));

  // set the new reference position so that current position in WGS84 coordinates remains the same as before
  if (hasPosition()) {
    setCurrentPosition(state, current_latitude, current_longitude);
  }

  return *this;
}

GlobalReference& GlobalReference::setCurrentAltitude(const State& state, double new_altitude) {
  State::ConstPositionType position = state.getPosition();
  setAltitude(new_altitude - position.z());
  return *this;
}

} // namespace hector_pose_estimation
