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

GlobalReference *GlobalReference::Instance()
{
  static GlobalReference *instance = new GlobalReference();
  return instance;
}

void GlobalReference::reset()
{
  position_ = Position();
  heading_ = Heading();

  has_position_ = false;
  has_heading_ = false;
  has_altitude_ = false;

  updated();
}

ParameterList& GlobalReference::parameters() {
  return parameters_;
}

void GlobalReference::updated() {
  // check if a non-default reference has been set
  if (position_.latitude  != Position().latitude)  has_position_ = true;
  if (position_.longitude != Position().longitude) has_position_ = true;
  if (position_.altitude  != Position().altitude)  has_altitude_ = true;
  if (heading_.value      != Heading().value)      has_heading_  = true;

  // WGS84 constants
  static const double equatorial_radius = 6378137.0;
  static const double flattening = 1.0/298.257223563;
  static const double excentrity2 = 2*flattening - flattening*flattening;

  // calculate earth radii
  double temp = 1.0 / (1.0 - excentrity2 * sin(position_.latitude) * sin(position_.latitude));
  double prime_vertical_radius = equatorial_radius * sqrt(temp);
  radius_.north = prime_vertical_radius * (1 - excentrity2) * temp;
  radius_.east  = prime_vertical_radius * cos(position_.latitude);

  // calculate sin and cos of the heading reference
  sincos(heading_.value, &heading_.sin, &heading_.cos);
}

void GlobalReference::fromWGS84(double latitude, double longitude, double &x, double &y) {
  double north = radius_.north * (latitude  - position_.latitude);
  double east  = radius_.east  * (longitude - position_.longitude);
  fromNorthEast(north, east, x, y);
}

void GlobalReference::toWGS84(double x, double y, double &latitude, double &longitude) {
  if (radius_.north == 0.0 || radius_.east == 0.0) {
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
  x = north * heading_.cos + east * heading_.sin;
  y = north * heading_.sin - east * heading_.cos;
}

void GlobalReference::toNorthEast(double x, double y, double &north, double &east) {
  north = x * heading_.cos + y * heading_.sin;
  east  = x * heading_.sin - y * heading_.cos;
}

GlobalReference& GlobalReference::setPosition(double latitude, double longitude) {
  position_.latitude = latitude;
  position_.longitude = longitude;
  has_position_ = true;
  updated();
  return *this;
}

GlobalReference& GlobalReference::setHeading(double heading) {
  heading_.value = heading;
  has_heading_ = true;
  updated();
  return *this;
}

GlobalReference& GlobalReference::setAltitude(double altitude) {
  position_.altitude = altitude;
  has_altitude_ = true;
  updated();
  return *this;
}

} // namespace hector_pose_estimation
