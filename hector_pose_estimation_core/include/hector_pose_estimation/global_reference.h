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

#ifndef HECTOR_POSE_ESTIMATION_GLOBAL_REFERENCE_H
#define HECTOR_POSE_ESTIMATION_GLOBAL_REFERENCE_H

#include <hector_pose_estimation/types.h>
#include <hector_pose_estimation/parameters.h>

namespace hector_pose_estimation {

  class PoseEstimation;

  class GlobalReference {
  public:
    struct Position {
      Position() : latitude(0.0), longitude(0.0), altitude(0.0) {}
      double latitude;
      double longitude;
      double altitude;
    };

    struct Heading {
      Heading() : value(0.0), cos(1.0), sin(0.0) {}
      double value;
      double cos;
      double sin;
      operator double() const { return value; }
    };

    struct Radius {
      Radius() : north(0.0), east(0.0) {}
      double north;
      double east;
    };

    const Position& position() const { return position_; }
    const Heading& heading() const { return heading_; }
    const Radius& radius() const { return radius_; }

    GlobalReference& setPosition(double latitude, double longitude, bool quiet = false);
    GlobalReference& setHeading(double heading, bool quiet = false);
    GlobalReference& setAltitude(double altitude, bool quiet = false);

    GlobalReference& setCurrentPosition(const State& state, double latitude, double longitude);
    GlobalReference& setCurrentHeading(const State& state, double heading);
    GlobalReference& setCurrentAltitude(const State& state, double altitude);

    bool hasPosition() const { return has_position_; }
    bool hasHeading() const  { return has_heading_; }
    bool hasAltitude() const { return has_altitude_; }

    void fromWGS84(double latitude, double longitude, double &x, double &y);
    void toWGS84(double x, double y, double &latitude, double &longitude);
    void fromNorthEast(double north, double east, double &x, double &y);
    void toNorthEast(double x, double y, double &north, double &east);

    ParameterList& parameters();

    static const GlobalReferencePtr &Instance();

    void updated();
    void reset();

  private:
    GlobalReference();

    Position position_;
    Heading heading_;
    Radius radius_;

    bool has_position_;
    bool has_heading_;
    bool has_altitude_;

    ParameterList parameters_;
  };

} // namespace hector_pose_estimation

#endif // HECTOR_POSE_ESTIMATION_GLOBAL_REFERENCE_H
