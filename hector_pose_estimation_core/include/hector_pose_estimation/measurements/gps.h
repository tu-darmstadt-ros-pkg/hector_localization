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

#ifndef HECTOR_POSE_ESTIMATION_GPS_H
#define HECTOR_POSE_ESTIMATION_GPS_H

#include <hector_pose_estimation/measurement.h>
#include <bfl/wrappers/matrix/matrix_wrapper.h>

namespace hector_pose_estimation {

class GPSModel : public MeasurementModel {
public:
  static const unsigned int MeasurementDimension = 4;
  typedef ColumnVector_<MeasurementDimension> MeasurementVector;
  typedef SymmetricMatrix_<MeasurementDimension> NoiseCovariance;

  GPSModel();
  virtual ~GPSModel();

  virtual bool init();
  virtual SystemStatus getStatusFlags() const;

  virtual ColumnVector ExpectedValueGet() const;
  virtual Matrix dfGet(unsigned int i) const;

protected:
  double position_stddev_;
  double velocity_stddev_;
};

struct GPSUpdate : public MeasurementUpdate {
  double latitude;
  double longitude;
  double velocity_north;
  double velocity_east;
};

class GPS : public Measurement_<GPSModel,GPSUpdate>
{
public:
  GPS(const std::string& name = "gps");
  virtual ~GPS();

  void onReset();

  GPSModel::MeasurementVector const& getValue(const GPSUpdate &update);
  bool beforeUpdate(PoseEstimation &estimator, const GPSUpdate &update);

private:
  double reference_latitude_;
  double reference_longitude_;
  double reference_heading_;
  bool has_reference_;

  double radius_north_, radius_east_;
  double cos_reference_heading_, sin_reference_heading_;

  void updateReference();

  GPSUpdate last_;
  GPSModel::MeasurementVector y_;
};

} // namespace hector_pose_estimation

#endif // HECTOR_POSE_ESTIMATION_GPS_H
