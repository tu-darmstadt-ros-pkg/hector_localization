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

#ifndef HECTOR_POSE_ESTIMATION_POSEUPDATE_H
#define HECTOR_POSE_ESTIMATION_POSEUPDATE_H

#include <hector_pose_estimation/measurement.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace hector_pose_estimation {

class PositionXYModel : public MeasurementModel {
public:
  PositionXYModel() : MeasurementModel(2) {}
  virtual ~PositionXYModel() {}

  virtual ColumnVector ExpectedValueGet() const;
  virtual Matrix dfGet(unsigned int i) const;
};

class PositionZModel : public MeasurementModel {
public:
  PositionZModel() : MeasurementModel(1) {}
  virtual ~PositionZModel() {}

  virtual ColumnVector ExpectedValueGet() const;
  virtual Matrix dfGet(unsigned int i) const;
};

class YawModel : public MeasurementModel {
public:
  YawModel() : MeasurementModel(1) {}
  virtual ~YawModel() {}

  virtual ColumnVector ExpectedValueGet() const;
  virtual Matrix dfGet(unsigned int i) const;
};

class PoseUpdate : public Measurement
{
public:
  PoseUpdate(const std::string& name = "poseupdate");
  virtual ~PoseUpdate();

  class Update : public MeasurementUpdate {
  public:
    Update() {}
    Update(const geometry_msgs::PoseWithCovarianceStamped& pose) : pose(new geometry_msgs::PoseWithCovarianceStamped(pose)) {}
    Update(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose) : pose(pose) {}
    geometry_msgs::PoseWithCovarianceStampedConstPtr pose;
  };

  bool update(PoseEstimation &estimator, const MeasurementUpdate &update);

private:
  PositionXYModel position_xy_model_;
  PositionZModel position_z_model_;
  YawModel yaw_model_;

  double alpha_, beta_;
  double fixed_position_xy_stddev_;
  double fixed_position_z_stddev_;
  double fixed_yaw_stddev_;

  double calculateOmega(const SymmetricMatrix &Ix, const SymmetricMatrix &Iy) const;
  double updateInternal(const SymmetricMatrix &Px, const ColumnVector &x, const SymmetricMatrix &Iy, const ColumnVector &i, const Matrix &H, SymmetricMatrix &Pxy, ColumnVector &xy);

protected:
  typedef Queue_<Update> Queue;
  Queue queue_;
  virtual Queue& queue() { return queue_; }
};

} // namespace hector_pose_estimation

#endif // HECTOR_POSE_ESTIMATION_POSEUPDATE_H
