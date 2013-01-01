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

#ifndef HECTOR_POSE_ESTIMATION_SYSTEM_MODEL_H
#define HECTOR_POSE_ESTIMATION_SYSTEM_MODEL_H

#include <hector_pose_estimation/types.h>
#include <hector_pose_estimation/parameters.h>

#include <bfl/model/analyticsystemmodel_gaussianuncertainty.h>
#include <bfl/pdf/analyticconditionalgaussian_additivenoise.h>
#include <bfl/pdf/gaussian.h>

#include <string>

namespace hector_pose_estimation {

class PoseEstimation;

class SystemModel : public BFL::AnalyticConditionalGaussianAdditiveNoise, public BFL::AnalyticSystemModelGaussianUncertainty {
public:
  SystemModel();
  virtual ~SystemModel();

  virtual std::string getName() const { return std::string(); }

  virtual bool init() { return true; }
  virtual void cleanup() { }
  virtual void reset() { }

  ParameterList& parameters() { return parameters_; }
  const ParameterList& parameters() const { return parameters_; }

  void set_dt(double dt) { dt_ = dt; }
  double get_dt() const { return dt_; }

  virtual void setMeasurementStatus(const SystemStatus& status) { measurement_status_ = status; }
  virtual SystemStatus getStatusFlags() const { return measurement_status_; }

  virtual void getPrior(BFL::Gaussian &prior) const;

  virtual SymmetricMatrix CovarianceGet(double dt) const;
  virtual SymmetricMatrix CovarianceGet() const {
    return CovarianceGet(dt_);
  }

  virtual ColumnVector ExpectedValueGet(double dt) const {
    return ExpectedValueGet();
  }
  virtual ColumnVector ExpectedValueGet() const {
    return ExpectedValueGet(dt_);
  }

  virtual Matrix dfGet(unsigned int i, double dt) const {
    return dfGet(i);
  }
  virtual Matrix dfGet(unsigned int i) const {
    return dfGet(i, dt_);
  }

  virtual void Limit(StateVector& x) const {
  }

  virtual double getGravity() const { return 0.0; }

protected:
  double dt_;
  ParameterList parameters_;

protected:
  const StateVector& x_;
  const ColumnVector& u_;
  mutable StateVector x_pred_;
  mutable Matrix A_;
  SystemStatus measurement_status_;
};

} // namespace hector_pose_estimation

#endif // HECTOR_POSE_ESTIMATION_SYSTEM_MODEL_H
