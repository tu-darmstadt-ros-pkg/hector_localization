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

#include <hector_pose_estimation/system.h>
#include <hector_pose_estimation/pose_estimation.h>
#include <bfl/filter/kalmanfilter.h>

namespace hector_pose_estimation {

System::System(const std::string &name)
  : name_(name)
  , status_flags_(0)
  , prior_(StateDimension)
{
}

System::~System() {
}

BFL::Gaussian *System::getPrior()
{
  getModel()->getPrior(prior_);
  return &prior_;
}

bool System::init()
{
  if (!getModel()) return false;
  return getModel()->init();
}

void System::cleanup()
{
  getModel()->cleanup();
}

void System::reset()
{
  getModel()->reset();
}

void System::updateInternal(PoseEstimation &estimator, double dt, ColumnVector const& u) {
  ROS_DEBUG("Updating with system model %s", getName().c_str());

//  std::cout << "     dt = " << dt << ", u = [" << u.transpose() << "]" << std::endl;

  getModel()->set_dt(dt);
  getModel()->setMeasurementStatus(estimator.getMeasurementStatus());
  estimator.filter()->Update(getModel(), u);
  updated();
  estimator.updated();

//  std::cout << "x_pred = [" << estimator.getState().transpose() << "]" << std::endl;
//  std::cout << "P_pred = [" << estimator.getCovariance() << "]" << std::endl;
}

void System::updated() {
  if (getModel()) status_flags_ = getModel()->getStatusFlags();
}

StateVector System::limitState(StateVector state) const {
  getModel()->Limit(state);
  return state;
}

} // namespace hector_pose_estimation
