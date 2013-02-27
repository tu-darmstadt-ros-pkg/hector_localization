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

#ifndef HECTOR_POSE_ESTIMATION_H
#define HECTOR_POSE_ESTIMATION_H

#include <hector_pose_estimation/types.h>
#include <hector_pose_estimation/state.h>
#include <hector_pose_estimation/system.h>
#include <hector_pose_estimation/measurement.h>
#include <hector_pose_estimation/parameters.h>

#include <boost/shared_ptr.hpp>
#include <vector>

#include <hector_pose_estimation/measurements/rate.h>
#include <hector_pose_estimation/measurements/gravity.h>
#include <hector_pose_estimation/measurements/zerorate.h>

namespace hector_pose_estimation {

class Filter;
class GlobalReference;

class PoseEstimation
{
public:
  PoseEstimation(const SystemPtr& system = SystemPtr());
  template <typename ConcreteSystemModel> PoseEstimation(ConcreteSystemModel *system_model);
  virtual ~PoseEstimation();

  static PoseEstimation *Instance();

  bool init();
  void cleanup();
  void reset();

  void update(ros::Time timestamp);
  void update(double dt);

  const SystemPtr& addSystem(const SystemPtr& system, const std::string& name = "system");
  const SystemPtr& addSystem(System *system) { return addSystem(SystemPtr(system)); }
  template <typename ConcreteSystemModel> const SystemPtr& addSystem(ConcreteSystemModel *model, const std::string& name = "system");
//  SystemPtr getSystem(std::size_t index = 0) const { return systems_.get(index); }
  SystemPtr getSystem(const std::string& name) const { return systems_.get(name); }

//  const SystemModel *getSystemModel(std::size_t index) const {
//    SystemPtr system = systems_.get(index);
//    if (!system) return 0;
//    return system->getModel();
//  }

  const SystemModel *getSystemModel(const std::string& name) const {
    SystemPtr system = systems_.get(name);
    if (!system) return 0;
    return system->getModel();
  }

  const MeasurementPtr& addMeasurement(const MeasurementPtr& measurement, const std::string& name = std::string());
  const MeasurementPtr& addMeasurement(Measurement *measurement) { return addMeasurement(MeasurementPtr(measurement)); }
  template <class ConcreteMeasurementModel> const MeasurementPtr& addMeasurement(ConcreteMeasurementModel *model, const std::string& name);
//  MeasurementPtr getMeasurement(std::size_t index) const { return measurements_.get(index); }
  MeasurementPtr getMeasurement(const std::string &name) const { return measurements_.get(name); }

  template <class InputType> boost::shared_ptr<InputType> registerInput(const std::string& name = std::string());
  template <class InputType> boost::shared_ptr<InputType> getInputType(const std::string& name) const { return inputs_.getType<InputType>(name); }

  InputPtr addInput(const InputPtr& input, const std::string& name = std::string());
  InputPtr addInput(Input *input) { return addInput(InputPtr(input)); }
  InputPtr setInput(const Input& input, std::string name = std::string());
//  InputPtr getInput(std::size_t index) const { return inputs_.get(index); }
  InputPtr getInput(const std::string& name) const { return inputs_.get(name); }

  virtual const State& state() const { return filter_->state(); }
  virtual State& state() { return filter_->state(); }

  virtual const State::Vector& getStateVector();
  virtual const State::Covariance& getCovariance();

  virtual SystemStatus getSystemStatus() const;
  virtual SystemStatus getMeasurementStatus() const;
  virtual bool inSystemStatus(SystemStatus test_status) const;
  virtual bool setSystemStatus(SystemStatus new_status);
  virtual bool setMeasurementStatus(SystemStatus new_status);
  virtual bool updateSystemStatus(SystemStatus set, SystemStatus clear);
  virtual bool updateMeasurementStatus(SystemStatus set, SystemStatus clear);

  virtual const GlobalReferencePtr &globalReference();

  virtual const ros::Time& getTimestamp() const;
  virtual void setTimestamp(const ros::Time& timestamp);

  virtual void getHeader(std_msgs::Header& header);
  virtual void getState(nav_msgs::Odometry& state, bool with_covariances = true);
  virtual void getPose(tf::Pose& pose);
  virtual void getPose(tf::Stamped<tf::Pose>& pose);
  virtual void getPose(geometry_msgs::Pose& pose);
  virtual void getPose(geometry_msgs::PoseStamped& pose);
  virtual void getPosition(tf::Point& point);
  virtual void getPosition(tf::Stamped<tf::Point>& point);
  virtual void getPosition(geometry_msgs::Point& pose);
  virtual void getPosition(geometry_msgs::PointStamped& pose);
  virtual void getGlobalPosition(double& latitude, double& longitude, double& altitude);
  virtual void getGlobalPosition(sensor_msgs::NavSatFix& global);
  virtual void getOrientation(tf::Quaternion& quaternion);
  virtual void getOrientation(tf::Stamped<tf::Quaternion>& quaternion);
  virtual void getOrientation(geometry_msgs::Quaternion& pose);
  virtual void getOrientation(geometry_msgs::QuaternionStamped& pose);
  virtual void getOrientation(double &yaw, double &pitch, double &roll);
  virtual void getImuWithBiases(geometry_msgs::Vector3& linear_acceleration, geometry_msgs::Vector3& angular_velocity);
  virtual void getVelocity(tf::Vector3& vector);
  virtual void getVelocity(tf::Stamped<tf::Vector3>& vector);
  virtual void getVelocity(geometry_msgs::Vector3& vector);
  virtual void getVelocity(geometry_msgs::Vector3Stamped& vector);
  virtual void getRate(tf::Vector3& vector);
  virtual void getRate(tf::Stamped<tf::Vector3>& vector);
  virtual void getRate(geometry_msgs::Vector3& vector);
  virtual void getRate(geometry_msgs::Vector3Stamped& vector);
  // virtual void getBias(tf::Vector3& angular_velocity, tf::Vector3& linear_acceleration);
  // virtual void getBias(tf::Stamped<tf::Vector3>& angular_velocity, tf::Stamped<tf::Vector3>& linear_acceleration);
  virtual void getBias(geometry_msgs::Vector3& angular_velocity, geometry_msgs::Vector3& linear_acceleration);
  virtual void getBias(geometry_msgs::Vector3Stamped& angular_velocity, geometry_msgs::Vector3Stamped& linear_acceleration);
  virtual void getTransforms(std::vector<tf::StampedTransform>& transforms);
  virtual void updateWorldToOtherTransform(tf::StampedTransform& world_to_other_transform);

  virtual ParameterList& parameters() { return parameters_; }
  virtual const ParameterList& parameters() const { return parameters_; }

  virtual boost::shared_ptr<Filter> filter() { return filter_; }
  virtual boost::shared_ptr<const Filter> filter() const { return filter_; }

  virtual void updated();

protected:
  Systems systems_;
  Measurements measurements_;
  Inputs inputs_;

private:
  boost::shared_ptr<Filter> filter_;

  ParameterList parameters_;

  ros::Time timestamp_;
  std::string world_frame_;
  std::string nav_frame_;
  std::string base_frame_;
  std::string stabilized_frame_;
  std::string footprint_frame_;
  std::string position_frame_;

  ros::Time alignment_start_;
  double alignment_time_;

  double gravity_;

  boost::shared_ptr<Rate> rate_update_;
  boost::shared_ptr<Gravity> gravity_update_;
  boost::shared_ptr<ZeroRate> zerorate_update_;
//  boost::shared_ptr<Heading> heading_update_;
};

template <typename ConcreteSystemModel>
PoseEstimation::PoseEstimation(ConcreteSystemModel *system_model) {
  *this = PoseEstimation(System::create(system_model));
}

template <typename ConcreteSystemModel>
const SystemPtr& PoseEstimation::addSystem(ConcreteSystemModel *model, const std::string& name)
{
  return addSystem(System::create(model, name));
}

template <class ConcreteMeasurementModel>
const MeasurementPtr& PoseEstimation::addMeasurement(ConcreteMeasurementModel *model, const std::string& name) {
  return addMeasurement(Measurement::create(model, name));
}

template <class InputType>
boost::shared_ptr<InputType> PoseEstimation::registerInput(const std::string& name) {
  boost::shared_ptr<InputType> input = getInputType<InputType>(name);
  if (input) return input;

  input.reset(new InputType());
  if (!addInput(boost::shared_static_cast<Input>(input), name)) input.reset();
  return input;
}

} // namespace hector_pose_estimation

#endif // HECTOR_POSE_ESTIMATION_H
