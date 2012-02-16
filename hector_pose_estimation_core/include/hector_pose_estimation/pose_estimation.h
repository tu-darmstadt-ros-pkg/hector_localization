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

#include "types.h"
#include "system.h"
#include "measurement.h"
#include "parameters.h"

#include <bfl/filter/extendedkalmanfilter.h>

#include <boost/shared_ptr.hpp>
#include <vector>

#include <boost/function.hpp>

#include <ros/time.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>

#include "measurements/gravity.h"
#include "measurements/zerorate.h"

namespace hector_pose_estimation {

class PoseEstimation
{
public:
	PoseEstimation(SystemModel *system_model = 0);
	virtual ~PoseEstimation();

	static PoseEstimation *Instance();

	void setSystemModel(SystemModel *system_model);
	const SystemModel *getSystemModel() const;

	bool init();
	void cleanup();
	void reset();

	void update(const InputVector& input, ros::Time timestamp);
	void update(double dt);

	Measurement *addMeasurement(Measurement *measurement);
	Measurement *addMeasurement(const std::string& name, Measurement *measurement);
	template <class ConcreteMeasurementModel>
	Measurement *addMeasurement(const std::string& name, ConcreteMeasurementModel *model) {
		return addMeasurement(new Measurement_<ConcreteMeasurementModel>(model, name));
	}

	Measurement *getMeasurement(const std::string &name) const;
	System *getSystem() const;

	const StateVector& getState();
	const StateCovariance& getCovariance();
	void setState(const StateVector& state);
	void setCovariance(const StateCovariance& covariance);

	SystemStatus getSystemStatus() const;
	SystemStatus getMeasurementStatus() const;
	bool setSystemStatus(SystemStatus new_status);
	bool setMeasurementStatus(SystemStatus new_status);
	bool updateSystemStatus(SystemStatus set, SystemStatus clear);
	bool updateMeasurementStatus(SystemStatus set, SystemStatus clear);

	typedef boost::function<bool(SystemStatus&)> SystemStatusCallback;
	void setSystemStatusCallback(SystemStatusCallback callback);

	ros::Time getTimestamp() const;
	void setTimestamp(ros::Time timestamp);

	void getHeader(std_msgs::Header& header);
	void getState(nav_msgs::Odometry& state, bool with_covariances = true);
	void getPose(tf::Pose& pose);
	void getPose(tf::Stamped<tf::Pose>& pose);
	void getPose(geometry_msgs::Pose& pose);
	void getPose(geometry_msgs::PoseStamped& pose);
	void getPosition(tf::Point& point);
	void getPosition(tf::Stamped<tf::Point>& point);
	void getPosition(geometry_msgs::Point& pose);
	void getPosition(geometry_msgs::PointStamped& pose);
	void getOrientation(tf::Quaternion& quaternion);
	void getOrientation(tf::Stamped<tf::Quaternion>& quaternion);
	void getOrientation(geometry_msgs::Quaternion& pose);
	void getOrientation(geometry_msgs::QuaternionStamped& pose);
	void getImuWithBiases(geometry_msgs::Vector3& linear_acceleration, geometry_msgs::Vector3& angular_velocity);
	void getVelocity(tf::Vector3& vector);
	void getVelocity(tf::Stamped<tf::Vector3>& vector);
	void getVelocity(geometry_msgs::Vector3& vector);
	void getVelocity(geometry_msgs::Vector3Stamped& vector);
	void getBias(tf::Vector3& angular_velocity, tf::Vector3& linear_acceleration);
	void getBias(tf::Stamped<tf::Vector3>& angular_velocity, tf::Stamped<tf::Vector3>& linear_acceleration);
	void getBias(geometry_msgs::Vector3& angular_velocity, geometry_msgs::Vector3& linear_acceleration);
	void getBias(geometry_msgs::Vector3Stamped& angular_velocity, geometry_msgs::Vector3Stamped& linear_acceleration);
	void getTransforms(std::vector<tf::StampedTransform>& transforms);

	ParameterList getParameters() const;

	ParameterList& parameters() { return parameters_; }
	const ParameterList& parameters() const { return parameters_; }

	BFL::KalmanFilter *filter() { return filter_; }
	const BFL::KalmanFilter *filter() const { return filter_; }

	void updated();

private:
	System *system_;
	typedef std::vector<Measurement *> Measurements;
	Measurements measurements_;
	BFL::ExtendedKalmanFilter *filter_;

	StateVector state_;
	StateCovariance covariance_;
	bool state_is_dirty_;
	bool covariance_is_dirty_;

	SystemStatus status_;
	SystemStatus measurement_status_;
	ParameterList parameters_;

	GlobalReference global_reference_;

	ros::Time timestamp_;
	std::string nav_frame_;
	std::string base_frame_;

	ros::Time alignment_start_;
	double alignment_time_;

	SystemStatusCallback status_callback_;

	Gravity gravity_;
	ZeroRate zerorate_;
};

} // namespace hector_pose_estimation

#endif // HECTOR_POSE_ESTIMATION_H
