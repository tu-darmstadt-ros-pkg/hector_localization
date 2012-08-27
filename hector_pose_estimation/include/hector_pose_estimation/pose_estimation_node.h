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

#ifndef HECTOR_POSE_ESTIMATION_NODE_H
#define HECTOR_POSE_ESTIMATION_NODE_H

#include <hector_pose_estimation/pose_estimation.h>

#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#ifdef USE_MAV_MSGS
  #include <mav_msgs/Height.h>
#else
  #include <geometry_msgs/PointStamped.h>
#endif
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

namespace hector_pose_estimation {

class PoseEstimationNode {
public:
  PoseEstimationNode(SystemModel* system_model = 0);
  virtual ~PoseEstimationNode();

  virtual bool init();
  virtual void reset();
  virtual void cleanup();

  virtual void publish();

protected:
  void imuCallback(const sensor_msgs::ImuConstPtr& imu);

#ifdef USE_MAV_MSGS
  void heightCallback(const mav_msgs::HeightConstPtr& height);
#else
  void heightCallback(const geometry_msgs::PointStampedConstPtr& height);
#endif

  void magneticCallback(const geometry_msgs::Vector3StampedConstPtr& magnetic);
  void gpsCallback(const sensor_msgs::NavSatFixConstPtr& gps, const geometry_msgs::Vector3StampedConstPtr& gps_velocity);
  void poseupdateCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose);
  void twistupdateCallback(const geometry_msgs::TwistWithCovarianceStampedConstPtr& twist);
  void syscommandCallback(const std_msgs::StringConstPtr& syscommand);

  virtual ros::NodeHandle& getNodeHandle() { return nh_; }
  virtual ros::NodeHandle& getPrivateNodeHandle() { return private_nh_; }

protected:
  PoseEstimation *pose_estimation_;

private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Subscriber imu_subscriber_, baro_subscriber_, magnetic_subscriber_;
  message_filters::Subscriber<sensor_msgs::NavSatFix> gps_subscriber_;
  message_filters::Subscriber<geometry_msgs::Vector3Stamped> gps_velocity_subscriber_;
  message_filters::TimeSynchronizer<sensor_msgs::NavSatFix,geometry_msgs::Vector3Stamped> *gps_synchronizer_;
  ros::Publisher state_publisher_, pose_publisher_, velocity_publisher_, imu_publisher_, global_publisher_;
  ros::Publisher angular_velocity_bias_publisher_, linear_acceleration_bias_publisher_, gps_pose_publisher_;
  ros::Subscriber poseupdate_subscriber_, twistupdate_subscriber_;
  ros::Subscriber syscommand_subscriber_;
  tf::TransformBroadcaster transform_broadcaster_;

  bool with_covariances_;
};

} // namespace hector_pose_estimation

#endif // HECTOR_POSE_ESTIMATION_NODE_H
