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

#include <hector_pose_estimation/pose_estimation_node.h>

#include <hector_pose_estimation/system/generic_quaternion_system_model.h>
#include <hector_pose_estimation/measurements/poseupdate.h>
#include <hector_pose_estimation/measurements/height.h>
#include <hector_pose_estimation/measurements/magnetic.h>
#include <hector_pose_estimation/measurements/gps.h>

namespace hector_pose_estimation {

PoseEstimationNode::PoseEstimationNode(SystemModel *system_model)
  : pose_estimation_(new PoseEstimation(system_model ? system_model : new GenericQuaternionSystemModel))
  , private_nh_("~")
{
  pose_estimation_->addMeasurement(new PoseUpdate("poseupdate"));
  pose_estimation_->addMeasurement(new Height("height"));
  pose_estimation_->addMeasurement(new Magnetic("magnetic"));
  pose_estimation_->addMeasurement(new GPS("gps"));
}

PoseEstimationNode::~PoseEstimationNode()
{
  cleanup();
  delete pose_estimation_;
}

bool PoseEstimationNode::init() {
  pose_estimation_->getParameters().registerParamsRos(getPrivateNodeHandle());

  if (!pose_estimation_->init()) {
    ROS_ERROR("Intitialization of pose estimation failed!");
    return false;
  }

  imu_subscriber_        = getNodeHandle().subscribe("raw_imu", 10, &PoseEstimationNode::imuCallback, this);
  baro_subscriber_       = getNodeHandle().subscribe("pressure_height", 10, &PoseEstimationNode::heightCallback, this);
  magnetic_subscriber_   = getNodeHandle().subscribe("magnetic", 10, &PoseEstimationNode::magneticCallback, this);

  gps_subscriber_.subscribe(getNodeHandle(), "fix", 10);
  gps_velocity_subscriber_.subscribe(getNodeHandle(), "fix_velocity", 10);
  gps_synchronizer_ = new message_filters::TimeSynchronizer<sensor_msgs::NavSatFix,geometry_msgs::Vector3Stamped>(gps_subscriber_, gps_velocity_subscriber_, 10);
  gps_synchronizer_->registerCallback(&PoseEstimationNode::gpsCallback, this);

  state_publisher_       = getNodeHandle().advertise<nav_msgs::Odometry>("state", 10, false);
  pose_publisher_        = getNodeHandle().advertise<geometry_msgs::PoseStamped>("pose", 10, false);
  velocity_publisher_    = getNodeHandle().advertise<geometry_msgs::Vector3Stamped>("velocity", 10, false);
  imu_publisher_         = getNodeHandle().advertise<sensor_msgs::Imu>("imu", 10, false);
  global_publisher_      = getNodeHandle().advertise<sensor_msgs::NavSatFix>("global", 10, false);

  angular_velocity_bias_publisher_    = getNodeHandle().advertise<geometry_msgs::Vector3Stamped>("angular_velocity_bias", 10, false);
  linear_acceleration_bias_publisher_ = getNodeHandle().advertise<geometry_msgs::Vector3Stamped>("linear_acceleration_bias", 10, false);
  gps_pose_publisher_                 = getNodeHandle().advertise<geometry_msgs::PoseStamped>("fix/pose", 10, false);

  poseupdate_subscriber_  = getNodeHandle().subscribe("poseupdate", 10, &PoseEstimationNode::poseupdateCallback, this);
  twistupdate_subscriber_ = getNodeHandle().subscribe("twistupdate", 10, &PoseEstimationNode::twistupdateCallback, this);
  syscommand_subscriber_  = getNodeHandle().subscribe("syscommand", 10, &PoseEstimationNode::syscommandCallback, this);

  getPrivateNodeHandle().param("with_covariances", with_covariances_, false);

  // publish initial state
  publish();

  return true;
}

void PoseEstimationNode::reset() {
  pose_estimation_->reset();
}

void PoseEstimationNode::cleanup() {
  pose_estimation_->cleanup();
  if (gps_synchronizer_) {
    delete gps_synchronizer_;
    gps_synchronizer_ = 0;
  }
}

void PoseEstimationNode::imuCallback(const sensor_msgs::ImuConstPtr& imu) {
  InputVector input(InputDimension);
  input(ACCEL_X) = imu->linear_acceleration.x;
  input(ACCEL_Y) = imu->linear_acceleration.y;
  input(ACCEL_Z) = imu->linear_acceleration.z;
  input(GYRO_X)  = imu->angular_velocity.x;
  input(GYRO_Y)  = imu->angular_velocity.y;
  input(GYRO_Z)  = imu->angular_velocity.z;

  pose_estimation_->update(input, imu->header.stamp);
  publish();
}

#ifdef USE_MAV_MSGS
void PoseEstimationNode::heightCallback(const mav_msgs::HeightConstPtr& height) {
  Height::MeasurementVector update(1);
  update = height->height;
  pose_estimation_->getMeasurement("height")->add(Height::Update(update));
}
#else
void PoseEstimationNode::heightCallback(const geometry_msgs::PointStampedConstPtr& height) {
  Height::MeasurementVector update(1);
  update = height->point.z;
  pose_estimation_->getMeasurement("height")->add(Height::Update(update));
}
#endif

void PoseEstimationNode::magneticCallback(const geometry_msgs::Vector3StampedConstPtr& magnetic) {
  Magnetic::MeasurementVector update(3);
  update(1) = magnetic->vector.x;
  update(2) = magnetic->vector.y;
  update(3) = magnetic->vector.z;
  pose_estimation_->getMeasurement("magnetic")->add(Magnetic::Update(update));
}

void PoseEstimationNode::gpsCallback(const sensor_msgs::NavSatFixConstPtr& gps, const geometry_msgs::Vector3StampedConstPtr& gps_velocity) {
  if (gps->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) return;
  GPS::Update update;
  update.latitude = gps->latitude * M_PI/180.0;
  update.longitude = gps->longitude * M_PI/180.0;
  update.velocity_north =  gps_velocity->vector.x;
  update.velocity_east  = -gps_velocity->vector.y;
  pose_estimation_->getMeasurement("gps")->add(update);

  if (gps_pose_publisher_.getNumSubscribers() > 0) {
    geometry_msgs::PoseStamped gps_pose;
    pose_estimation_->getHeader(gps_pose.header);
    gps_pose.header.seq = gps->header.seq;
    gps_pose.header.stamp = gps->header.stamp;
    GPSModel::MeasurementVector y = static_cast<GPS *>(pose_estimation_->getMeasurement("gps"))->getValue(update);
    gps_pose.pose.position.x = y(1);
    gps_pose.pose.position.y = y(2);
    gps_pose.pose.orientation.w = 1.0;
    gps_pose_publisher_.publish(gps_pose);
  }
}

void PoseEstimationNode::poseupdateCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose) {
  pose_estimation_->getMeasurement("poseupdate")->add(PoseUpdate::Update(pose));
}

void PoseEstimationNode::twistupdateCallback(const geometry_msgs::TwistWithCovarianceStampedConstPtr& twist) {
  pose_estimation_->getMeasurement("poseupdate")->add(PoseUpdate::Update(twist));
}

void PoseEstimationNode::syscommandCallback(const std_msgs::StringConstPtr& syscommand) {
  if (syscommand->data == "reset") {
    ROS_INFO("Resetting pose_estimation");
    pose_estimation_->reset();
    publish();
  }
}

void PoseEstimationNode::publish() {
  if (state_publisher_) {
    nav_msgs::Odometry state;
    pose_estimation_->getState(state, with_covariances_);
    state_publisher_.publish(state);
  }

  if (pose_publisher_) {
    geometry_msgs::PoseStamped pose_msg;
    pose_estimation_->getPose(pose_msg);
    pose_publisher_.publish(pose_msg);
  }

  if (imu_publisher_) {
    sensor_msgs::Imu imu_msg;
    pose_estimation_->getHeader(imu_msg.header);
    pose_estimation_->getOrientation(imu_msg.orientation);
    pose_estimation_->getImuWithBiases(imu_msg.linear_acceleration, imu_msg.angular_velocity);
    imu_publisher_.publish(imu_msg);
  }

  if (velocity_publisher_) {
    geometry_msgs::Vector3Stamped velocity_msg;
    pose_estimation_->getVelocity(velocity_msg);
    velocity_publisher_.publish(velocity_msg);
  }

  if (global_publisher_) {
    sensor_msgs::NavSatFix global_msg;
    pose_estimation_->getGlobalPosition(global_msg);
    global_publisher_.publish(global_msg);
  }

  if (angular_velocity_bias_publisher_ || linear_acceleration_bias_publisher_) {
    geometry_msgs::Vector3Stamped angular_velocity_msg, linear_acceleration_msg;
    pose_estimation_->getBias(angular_velocity_msg, linear_acceleration_msg);
    if (angular_velocity_bias_publisher_) angular_velocity_bias_publisher_.publish(angular_velocity_msg);
    if (linear_acceleration_bias_publisher_) linear_acceleration_bias_publisher_.publish(linear_acceleration_msg);
  }

  // if (transform_broadcaster_)
  {
    std::vector<tf::StampedTransform> transforms(3);
    pose_estimation_->getTransforms(transforms);
    transform_broadcaster_.sendTransform(transforms);
  }
}

} // namespace hector_pose_estimation
