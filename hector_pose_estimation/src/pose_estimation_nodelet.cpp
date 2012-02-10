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

#include <hector_pose_estimation/pose_estimation.h>
#include <hector_pose_estimation/nodelet.h>

#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <mav_msgs/Height.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf/transform_broadcaster.h>

#include <hector_pose_estimation/system/generic_quaternion_system_model.h>
#include <hector_pose_estimation/measurements/poseupdate.h>
#include <hector_pose_estimation/measurements/height.h>
#include <hector_pose_estimation/measurements/magnetic.h>
#include <hector_pose_estimation/measurements/gps.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

namespace hector_pose_estimation {

class PoseEstimationNodelet : public Nodelet {
private:
  ros::Subscriber imu_subscriber_, baro_subscriber_, magnetic_subscriber_;
  message_filters::Subscriber<sensor_msgs::NavSatFix> gps_subscriber_;
  message_filters::Subscriber<geometry_msgs::Vector3Stamped> gps_velocity_subscriber_;
  message_filters::TimeSynchronizer<sensor_msgs::NavSatFix,geometry_msgs::Vector3Stamped> *gps_synchronizer_;
  ros::Publisher pose_publisher_, orientation_publisher_, velocity_publisher_, imu_publisher_;
  ros::Publisher angular_velocity_bias_publisher_, linear_acceleration_bias_publisher_;
  ros::Subscriber poseupdate_subscriber_;
  ros::Subscriber syscommand_subscriber_;
  tf::TransformBroadcaster transform_broadcaster_;

private:
  void onInit() {
    pose_estimation_->setSystemModel(new GenericQuaternionSystemModel);
    pose_estimation_->addMeasurement(new PoseUpdate("poseupdate"));
    pose_estimation_->addMeasurement(new Height("height"));
    pose_estimation_->addMeasurement(new Magnetic("magnetic"));
    pose_estimation_->addMeasurement(new GPS("gps"));

    if (!pose_estimation_->init()) {
      ROS_ERROR("Intitialization of pose estimation failed!");
      return;
    }

    imu_subscriber_        = getNodeHandle().subscribe("raw_imu", 10, &PoseEstimationNodelet::imuCallback, this);
    baro_subscriber_       = getNodeHandle().subscribe("height", 10, &PoseEstimationNodelet::heightCallback, this);
    magnetic_subscriber_   = getNodeHandle().subscribe("magnetic", 10, &PoseEstimationNodelet::magneticCallback, this);

    gps_subscriber_.subscribe(getNodeHandle(), "fix", 10);
    gps_velocity_subscriber_.subscribe(getNodeHandle(), "gps_velocity", 10);
    gps_synchronizer_ = new message_filters::TimeSynchronizer<sensor_msgs::NavSatFix,geometry_msgs::Vector3Stamped>(gps_subscriber_, gps_velocity_subscriber_, 10);
    gps_synchronizer_->registerCallback(&PoseEstimationNodelet::gpsCallback, this);

    pose_publisher_        = getNodeHandle().advertise<geometry_msgs::PoseStamped>("pose", 10, false);
    orientation_publisher_ = getNodeHandle().advertise<geometry_msgs::QuaternionStamped>("orientation", 10, false);
    velocity_publisher_    = getNodeHandle().advertise<geometry_msgs::Vector3Stamped>("velocity", 10, false);
    imu_publisher_         = getNodeHandle().advertise<sensor_msgs::Imu>("imu", 10, false);

    angular_velocity_bias_publisher_    = getNodeHandle().advertise<geometry_msgs::Vector3Stamped>("angular_velocity_bias", 10, false);
    linear_acceleration_bias_publisher_ = getNodeHandle().advertise<geometry_msgs::Vector3Stamped>("linear_acceleration_bias", 10, false);

    poseupdate_subscriber_ = getNodeHandle().subscribe("poseupdate", 10, &PoseEstimationNodelet::poseupdateCallback, this);
    syscommand_subscriber_ = getNodeHandle().subscribe("syscommand", 10, &PoseEstimationNodelet::syscommandCallback, this);

    pose_estimation_->getParameters().registerParams(getPrivateNodeHandle());

    // publish initial state
    publish();
  }

  void onReset() {
    pose_estimation_->reset();
  }

  void onCleanup() {
    pose_estimation_->cleanup();
    delete gps_synchronizer_;
  }

  void imuCallback(const sensor_msgs::ImuConstPtr& imu) {
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

  void heightCallback(const mav_msgs::HeightConstPtr& height) {
    Height::MeasurementVector update(1);
    update = height->height;
    pose_estimation_->getMeasurement("height")->add(Height::Update(update));
  }

  void magneticCallback(const geometry_msgs::Vector3StampedConstPtr& magnetic) {
    Magnetic::MeasurementVector update(3);
    update(1) = magnetic->vector.x;
    update(2) = magnetic->vector.y;
    update(3) = magnetic->vector.z;
    pose_estimation_->getMeasurement("magnetic")->add(Magnetic::Update(update));
  }

  void gpsCallback(const sensor_msgs::NavSatFixConstPtr& gps, const geometry_msgs::Vector3StampedConstPtr& gps_velocity) {
    if (gps->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) return;
    GPS::Update update;
    update.latitude = gps->latitude * M_PI/180.0;
    update.longitude = gps->longitude * M_PI/180.0;
    update.velocity_north =  gps_velocity->vector.x;
    update.velocity_east  = -gps_velocity->vector.y;
    pose_estimation_->getMeasurement("gps")->add(update);
  }

  void poseupdateCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose) {
    pose_estimation_->getMeasurement("poseupdate")->add(PoseUpdate::Update(pose));
  }

  void syscommandCallback(const std_msgs::StringConstPtr& syscommand) {
    if (syscommand->data == "reset") {
      ROS_INFO("Resetting pose_estimation");
      pose_estimation_->reset();
      publish();
    }
  }

  void publish() {
    if (pose_publisher_) {
      tf::Stamped<tf::Pose> pose_tf;
      pose_estimation_->getPose(pose_tf);
      geometry_msgs::PoseStamped pose_msg;
      tf::poseStampedTFToMsg(pose_tf, pose_msg);
      pose_publisher_.publish(pose_msg);
    }

    tf::Stamped<tf::Quaternion> quaternion_tf;
    if (orientation_publisher_ || imu_publisher_) {
      pose_estimation_->getOrientation(quaternion_tf);
    }

    if (orientation_publisher_) {
      geometry_msgs::QuaternionStamped quaternion_msg;
      tf::quaternionStampedTFToMsg(quaternion_tf, quaternion_msg);
      orientation_publisher_.publish(quaternion_msg);
    }

    if (imu_publisher_) {
      sensor_msgs::Imu imu_msg;
      imu_msg.header.stamp = quaternion_tf.stamp_;
      imu_msg.header.frame_id = quaternion_tf.frame_id_;
      tf::quaternionTFToMsg(quaternion_tf, imu_msg.orientation);
      pose_estimation_->getImuWithBiases(imu_msg.linear_acceleration, imu_msg.angular_velocity);
      imu_publisher_.publish(imu_msg);
    }

    if (velocity_publisher_) {
      tf::Stamped<tf::Vector3> velocity_tf;
      pose_estimation_->getVelocity(velocity_tf);
      geometry_msgs::Vector3Stamped velocity_msg;
      tf::vector3StampedTFToMsg(velocity_tf, velocity_msg);
      velocity_publisher_.publish(velocity_msg);
    }

    if (angular_velocity_bias_publisher_ || linear_acceleration_bias_publisher_) {
      tf::Stamped<tf::Vector3> angular_velocity_tf, linear_acceleration_tf;
      pose_estimation_->getBias(angular_velocity_tf, linear_acceleration_tf);

      if (angular_velocity_bias_publisher_) {
        geometry_msgs::Vector3Stamped angular_velocity_msg;
        tf::vector3StampedTFToMsg(angular_velocity_tf, angular_velocity_msg);
        angular_velocity_bias_publisher_.publish(angular_velocity_msg);
      }

      if (linear_acceleration_bias_publisher_) {
        geometry_msgs::Vector3Stamped linear_acceleration_msg;
        tf::vector3StampedTFToMsg(linear_acceleration_tf, linear_acceleration_msg);
        linear_acceleration_bias_publisher_.publish(linear_acceleration_msg);
      }
    }

    // if (transform_broadcaster_)
    {
      std::vector<tf::StampedTransform> transforms(3);
      pose_estimation_->getTransforms(transforms);
      transform_broadcaster_.sendTransform(transforms);
    }
  }
};

} // namespace hector_pose_estimation

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(hector_pose_estimation, PoseEstimation, hector_pose_estimation::PoseEstimationNodelet, nodelet::Nodelet)
