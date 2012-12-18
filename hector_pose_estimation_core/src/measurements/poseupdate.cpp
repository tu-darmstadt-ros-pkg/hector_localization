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

#include <hector_pose_estimation/measurements/poseupdate.h>
#include <hector_pose_estimation/pose_estimation.h>
#include <hector_pose_estimation/bfl_conversions.h>
#include <Eigen/Geometry>

namespace hector_pose_estimation {

PoseUpdate::PoseUpdate(const std::string& name)
  : Measurement(name)
{
  fixed_alpha_ = 0.0;
  fixed_beta_  = 0.0;
  interpret_covariance_as_information_matrix_ = true;

  fixed_position_xy_stddev_ = 0.0;
  fixed_position_z_stddev_ = 0.0;
  fixed_yaw_stddev_ = 0.0;

  fixed_velocity_xy_stddev_ = 0.0;
  fixed_velocity_z_stddev_ = 0.0;
  fixed_angular_rate_xy_stddev_ = 0.0;
  fixed_angular_rate_z_stddev_ = 0.0;

  max_time_difference_ = 1.0;
  max_position_xy_error_ = 3.0; // 3 sigma
  max_position_z_error_ = 3.0; // 3 sigma
  max_yaw_error_ = 3.0; // 3 sigma

  max_velocity_xy_error_ = 3.0; // 3 sigma
  max_velocity_z_error_ = 3.0; // 3 sigma
  max_angular_rate_xy_error_ = 3.0; // 3 sigma
  max_angular_rate_z_error_ = 3.0; // 3 sigma

  jump_on_max_error_ = true;

  parameters().add("fixed_alpha", fixed_alpha_);
  parameters().add("fixed_beta", fixed_beta_);
  parameters().add("interpret_covariance_as_information_matrix", interpret_covariance_as_information_matrix_);

  parameters().add("fixed_position_xy_stddev", fixed_position_xy_stddev_);
  parameters().add("fixed_position_z_stddev", fixed_position_z_stddev_);
  parameters().add("fixed_yaw_stddev", fixed_yaw_stddev_);
  parameters().add("fixed_velocity_xy_stddev", fixed_velocity_xy_stddev_);
  parameters().add("fixed_velocity_z_stddev", fixed_velocity_z_stddev_);
  parameters().add("fixed_angular_rate_xy_stddev", fixed_angular_rate_xy_stddev_);
  parameters().add("fixed_angular_rate_z_stddev", fixed_angular_rate_z_stddev_);
  parameters().add("max_time_difference", max_time_difference_);
  parameters().add("max_position_xy_error", max_position_xy_error_ );
  parameters().add("max_position_z_error", max_position_z_error_);
  parameters().add("max_yaw_error", max_yaw_error_);
  parameters().add("max_velocity_xy_error", max_velocity_xy_error_ );
  parameters().add("max_velocity_z_error", max_velocity_z_error_);
  parameters().add("max_angular_rate_xy_error", max_angular_rate_xy_error_ );
  parameters().add("max_angular_rate_z_error", max_angular_rate_z_error_);
  parameters().add("jump_on_max_error", jump_on_max_error_);
}

PoseUpdate::~PoseUpdate()
{
}

bool PoseUpdate::update(PoseEstimation &estimator, const MeasurementUpdate &update_) {
  Update const &update = static_cast<Update const &>(update_);

  // fetch current state
  ColumnVector state = estimator.getState();
  SymmetricMatrix covariance = estimator.getCovariance();

  while (update.pose) {
    // convert incoming update information to Eigen
    Eigen::Vector3d update_pose(update.pose->pose.pose.position.x, update.pose->pose.pose.position.y, update.pose->pose.pose.position.z);
    Eigen::Quaterniond update_orientation(update.pose->pose.pose.orientation.w, update.pose->pose.pose.orientation.x, update.pose->pose.pose.orientation.y, update.pose->pose.pose.orientation.z);
    Eigen::Vector3d update_euler(update_orientation.toRotationMatrix().eulerAngles(2,1,0));

    // information is the information matrix if interpret_covariance_as_information_matrix_ is true and a covariance matrix otherwise
    // zero elements are counted as zero information in any case
    SymmetricMatrix information(6);
    covarianceMsgToBfl(update.pose->pose.covariance, information);

    // forward state vector to the individual measurement models
    position_xy_model_.ConditionalArgumentSet(0,state);
    position_z_model_.ConditionalArgumentSet(0,state);
    yaw_model_.ConditionalArgumentSet(0,state);

    ROS_DEBUG_STREAM_NAMED("poseupdate", "PoseUpdate: state = [ " << state.transpose() << " ], P = [ " << covariance << " ]" << std::endl
                                      << "update: pose = [ " << update_pose.transpose() << " ], euler = [ " << update_euler.transpose() << " ], information = [ " << information << " ]");
    ROS_DEBUG_STREAM_NAMED("poseupdate", "dt = " << (estimator.getTimestamp() - update.pose->header.stamp).toSec() << " s");

    // predict update pose using the estimated velocity and degrade information
    if (!update.pose->header.stamp.isZero()) {
      double dt = (estimator.getTimestamp() - update.pose->header.stamp).toSec();
      if (dt < 0.0) {
        ROS_DEBUG_STREAM_NAMED("poseupdate", "Ignoring pose update as it has a negative time difference: dt = " << dt << "s");
        break;

      } else if (max_time_difference_ > 0.0 && dt >= max_time_difference_) {
        ROS_DEBUG_STREAM_NAMED("poseupdate", "Ignoring pose update as the time difference is too large: dt = " << dt << "s");
        break;

      } else if (max_time_difference_ > 0.0){
        if (interpret_covariance_as_information_matrix_)
          information = information * (1.0 - dt/max_time_difference_);
        else
          information = information / (1.0 - dt/max_time_difference_);
      }

      Eigen::Vector3d state_velocity(state.sub(VELOCITY_X, VELOCITY_Z));
      update_pose = update_pose + dt * state_velocity;
  #ifdef USE_RATE_SYSTEM_MODEL
      Eigen::Vector3d state_rate(state.sub(RATE_X,RATE_Z));
      Eigen::AngleAxisd state_angle_offset(state_rate.norm() * dt, state_rate.normalized());
      update_orientation = state_angle_offset * update_orientation;
  #endif
    }

    // update PositionXY
    if (information(1,1) > 0.0 || information(2,2) > 0.0) {
      // fetch observation matrix H
      Matrix H = position_xy_model_.dfGet(0);
      ColumnVector x(position_xy_model_.ExpectedValueGet());
      ColumnVector y(2);
      SymmetricMatrix Iy(information.sub(1,2,1,2));
      y(1) = update_pose.x();
      y(2) = update_pose.y();

      // invert Iy if information is a covariance matrix
      if (!interpret_covariance_as_information_matrix_) Iy = Iy.inverse();

      // fixed_position_xy_stddev_ = 1.0;
      if (fixed_position_xy_stddev_ != 0.0) {
        Iy = 0.0;
        Iy(1,1) = Iy(2,2) = 1.0 / (fixed_position_xy_stddev_*fixed_position_xy_stddev_);
      }

      ROS_DEBUG_STREAM_NAMED("poseupdate", "Position Update: ");
      ROS_DEBUG_STREAM_NAMED("poseupdate", "      x = [" << x.transpose() << "], H = [ " << H << " ], Px = [" <<  (H*covariance*H.transpose()) << "], Ix = [ " << (H*covariance*H.transpose()).inverse() << "]");
      ROS_DEBUG_STREAM_NAMED("poseupdate", "      y = [" << y.transpose() << "], Iy = [ " << Iy << " ]");
      double innovation = updateInternal(covariance, state, Iy, y - x, H, covariance, state, "position_xy", max_position_xy_error_);
      ROS_DEBUG_STREAM_NAMED("poseupdate", " ==> xy = [" << position_xy_model_.PredictionGet(ColumnVector(), state).transpose() << "], Pxy = [ " << (H*covariance*H.transpose()) << " ], innovation = " << innovation);

      status_flags_ |= STATE_XY_POSITION;
    }

    // update PositionZ
    if (information(3,3) > 0.0) {
      // fetch observation matrix H
      Matrix H = position_z_model_.dfGet(0);
      ColumnVector x(position_z_model_.ExpectedValueGet());
      ColumnVector y(1); y(1) =  update_pose.z();
      SymmetricMatrix Iy(information.sub(3,3,3,3));

      // invert Iy if information is a covariance matrix
      if (!interpret_covariance_as_information_matrix_) Iy = Iy.inverse();

      // fixed_position_z_stddev_ = 1.0;
      if (fixed_position_z_stddev_ != 0.0) {
        Iy = 0.0;
        Iy(1,1) = 1.0 / (fixed_position_z_stddev_*fixed_position_z_stddev_);
      }

      ROS_DEBUG_STREAM_NAMED("poseupdate", "Height Update: ");
      ROS_DEBUG_STREAM_NAMED("poseupdate", "      x = " << x(1) << ", H = [ " << H << " ], Px = [" <<  (H*covariance*H.transpose()) << "], Ix = [ " << (H*covariance*H.transpose()).inverse() << "]");
      ROS_DEBUG_STREAM_NAMED("poseupdate", "      y = " << y(1) << ", Iy = [ " << Iy << " ]");
      double innovation = updateInternal(covariance, state, Iy, y - x, H, covariance, state, "position_z", max_position_z_error_);
      ROS_DEBUG_STREAM_NAMED("poseupdate", " ==> xy = " << position_z_model_.PredictionGet(ColumnVector(), state) << ", Pxy = [ " << (H*covariance*H.transpose()) << " ], innovation = " << innovation);

      status_flags_ |= STATE_Z_POSITION;
    }

    // update Yaw
    if (information(6,6) > 0.0) {
      // fetch observation matrix H
      Matrix H = yaw_model_.dfGet(0);
      ColumnVector x(yaw_model_.ExpectedValueGet());
      ColumnVector y(1); y(1) = update_euler(0);
      SymmetricMatrix Iy(information.sub(6,6,6,6));

      // invert Iy if information is a covariance matrix
      if (!interpret_covariance_as_information_matrix_) Iy = Iy.inverse();

      // fixed_yaw_stddev_ = 5.0 * M_PI/180.0;
      if (fixed_yaw_stddev_ != 0.0) {
        Iy = 0.0;
        Iy(1,1) = 1.0 / (fixed_yaw_stddev_*fixed_yaw_stddev_);
      }

      ROS_DEBUG_STREAM_NAMED("poseupdate", "Yaw Update: ");
      ROS_DEBUG_STREAM_NAMED("poseupdate", "      x = " << x(1) * 180.0/M_PI << "°, H = [ " << H << " ], Px = [" <<  (H*covariance*H.transpose()) << "], Ix = [ " << (H*covariance*H.transpose()).inverse() << "]");
      ROS_DEBUG_STREAM_NAMED("poseupdate", "      y = " << y(1) * 180.0/M_PI << "°, Iy = [ " << Iy << " ]");

      ColumnVector error(y - x);
      error(1) = error(1) - 2.0*M_PI * round(error(1) / (2.0*M_PI));

      double innovation = updateInternal(covariance, state, Iy, error, H, covariance, state, "yaw", max_yaw_error_);
      ROS_DEBUG_STREAM_NAMED("poseupdate", " ==> xy = " << yaw_model_.PredictionGet(ColumnVector(), state) * 180.0/M_PI << "°, Pxy = [ " << (H*covariance*H.transpose()) << " ], innovation = " << innovation);

      status_flags_ |= STATE_YAW;
    }

    break;
  }

  while (update.twist) {
    // convert incoming update information to Eigen
    Eigen::Vector3d update_linear(update.twist->twist.twist.linear.x, update.twist->twist.twist.linear.y, update.twist->twist.twist.linear.z);
    Eigen::Vector3d update_angular(update.twist->twist.twist.angular.x, update.twist->twist.twist.angular.y, update.twist->twist.twist.angular.z);

    // information is the information matrix if interpret_covariance_as_information_matrix_ is true and a covariance matrix otherwise
    // zero elements are counted as zero information in any case
    SymmetricMatrix information(6);
    covarianceMsgToBfl(update.twist->twist.covariance, information);

    // forward state vector to the individual measurement models
    twist_model_.ConditionalArgumentSet(0,state);

    ROS_DEBUG_STREAM_NAMED("poseupdate", "TwistUpdate:  state = [ " << state.transpose() << " ], P = [ " << covariance << " ]" << std::endl
              << "     update: linear = [ " << update_linear.transpose() << " ], angular = [ " << update_angular.transpose() << " ], information = [ " << information << " ]");
    ROS_DEBUG_STREAM_NAMED("poseupdate", "                dt = " << (estimator.getTimestamp() - update.twist->header.stamp).toSec() << " s");

    // degrade information if the time difference is too large
    if (!update.twist->header.stamp.isZero()) {
      double dt = (estimator.getTimestamp() - update.twist->header.stamp).toSec();
      if (dt < 0.0) {
        ROS_DEBUG_STREAM_NAMED("poseupdate", "Ignoring twist update as it has a negative time difference: dt = " << dt << "s");
        break;

      } else if (max_time_difference_ > 0.0 && dt >= max_time_difference_) {
        ROS_DEBUG_STREAM_NAMED("poseupdate", "Ignoring twist update as the time difference is too large: dt = " << dt << "s");
        break;

      } else if (max_time_difference_ > 0.0){
        if (interpret_covariance_as_information_matrix_)
          information = information * (1.0 - dt/max_time_difference_);
        else
          information = information / (1.0 - dt/max_time_difference_);
      }
    }

    // fetch observation matrix H
    Matrix H = twist_model_.dfGet(0);
    ColumnVector x(twist_model_.ExpectedValueGet());
    TwistModel::NoiseCovariance Iy(information);
    TwistModel::MeasurementVector y;
    y(1) = update_linear.x();
    y(2) = update_linear.y();
    y(3) = update_linear.z();
    y(4) = update_angular.x();
    y(5) = update_angular.y();
    y(6) = update_angular.z();

    // invert Iy if information is a covariance matrix
    if (!interpret_covariance_as_information_matrix_) {
      ROS_DEBUG_NAMED("poseupdate", "Twist updates with covariance matrices are currently not supported");
      break;
    }

    // update VelocityXY
    if (information(1,1) > 0.0 || information(2,2) > 0.0) {
      status_flags_ |= STATE_XY_VELOCITY;

      // fixed_velocity_xy_stddev_ = 1.0;
      if (fixed_velocity_xy_stddev_ != 0.0) {
        for(int i = 1; i <= 6; ++i) Iy(1,i) = Iy(2,i) = Iy(i,1) = Iy(i,2) = 0.0;
        Iy(1,1) = Iy(2,2) = 1.0 / (fixed_velocity_xy_stddev_*fixed_velocity_xy_stddev_);
      }
    }

    // update VelocityZ
    if (information(3,3) > 0.0) {
      status_flags_ |= STATE_Z_VELOCITY;

      // fixed_velocity_z_stddev_ = 1.0;
      if (fixed_velocity_z_stddev_ != 0.0) {
          for(int i = 1; i <= 6; ++i) Iy(3,i) = Iy(i,3) = 0.0;
        Iy(3,3) = 1.0 / (fixed_velocity_z_stddev_*fixed_velocity_z_stddev_);
      }
    }

    // update RateXY
    if (information(4,4) > 0.0 || information(5,5) > 0.0) {
      // fixed_angular_rate_xy_stddev_ = 1.0;
      if (fixed_angular_rate_xy_stddev_ != 0.0) {
        for(int i = 1; i <= 6; ++i) Iy(4,i) = Iy(4,i) = Iy(i,5) = Iy(i,5) = 0.0;
        Iy(4,4) = Iy(5,5) = 1.0 / (fixed_angular_rate_xy_stddev_*fixed_angular_rate_xy_stddev_);
      }
    }

    // update RateZ
    if (information(6,6) > 0.0) {
      // fixed_angular_rate_z_stddev_ = 1.0;
      if (fixed_angular_rate_z_stddev_ != 0.0) {
        for(int i = 1; i <= 6; ++i) Iy(6,i) = Iy(i,6) = 0.0;
        Iy(6,6) = 1.0 / (fixed_angular_rate_z_stddev_*fixed_angular_rate_z_stddev_);
      }
    }

    ROS_DEBUG_STREAM_NAMED("poseupdate", "Twist Update: ");
    ROS_DEBUG_STREAM_NAMED("poseupdate", "      x = [" << x.transpose() << "], H = [ " << H << " ], Px = [" <<  (H*covariance*H.transpose()) << "], Ix = [ " << (H*covariance*H.transpose()).inverse() << "]");
    ROS_DEBUG_STREAM_NAMED("poseupdate", "      y = [" << y.transpose() << "], Iy = [ " << Iy << " ]");
    double innovation = updateInternal(covariance, state, Iy, y - x, H, covariance, state, "twist", 0.0);
    ROS_DEBUG_STREAM_NAMED("poseupdate", " ==> xy = [" << twist_model_.PredictionGet(ColumnVector(), state).transpose() << "], Pxy = [ " << (H*covariance*H.transpose()) << " ], innovation = " << innovation);

    break;
  }

  estimator.setState(state);
  estimator.setCovariance(covariance);
  estimator.updated();
  updated();

  return true;
}

double PoseUpdate::calculateOmega(const SymmetricMatrix &Ix, const SymmetricMatrix &Iy) const {
  double tr_x = static_cast<EigenMatrix>(Ix).trace();
  double tr_y = static_cast<EigenMatrix>(Iy).trace();
  return tr_y / (tr_x + tr_y);
}

double PoseUpdate::updateInternal(const SymmetricMatrix &Px, const ColumnVector &x, const SymmetricMatrix &Iy, const ColumnVector &error, const Matrix &H, SymmetricMatrix &Pxy, ColumnVector &xy, const std::string& text, const double max_error) {
  Matrix HT(H.transpose());
  SymmetricMatrix H_Px_HT(H*Px*HT);

  if (H_Px_HT.determinant() <= 0) {
    ROS_WARN_STREAM("Ignoring poseupdate for " << text << " as the a-priori state covariance is zero!");
    return 0.0;
  }
  SymmetricMatrix Ix(H_Px_HT.inverse());

  ROS_DEBUG_STREAM_NAMED("poseupdate", "H = [" << H << "]");
  ROS_DEBUG_STREAM_NAMED("poseupdate", "Ix = [" << Ix << "]");

  double alpha = fixed_alpha_, beta = fixed_beta_;
  if (alpha == 0.0 && beta == 0.0) {
    beta = calculateOmega(Ix, Iy);
    alpha = 1.0 - beta;

//    if (beta > 0.8) {
//      ROS_DEBUG_STREAM("Reducing update variance for " << text << " due to high information difference between Ix = [" << Ix << "] and Iy = [" << Iy << "]");
//      beta = 0.8;
//      alpha = 1.0 - beta;
//    }
  }
  ROS_DEBUG_STREAM_NAMED("poseupdate", "alpha = " << alpha << ", beta = " << beta);

  if (max_error > 0.0) {
    double error2 = error.transpose() * Ix * (Ix + Iy).inverse() * Iy * error;
    if (error2 > max_error * max_error) {
      if (!jump_on_max_error_) {
        ROS_WARN_STREAM_NAMED("poseupdate", "Ignoring poseupdate for " << text << " as the error [ " << error.transpose() << " ], |error| = " << sqrt(error2) << " sigma exceeds max_error!");
        return 0.0;
      } else {
        ROS_WARN_STREAM_NAMED("poseupdate", "Update for " << text << " with error [ " << error.transpose() << " ], |error| = " << sqrt(error2) << " sigma exceeds max_error!");
        alpha = 0.0;
        beta = 1.0;
      }
    }
  }

//  SymmetricMatrix Ii(Ix * (alpha - 1) + Iy * beta);
//  double innovation = Ii.determinant();
//  ROS_DEBUG_STREAM_NAMED("poseupdate", "Ii = [" << Ii << "], innovation = " << innovation);

  // S_1 is equivalent to S^(-1) = (H*P*H^T + R)^(-1) in the standard Kalman gain
  SymmetricMatrix S_1(Ix - Ix * (Ix * alpha + Iy * beta).inverse() * Ix);
  double innovation = S_1.determinant();

  Pxy = Px - Px  * HT * S_1 * H * Px; // may invalidate Px if &Pxy == &Px
   xy =  x + Pxy * HT * Iy * beta * error;

  ROS_DEBUG_STREAM_NAMED("poseupdate", "K = [" << (Pxy * HT * Iy * beta) << "]");
  ROS_DEBUG_STREAM_NAMED("poseupdate", "dx = [" << ( Pxy * HT * Iy * beta * error).transpose() << "]");

  return innovation;
}

ColumnVector PositionXYModel::ExpectedValueGet() const {
  y_(1) = x_(POSITION_X);
  y_(2) = x_(POSITION_Y);
  return y_;
}

Matrix PositionXYModel::dfGet(unsigned int i) const {
  if (i != 0) return Matrix();
  C_(1,POSITION_X)   = 1.0;
  C_(2,POSITION_Y)   = 1.0;
  return C_;
}

ColumnVector PositionZModel::ExpectedValueGet() const {
  y_(1) = x_(POSITION_Z);
  return y_;
}

Matrix PositionZModel::dfGet(unsigned int i) const {
  if (i != 0) return Matrix();
  C_(1,POSITION_Z)   = 1.0;
  return C_;
}

ColumnVector YawModel::ExpectedValueGet() const {
  const double qw = x_(QUATERNION_W);
  const double qx = x_(QUATERNION_X);
  const double qy = x_(QUATERNION_Y);
  const double qz = x_(QUATERNION_Z);

  y_(1) = atan2(2*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz);

  return y_;
}

Matrix YawModel::dfGet(unsigned int i) const {
  if (i != 0) return Matrix();

  const double qw = x_(QUATERNION_W);
  const double qx = x_(QUATERNION_X);
  const double qy = x_(QUATERNION_Y);
  const double qz = x_(QUATERNION_Z);
  const double t1 = qw*qw + qx*qx - qy*qy - qz*qz;
  const double t2 = 2*(qx*qy + qw*qz);
  const double t3 = 1.0 / (t1*t1 + t2*t2);

  C_(1,QUATERNION_W) = 2.0 * t3 * (qz * t1 - qw * t2);
  C_(1,QUATERNION_X) = 2.0 * t3 * (qy * t1 - qx * t2);
  C_(1,QUATERNION_Y) = 2.0 * t3 * (qx * t1 + qy * t2);
  C_(1,QUATERNION_Z) = 2.0 * t3 * (qw * t1 + qz * t2);

  return C_;
}

ColumnVector TwistModel::ExpectedValueGet() const {
  y_(1) = x_(VELOCITY_X);
  y_(2) = x_(VELOCITY_Y);
  y_(3) = x_(VELOCITY_Z);
#ifdef USE_RATE_SYSTEM_MODEL
  y_(4) = x_(RATE_X);
  y_(5) = x_(RATE_Y);
  y_(6) = x_(RATE_Z);
#else // USE_RATE_SYSTEM_MODEL
  y_(4) = 0.0;
  y_(5) = 0.0;
  y_(6) = 0.0;
#endif // USE_RATE_SYSTEM_MODEL
  return y_;
}

Matrix TwistModel::dfGet(unsigned int i) const {
  if (i != 0) return Matrix();

  C_(1,VELOCITY_X) = 1.0;
  C_(2,VELOCITY_Y) = 1.0;
  C_(3,VELOCITY_Z) = 1.0;
#ifdef USE_RATE_SYSTEM_MODEL
  C_(4,RATE_X) = 1.0;
  C_(5,RATE_Y) = 1.0;
  C_(6,RATE_Z) = 1.0;
#endif // USE_RATE_SYSTEM_MODEL
  return C_;
}

} // namespace hector_pose_estimation
