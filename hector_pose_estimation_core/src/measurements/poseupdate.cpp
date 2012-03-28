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
  alpha_ = 0.0;
  beta_  = 0.0;
  fixed_position_xy_stddev_ = 0.0;
  fixed_position_z_stddev_ = 0.0;
  fixed_yaw_stddev_ = 0.0;

  max_time_difference_ = 1.0;
  max_position_xy_error_ = 3.0; // 3 sigma
  max_position_z_error_ = 3.0; // 3 sigma
  max_yaw_error_ = 3.0; // 3 sigma

  jump_on_max_error_ = false;

  parameters().add("alpha", alpha_);
  parameters().add("beta", beta_);
  parameters().add("fixed_position_xy_stddev", fixed_position_xy_stddev_);
  parameters().add("fixed_position_z_stddev", fixed_position_z_stddev_);
  parameters().add("fixed_yaw_stddev", fixed_yaw_stddev_);
  parameters().add("max_time_difference", max_time_difference_);
  parameters().add("max_position_xy_error", max_position_xy_error_ );
  parameters().add("max_position_z_error", max_position_z_error_);
  parameters().add("max_yaw_error", max_yaw_error_);
  parameters().add("jump_on_max_error", jump_on_max_error_);
}

PoseUpdate::~PoseUpdate()
{
}

bool PoseUpdate::update(PoseEstimation &estimator, const MeasurementUpdate &update_) {
  Update const &update = static_cast<Update const &>(update_);

  // convert incoming update information to Eigen
  Eigen::Vector3d update_pose(update.pose->pose.pose.position.x, update.pose->pose.pose.position.y, update.pose->pose.pose.position.z);
  Eigen::Quaterniond update_orientation(update.pose->pose.pose.orientation.w, update.pose->pose.pose.orientation.x, update.pose->pose.pose.orientation.y, update.pose->pose.pose.orientation.z);
  Eigen::Vector3d update_euler(update_orientation.toRotationMatrix().eulerAngles(2,1,0));

   // assume that message covariance is a information matrix directly!
  SymmetricMatrix information(6);
  covarianceMsgToBfl(update.pose->pose.covariance, information);

  // fetch current state
  ColumnVector state = estimator.getState();
  SymmetricMatrix covariance = estimator.getCovariance();

  // forward state vector to the individual measurement models
  position_xy_model_.ConditionalArgumentSet(0,state);
  position_z_model_.ConditionalArgumentSet(0,state);
  yaw_model_.ConditionalArgumentSet(0,state);

//  std::cout << "PoseUpdate: state = [ " << state.transpose() << " ]" << std::endl
//            << "     update: pose = [ " << update_pose.transpose() << " ], euler = [ " << update_euler.transpose() << " ], information = [ " << information << " ]" << std::endl;
//  std::cout << "             dt = " << (estimator.getTimestamp() - update.pose->header.stamp).toSec() << " s" << std::endl;

  // predict update pose using the estimated velocity and degrade information
  double dt = (estimator.getTimestamp() - update.pose->header.stamp).toSec();
  if (max_time_difference_ > 0.0) {
    if (dt < 0.0 || dt > max_time_difference_) return false;
    Eigen::Vector3d state_velocity(state.sub(VELOCITY_X, VELOCITY_Z));
    update_pose = update_pose + dt * state_velocity;
    information = information * (1.0 - (dt*dt)/(max_time_difference_*max_time_difference_));
  }

  // update PositionXY
  if (information(1,1) > 0.0 && information(2,2) > 0.0) {
    // fetch observation matrix H
    Matrix H = position_xy_model_.dfGet(0);
    ColumnVector x(position_xy_model_.ExpectedValueGet());
    ColumnVector y(2);
    SymmetricMatrix Iy(information.sub(1,2,1,2));
    y(1) = update_pose.x();
    y(2) = update_pose.y();

    // fixed_position_xy_stddev_ = 1.0;
    if (fixed_position_xy_stddev_ != 0.0) {
      Iy = 0.0;
      Iy(1,1) = Iy(2,2) = 1.0 / (fixed_position_xy_stddev_*fixed_position_xy_stddev_);
    }

//    std::cout << "Position Update: " << std::endl;
//    std::cout << "      x = [" << x.transpose() << "], Px = [" <<  (H*covariance*H.transpose()) << "], Ix = [ " << (H*covariance*H.transpose()).inverse() << "]" << std::endl;
//    std::cout << "      y = [" << y.transpose() << "], Iy = [ " << Iy << " ]" << std::endl;
    /* double innovation = */
    updateInternal(covariance, state, Iy, y - x, H, covariance, state, "position_xy", max_position_xy_error_);
//    std::cout << " ==> xy = [" << position_xy_model_.PredictionGet(ColumnVector(), state).transpose() << "], Pxy = [ " << (H*covariance*H.transpose()) << " ], innovation = " << innovation << std::endl;

    status_flags_ |= STATE_XY_POSITION;
  }

  // update PositionZ
  if (information(3,3) > 0.0) {
    // fetch observation matrix H
    Matrix H = position_z_model_.dfGet(0);
    ColumnVector x(position_z_model_.ExpectedValueGet());
    ColumnVector y(1); y(1) =  update_pose.z();
    SymmetricMatrix Iy(information.sub(3,3,3,3));

    // fixed_position_z_stddev_ = 1.0;
    if (fixed_position_z_stddev_ != 0.0) {
      Iy = 0.0;
      Iy(1,1) = 1.0 / (fixed_position_z_stddev_*fixed_position_z_stddev_);
    }

//    std::cout << "Height Update: " << std::endl;
//    std::cout << "      x = " << x(1) << ", Px = [" <<  (H*covariance*H.transpose()) << "], Ix = [ " << (H*covariance*H.transpose()).inverse() << "]" << std::endl;
//    std::cout << "      y = " << y(1) << ", Iy = [ " << Iy << " ]" << std::endl;
    /* double innovation = */
    updateInternal(covariance, state, Iy, y - x, H, covariance, state, "position_z", max_position_z_error_);
//    std::cout << " ==> xy = " << position_z_model_.PredictionGet(ColumnVector(), state) << ", Pxy = [ " << (H*covariance*H.transpose()) << " ], innovation = " << innovation << std::endl;

    status_flags_ |= STATE_Z_POSITION;
  }

  // update Yaw
  if (information(6,6) > 0.0) {
    // fetch observation matrix H
    Matrix H = yaw_model_.dfGet(0);
    ColumnVector x(yaw_model_.ExpectedValueGet());
    ColumnVector y(1); y(1) = update_euler(0);
    SymmetricMatrix Iy(information.sub(6,6,6,6));

    // fixed_yaw_stddev_ = 5.0 * M_PI/180.0;
    if (fixed_yaw_stddev_ != 0.0) {
      Iy = 0.0;
      Iy(1,1) = 1.0 / (fixed_yaw_stddev_*fixed_yaw_stddev_);
    }

//    std::cout << "Yaw Update: " << std::endl;
//    std::cout << "      x = " << x(1) * 180.0/M_PI << "°, Px = [" <<  (H*covariance*H.transpose()) << "], Ix = [ " << (H*covariance*H.transpose()).inverse() << "]" << std::endl;
//    std::cout << "      y = " << y(1) * 180.0/M_PI << "°, Iy = [ " << Iy << " ]" << std::endl;

    ColumnVector error(y - x);
    error(1) = error(1) - 2.0*M_PI * round(error(1) / (2.0*M_PI));

    /* double innovation = */
    updateInternal(covariance, state, Iy, error, H, covariance, state, "yaw", max_yaw_error_);
//    std::cout << " ==> xy = " << yaw_model_.PredictionGet(ColumnVector(), state) * 180.0/M_PI << "°, Pxy = [ " << (H*covariance*H.transpose()) << " ], innovation = " << innovation << std::endl;

    status_flags_ |= STATE_YAW;
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
  SymmetricMatrix Ix(H_Px_HT.inverse());

//  std::cout << "H = [" << H << "]" << std::endl;
//  std::cout << "Ix = [" << Ix << "]" << std::endl;

  double alpha = alpha_, beta = beta_;
  if (alpha == 0.0 && beta == 0.0) {
    beta = calculateOmega(Ix, Iy);
    alpha = 1.0 - beta;
  }

//  std::cout << "alpha = " << alpha << ", beta = " << beta << std::endl;

  if (beta > 0.8) {
    ROS_DEBUG_STREAM("Reducing update variance for " << text << " due to high information difference between Ix = [" << Ix << "] and Iy = [" << Iy << "]");
    beta = 0.8;
    alpha = 1.0 - beta;
  }

  if (max_error > 0.0) {
    if (error.transpose() * Ix * error > max_error * max_error) {
      if (!jump_on_max_error_) {
        ROS_WARN_STREAM("Ignoring poseupdate for " << text << " as the error [ " << error.transpose() << " ], Ix = [ " << Ix << " ] is too high!");
        return 0.0;
      } else {
        alpha = 0.0;
        beta = 1.0;
      }
    }
  }

  SymmetricMatrix Ii(Ix * (alpha - 1) + Iy * beta);
  double innovation = Ii.determinant();

//  std::cout << "Ii = [" << Ii << "], innovation = " << innovation << std::endl;

  SymmetricMatrix S_1(Ii.rows());
  if (innovation > 0.0) {
    S_1 = (Ii.inverse() + H_Px_HT).inverse();
  } else if (innovation <= 0.0) {
    S_1 = 0.0;
    // ROS_DEBUG_STREAM("Ignoring useless poseupdate for " << text << " with information [" << Iy << "]");
    // return innovation;
  }

  Pxy = Px - Px  * HT * S_1 * H * Px; // may invalidate Px if &Pxy == &Px
   xy =  x + Pxy * HT * Iy * beta * error;

//  std::cout << "K = [" << (Pxy * HT * Iy * beta) << "]" << std::endl;
//  std::cout << "dx = [" << ( Pxy * HT * Iy * beta * error).transpose() << "]" << std::endl;

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

} // namespace hector_pose_estimation
