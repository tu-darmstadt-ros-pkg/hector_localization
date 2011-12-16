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
#include <hector_pose_estimation/bfl_conversions.h>
#include <Eigen/Geometry>

namespace hector_pose_estimation {

PoseUpdate::PoseUpdate()
  : Measurement("poseupdate")
{
  alpha_ = 0.0;
  beta_  = 0.0;
  fixed_position_xy_stddev_ = 0.0;
  fixed_position_z_stddev_ = 0.0;
  fixed_yaw_stddev_ = 0.0;

  parameters().add("alpha", alpha_);
  parameters().add("beta", beta_);
  parameters().add("fixed_position_xy_stddev", fixed_position_xy_stddev_);
  parameters().add("fixed_position_z_stddev", fixed_position_z_stddev_);
  parameters().add("fixed_yaw_stddev", fixed_yaw_stddev_);
}

PoseUpdate::~PoseUpdate()
{
}

void PoseUpdate::update(BFL::KalmanFilter &filter, const SystemStatus &status, const MeasurementUpdate &update_) {
  Update const &update = static_cast<Update const &>(update_);

  // convert incoming update information
  ColumnVector update_euler(Eigen::Quaterniond(update.pose->pose.pose.orientation.w, update.pose->pose.pose.orientation.x, update.pose->pose.pose.orientation.y, update.pose->pose.pose.orientation.z).toRotationMatrix().eulerAngles(2,1,0));
  SymmetricMatrix sigma(6);
  covarianceMsgToBfl(update.pose->pose.covariance, sigma);

  // fetch current state
  ColumnVector state(filter.PostGet()->ExpectedValueGet());
  SymmetricMatrix covariance(filter.PostGet()->CovarianceGet());

  // forward state vector to the individual measurement models
  position_xy_model_.ConditionalArgumentSet(0,state);
  position_z_model_.ConditionalArgumentSet(0,state);
  yaw_model_.ConditionalArgumentSet(0,state);

//   std::cout << "PoseUpdate: state = [ " << state.transpose() << " ]" << std::endl
//             << "            euler = [ " << update_euler.transpose() << " ], sigma = [ " << sigma << " ]" << std::endl;

  // update PositionXY
  if (sigma(1,1) > 0.0 && sigma(2,2) > 0.0) {
//    ColumnVector x(state.sub(POSITION_X,POSITION_Y));
//    SymmetricMatrix Ix(covariance.sub(POSITION_X,POSITION_Y,POSITION_X,POSITION_Y).inverse());
//    ColumnVector y(2);
//    SymmetricMatrix Iy(sigma.sub(1,2,1,2)); // assume that sigma is a information matrix directly!
//    y(1) = update.pose->pose.pose.position.x;
//    y(2) = update.pose->pose.pose.position.y;

//    fixed_position_xy_stddev_ = 1.0;

//    if (fixed_position_xy_stddev_ != 0.0) {
//      Iy = 0.0;
//      Iy(1,1) = Iy(2,2) = 1.0 / (fixed_position_xy_stddev_*fixed_position_xy_stddev_);
//    }

////     std::cout << "XY: ";
////    std::cout << "state:  xy = (" << x(1) << "," << x(2) << "), P_xy = [" << (covariance.sub(POSITION_X,POSITION_Y,POSITION_X,POSITION_Y)) << ", Ix_xy = [" << Ix << "]" << std::endl;
////    std::cout << "update: xy = (" << y(1) << "," << y(2) << "), R_xy = [" << (Iy.inverse()) << "], Iy_xy = [" << Iy << "]" << std::endl;

//    ColumnVector ixy(x.rows());
//    SymmetricMatrix Ixy(Ix.rows());
//    updateInternal(Ix, x, Iy, y, Ixy, ixy);
//    updateState(state, covariance, POSITION_X, POSITION_Y, Ixy, ixy);

//    status_flags_ |= STATE_XY_POSITION;

    // fetch observation matrix H
    Matrix H = position_xy_model_.dfGet(0);
    ColumnVector x(position_xy_model_.ExpectedValueGet());
    ColumnVector y(2);
    SymmetricMatrix Iy(sigma.sub(1,2,1,2)); // assume that sigma is a information matrix directly!
    y(1) = update.pose->pose.pose.position.x;
    y(2) = update.pose->pose.pose.position.y;

    // fixed_position_xy_stddev_ = 1.0;
    if (fixed_position_xy_stddev_ != 0.0) {
      Iy = 0.0;
      Iy(1,1) = Iy(2,2) = 1.0 / (fixed_position_xy_stddev_*fixed_position_xy_stddev_);
    }

//    std::cout << "Position Update: " << std::endl;
//    std::cout << "      x = [" << x.transpose() << "], Px = [" <<  (H*covariance*H.transpose()) << "], Ix = [ " << (H*covariance*H.transpose()).inverse() << "]" << std::endl;
//    std::cout << "      y = [" << y.transpose() << "], Iy = [ " << Iy << " ]" << std::endl;

    updateInternal(covariance, state, Iy, y - x, H, covariance, state);

//    std::cout << " ==> xy = [" << position_xy_model_.PredictionGet(ColumnVector(), state).transpose() << "], Pxy = [ " << (H*covariance*H.transpose()) << " ], innovation = " << innovation << std::endl;

    status_flags_ |= STATE_XY_POSITION;
  }

  // update PositionZ
  if (sigma(3,3) > 0.0) {
//    ColumnVector x(state.sub(POSITION_Z,POSITION_Z));
//    SymmetricMatrix Ix(covariance.sub(POSITION_Z,POSITION_Z,POSITION_Z,POSITION_Z).inverse());
//    ColumnVector y(1);
//    SymmetricMatrix Iy(sigma.sub(3,3,3,3)); // assume that sigma is a information matrix directly!
//    y(1) = update.pose->pose.pose.position.z;

//    if (fixed_position_z_stddev_ != 0.0) {
//      Iy = 0.0;
//      Iy(1,1) = 1.0 / (fixed_position_z_stddev_*fixed_position_z_stddev_);
//    }

////     std::cout << "Z: ";
//    ColumnVector ixy(x.rows());
//    SymmetricMatrix Ixy(Ix.rows());
//    updateInternal(Ix, x, Iy, y, Ixy, ixy);
//    updateState(state, covariance, POSITION_Z, POSITION_Z, Ixy, ixy);

//    status_flags_ |= STATE_Z_POSITION;

    // fetch observation matrix H
    Matrix H = position_z_model_.dfGet(0);
    ColumnVector x(position_z_model_.ExpectedValueGet());
    ColumnVector y(1); y(1) =  update.pose->pose.pose.position.z;
    SymmetricMatrix Iy(sigma.sub(3,3,3,3)); // assume that sigma is a information matrix directly!

    // fixed_position_z_stddev_ = 1.0;
    if (fixed_position_z_stddev_ != 0.0) {
      Iy = 0.0;
      Iy(1,1) = 1.0 / (fixed_position_z_stddev_*fixed_position_z_stddev_);
    }

//    std::cout << "Height Update: " << std::endl;
//    std::cout << "      x = " << x(1) << ", Px = [" <<  (H*covariance*H.transpose()) << "], Ix = [ " << (H*covariance*H.transpose()).inverse() << "]" << std::endl;
//    std::cout << "      y = " << y(1) << ", Iy = [ " << Iy << " ]" << std::endl;

    updateInternal(covariance, state, Iy, y - x, H, covariance, state);

//    std::cout << " ==> xy = " << position_z_model_.PredictionGet(ColumnVector(), state) << ", Pxy = [ " << (H*covariance*H.transpose()) << " ], innovation = " << innovation << std::endl;

    status_flags_ |= STATE_Z_POSITION;
  }

  // update Yaw
  if (sigma(4,4) > 0.0) {
//    // directly set the state without changing the covariance!
//    ColumnVector state_euler(Eigen::Quaterniond(state(QUATERNION_W), state(QUATERNION_X), state(QUATERNION_Y), state(QUATERNION_Z)).toRotationMatrix().eulerAngles(2,1,0));
//    std::cout << "state:  " << state_euler << std::endl;
//    std::cout << "update: " << update_euler << std::endl;
//    Eigen::Quaterniond new_quaternion(Eigen::AngleAxisd(update_euler(1), Vector3d::UnitZ())
//                               * Eigen::AngleAxisd(state_euler(2), Vector3d::UnitY())
//                               * Eigen::AngleAxisd(state_euler(3), Vector3d::UnitX()));
//    state(QUATERNION_W) = new_quaternion.w();
//    state(QUATERNION_X) = new_quaternion.x();
//    state(QUATERNION_Y) = new_quaternion.y();
//    state(QUATERNION_Z) = new_quaternion.z();

//    ColumnVector x(state.sub(QUATERNION_W,QUATERNION_Z));
//    SymmetricMatrix Ix(covariance.sub(QUATERNION_W,QUATERNION_Z,QUATERNION_W,QUATERNION_Z).inverse());
//    Matrix dquat_dyaw(4,1);
//    dquat_dyaw(1,1) = -0.5 * x(4);
//    dquat_dyaw(2,1) = -0.5 * x(3);
//    dquat_dyaw(3,1) =  0.5 * x(2);
//    dquat_dyaw(4,1) =  0.5 * x(1);
//    ColumnVector y(4);
//    quaternionMsgToBfl(update.pose->pose.pose.orientation, y);
//    SymmetricMatrix Iy(dquat_dyaw * sigma(4,4) * dquat_dyaw.transpose());

    // fetch observation matrix H
    Matrix H = yaw_model_.dfGet(0);
    ColumnVector x(yaw_model_.ExpectedValueGet());
    ColumnVector y(1); y(1) = update_euler(1);
    SymmetricMatrix Iy(sigma.sub(4,4,4,4)); // assume that sigma is a information matrix directly!

    // fixed_yaw_stddev_ = 5.0 * M_PI/180.0;
    if (fixed_yaw_stddev_ != 0.0) {
      Iy = 0.0;
      Iy(1,1) = 1.0 / (fixed_yaw_stddev_*fixed_yaw_stddev_);
    }

//    std::cout << "Yaw Update: " << std::endl;
//    std::cout << "      x = " << x(1) * 180.0/M_PI << "°, Px = [" <<  (H*covariance*H.transpose()) << "], Ix = [ " << (H*covariance*H.transpose()).inverse() << "]" << std::endl;
//    std::cout << "      y = " << y(1) * 180.0/M_PI << "°, Iy = [ " << Iy << " ]" << std::endl;

    updateInternal(covariance, state, Iy, y - x, H, covariance, state);

//    std::cout << " ==> xy = " << yaw_model_.PredictionGet(ColumnVector(), state) * 180.0/M_PI << "°, Pxy = [ " << (H*covariance*H.transpose()) << " ], innovation = " << innovation << std::endl;

    status_flags_ |= STATE_YAW;
  }

  filter.PostGet()->ExpectedValueSet(state);
  filter.PostGet()->CovarianceSet(covariance);
  updated();
}

double PoseUpdate::calculateOmega(const SymmetricMatrix &Ix, const SymmetricMatrix &Iy) const {
  double tr_x = static_cast<EigenMatrix>(Ix).trace();
  double tr_y = static_cast<EigenMatrix>(Iy).trace();
  return tr_y / (tr_x + tr_y);
}

double PoseUpdate::updateInternal(const SymmetricMatrix &Px, const ColumnVector &x, const SymmetricMatrix &Iy, const ColumnVector &i, const Matrix &H, SymmetricMatrix &Pxy, ColumnVector &xy) {
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

//  alpha = 0.5;
//  beta = 0.5;

//  std::cout << "alpha = " << alpha << ", beta = " << beta << std::endl;

  if (beta > 0.8) {
    ROS_DEBUG_STREAM("Reducing update variance due to high information difference between Ix = [" << Ix << "] and Iy = [" << Iy << "]");
    beta = 0.8;
    alpha = 1.0 - beta;
  }

  SymmetricMatrix Ii(Ix * (alpha - 1) + Iy * beta);
  double innovation = Ii.determinant();

//  std::cout << "Ii = [" << Ii << "], innovation = " << innovation << std::endl;

  SymmetricMatrix S_1(Ii.rows()); S_1 = 0.0;
  if (innovation > 0.0) {
    S_1 = (Ii.inverse() + H_Px_HT).inverse();
  } else if (innovation <= 0.0) {
    // ROS_DEBUG_STREAM("Ignoring useless poseupdate for H = [" << H << "]" << " with information [" << Iy << "]");
    // return innovation;
  }

  Pxy = Px - Px  * HT * S_1 * H * Px; // may invalidate Px if &Pxy == &Px
   xy =  x + Pxy * HT * Iy * beta * i;

//  std::cout << "K = [" << (Pxy * HT * Iy * beta) << "]" << std::endl;
//  std::cout << "dx = [" << ( Pxy * HT * Iy * beta * i).transpose() << "]" << std::endl;

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
