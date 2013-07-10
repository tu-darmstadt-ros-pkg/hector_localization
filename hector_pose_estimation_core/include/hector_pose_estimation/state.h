//=================================================================================================
// Copyright (c) 2013, Johannes Meyer, TU Darmstadt
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

#ifndef HECTOR_POSE_ESTIMATION_STATE_H
#define HECTOR_POSE_ESTIMATION_STATE_H

#include <hector_pose_estimation/types.h>

#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

#include <ros/time.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>

// Use system model with angular rates.
// #define USE_RATE_SYSTEM_MODEL

// #define VELOCITY_IN_BODY_FRAME
#define VELOCITY_IN_WORLD_FRAME

namespace hector_pose_estimation {

class State {
public:
  enum VectorIndex {
    X = 0,
    Y = 1,
    Z = 2,
    W = 3
  };

  enum StateIndex {
    QUATERNION_X = 0,
    QUATERNION_Y,
    QUATERNION_Z,
    QUATERNION_W,
#ifdef USE_RATE_SYSTEM_MODEL
    RATE_X, // body frame
    RATE_Y, // body frame
    RATE_Z, // body frame
#endif // USE_RATE_SYSTEM_MODEL
    POSITION_X,
    POSITION_Y,
    POSITION_Z,
    VELOCITY_X,
    VELOCITY_Y,
    VELOCITY_Z,
    Dimension,

#ifndef USE_RATE_SYSTEM_MODEL
    RATE_X = -1,
    RATE_Y = -1,
    RATE_Z = -1,
#endif // USE_RATE_SYSTEM_MODEL
    ACCELERATION_X = -1,
    ACCELERATION_Y = -1,
    ACCELERATION_Z = -1
  };

  typedef ColumnVector Vector;
  typedef SymmetricMatrix Covariance;
  typedef VectorBlock<Vector,Dimension> VectorSegment;
  typedef Block<Covariance::Base,Dimension,Dimension> CovarianceBlock;
  typedef VectorBlock<const Vector,Dimension> ConstVectorSegment;
  typedef Block<const Covariance::Base,Dimension,Dimension> ConstCovarianceBlock;

  typedef VectorBlock<Vector,4> OrientationType;
  typedef VectorBlock<const Vector,4> ConstOrientationType;
  typedef VectorBlock<Vector,3> RateType;
  typedef VectorBlock<const Vector,3> ConstRateType;
  typedef VectorBlock<Vector,3> PositionType;
  typedef VectorBlock<const Vector,3> ConstPositionType;
  typedef VectorBlock<Vector,3> VelocityType;
  typedef VectorBlock<const Vector,3> ConstVelocityType;
  typedef VectorBlock<Vector,3> AccelerationType;
  typedef VectorBlock<const Vector,3> ConstAccelerationType;

  typedef std::vector<SubStatePtr> SubStates;

  typedef Matrix_<3,3> RotationMatrix;

public:
  State();
  State(const Vector &vector, const Covariance& covariance);
  virtual ~State();

  static IndexType getDimension0() { return Dimension; }
  virtual IndexType getDimension() const { return vector_.rows(); }

  virtual void reset();
  virtual void updated();
  virtual double normalize();

  virtual bool valid() const;

  virtual BaseState& base() { return *base_; }
  virtual const BaseState& base() const { return *base_; }

  virtual const Vector& getVector() const { return vector_; }
  virtual const Covariance& getCovariance() const { return covariance_; }
  template <int Size> VectorBlock<const Vector, Size> getSegment(IndexType start) const { return vector_.segment<Size>(start); }

  virtual Vector& x() { return vector_; }
  virtual Covariance& P() { return covariance_; }
  virtual VectorSegment x0() { return vector_.segment<Dimension>(0); }
  virtual CovarianceBlock P0() { return covariance_.block<Dimension,Dimension>(0, 0); }

  virtual SystemStatus getSystemStatus() const { return system_status_; }
  virtual SystemStatus getMeasurementStatus() const { return measurement_status_; }

  virtual bool inSystemStatus(SystemStatus test_status) const;
  virtual bool setSystemStatus(SystemStatus new_status);
  virtual bool setMeasurementStatus(SystemStatus new_status);
  virtual bool updateSystemStatus(SystemStatus set, SystemStatus clear);
  virtual bool updateMeasurementStatus(SystemStatus set, SystemStatus clear);

  typedef boost::function<bool(SystemStatus&)> SystemStatusCallback;
  virtual void addSystemStatusCallback(const SystemStatusCallback& callback);

  virtual OrientationType orientation()                 { return (getOrientationIndex() >= 0) ? vector_.segment<4>(getOrientationIndex()) : fake_orientation_.segment<4>(0); }
  virtual ConstOrientationType getOrientation() const   { return (getOrientationIndex() >= 0) ? vector_.segment<4>(getOrientationIndex()) : fake_orientation_.segment<4>(0); }
  virtual RateType rate()                               { return (getRateIndex() >= 0) ? vector_.segment<3>(getRateIndex()) : fake_rate_.segment<3>(0); }
  virtual ConstRateType getRate() const                 { return (getRateIndex() >= 0) ? vector_.segment<3>(getRateIndex()) : fake_rate_.segment<3>(0); }
  virtual PositionType position()                       { return (getPositionIndex() >= 0) ? vector_.segment<3>(getPositionIndex()) : fake_position_.segment<3>(0); }
  virtual ConstPositionType getPosition() const         { return (getPositionIndex() >= 0) ? vector_.segment<3>(getPositionIndex()) : fake_position_.segment<3>(0); }
  virtual VelocityType velocity()                       { return (getVelocityIndex() >= 0) ? vector_.segment<3>(getVelocityIndex()) : fake_velocity_.segment<3>(0); }
  virtual ConstVelocityType getVelocity() const         { return (getVelocityIndex() >= 0) ? vector_.segment<3>(getVelocityIndex()) : fake_velocity_.segment<3>(0); }
  virtual AccelerationType acceleration()               { return (getAccelerationIndex() >= 0) ? vector_.segment<3>(getAccelerationIndex()) : fake_acceleration_.segment<3>(0); }
  virtual ConstAccelerationType getAcceleration() const { return (getAccelerationIndex() >= 0) ? vector_.segment<3>(getAccelerationIndex()) : fake_acceleration_.segment<3>(0); }

  RotationMatrix getRotationMatrix() const;
  void getRotationMatrix(RotationMatrix &R) const;

  double getYaw() const;

  template <typename Derived> void setOrientation(const Eigen::MatrixBase<Derived>& orientation);
  template <typename Derived> void setRate(const Eigen::MatrixBase<Derived>& rate);
  template <typename Derived> void setPosition(const Eigen::MatrixBase<Derived>& position);
  template <typename Derived> void setVelocity(const Eigen::MatrixBase<Derived>& velocity);
  template <typename Derived> void setAcceleration(const Eigen::MatrixBase<Derived>& acceleration);

  virtual IndexType getOrientationIndex() const { return QUATERNION_X; }
  virtual IndexType getRateIndex() const { return RATE_X; }
  virtual IndexType getPositionIndex() const { return POSITION_X; }
  virtual IndexType getVelocityIndex() const { return VELOCITY_X; }
  virtual IndexType getAccelerationIndex() const { return ACCELERATION_X; }

  const SubStates& getSubStates() const { return substates_; }
  template <int SubDimension> boost::shared_ptr<SubState_<SubDimension> > getSubState(const Model *model) const;
  template <int SubDimension> boost::shared_ptr<SubState_<SubDimension> > getSubState(const std::string& name) const;
  template <int SubDimension> boost::shared_ptr<SubState_<SubDimension> > addSubState(const Model *model, const std::string& name = std::string());

  const ros::Time& getTimestamp() const { return timestamp_; }
  void setTimestamp(const ros::Time& timestamp) { timestamp_ = timestamp; }

private:
  Vector vector_;
  Covariance covariance_;

  SystemStatus system_status_;
  SystemStatus measurement_status_;

  Vector fake_orientation_;
  Vector fake_rate_;
  Vector fake_position_;
  Vector fake_velocity_;
  Vector fake_acceleration_;

  std::vector<SystemStatusCallback> status_callbacks_;

  SubStates substates_;
  std::map<const Model *, SubStateWPtr> substates_by_model_;
  std::map<std::string, SubStateWPtr> substates_by_name_;

  boost::shared_ptr<BaseState> base_;

  ros::Time timestamp_;
};

template <typename Derived>
void State::setOrientation(const Eigen::MatrixBase<Derived>& orientation) {
  eigen_assert(orientation.rows() == 4 && orientation.cols() == 1);
  fake_orientation_ = orientation;
}

template <typename Derived>
void State::setRate(const Eigen::MatrixBase<Derived>& rate) {
  eigen_assert(rate.rows() == 3 && rate.cols() == 1);
  fake_rate_ = rate;
}

template <typename Derived>
void State::setPosition(const Eigen::MatrixBase<Derived>& position) {
  eigen_assert(position.rows() == 3 && position.cols() == 1);
  fake_position_ = position;
}

template <typename Derived>
void State::setVelocity(const Eigen::MatrixBase<Derived>& velocity) {
  eigen_assert(velocity.rows() == 3 && velocity.cols() == 1);
  fake_velocity_ = velocity;
}

template <typename Derived>
void State::setAcceleration(const Eigen::MatrixBase<Derived>& acceleration) {
  eigen_assert(acceleration.rows() == 3 && acceleration.cols() == 1);
  fake_acceleration_ = acceleration;
}

} // namespace hector_pose_estimation

#endif // HECTOR_POSE_ESTIMATION_STATE_H
