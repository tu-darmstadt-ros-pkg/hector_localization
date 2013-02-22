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
#define USE_RATE_SYSTEM_MODEL

namespace hector_pose_estimation {

class SystemModel;
class SubState;
template <int Dimension> class SubState_;
typedef boost::shared_ptr<SubState> SubStatePtr;
typedef boost::weak_ptr<SubState> SubStateWPtr;

class State {
public:
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
    VELOCITY_X, // body frame
    VELOCITY_Y, // body frame
    VELOCITY_Z, // body frame
    StateDimension
  };
  enum { Dimension = StateDimension };
//  typedef ColumnVector_<Dimension> Vector;
//  typedef SymmetricMatrix_<Dimension> Covariance;
  typedef ColumnVector Vector;
  typedef SymmetricMatrix Covariance;

  typedef VectorBlock<Vector,4> OrientationType;
  typedef VectorBlock<Vector,3> RateType;
  typedef VectorBlock<Vector,3> PositionType;
  typedef VectorBlock<Vector,3> VelocityType;
  typedef VectorBlock<Vector,3> AccelerationType;

  typedef std::vector<SubStatePtr> SubStates;

public:
  State();
  State(const Vector &vector, const Covariance& covariance);
  virtual ~State();

  virtual void reset();
  virtual void updated();
  virtual double normalize();

  virtual const Vector& getVector() const { return state_; }
  virtual const Covariance& getCovariance() const { return covariance_; }
  template <int Size> VectorBlock<const Vector, Size> getSegment(IndexType start) const { return state_.segment<Size>(start); }

  virtual Vector& x() { return state_; }
  virtual Covariance& P() { return covariance_; }

  virtual SystemStatus getSystemStatus() const { return system_status_; }
  virtual SystemStatus getMeasurementStatus() const { return measurement_status_; }

  virtual bool inSystemStatus(SystemStatus test_status) const;
  virtual bool setSystemStatus(SystemStatus new_status);
  virtual bool setMeasurementStatus(SystemStatus new_status);
  virtual bool updateSystemStatus(SystemStatus set, SystemStatus clear);
  virtual bool updateMeasurementStatus(SystemStatus set, SystemStatus clear);

  typedef boost::function<bool(SystemStatus&)> SystemStatusCallback;
  virtual void addSystemStatusCallback(const SystemStatusCallback& callback);

  virtual OrientationType& orientation() { return orientation_; }
  virtual const OrientationType& getOrientation() const { return orientation_; }
  virtual RateType& rate() { return rate_; }
  virtual const RateType& getRate() const { return rate_; }
  virtual PositionType& position() { return position_; }
  virtual const PositionType& getPosition() const { return position_; }
  virtual VelocityType& velocity() { return velocity_; }
  virtual const VelocityType& getVelocity() const { return velocity_; }
  virtual AccelerationType& acceleration() { return acceleration_; }
  virtual const AccelerationType& getAcceleration() const { return acceleration_; }

  virtual void setRate(const ConstVectorBlock3& rate);
  virtual void setAcceleration(const ConstVectorBlock3& acceleration);

  virtual IndexType getOrientationIndex() const { return QUATERNION_X; }
#ifdef USE_RATE_SYSTEM_MODEL
  virtual IndexType getRateIndex() const { return RATE_X; }
#else
  virtual IndexType getRateIndex() const { return IndexType(-1); }
#endif
  virtual IndexType getPositionIndex() const { return POSITION_X; }
  virtual IndexType getVelocityIndex() const { return VELOCITY_X; }
  virtual IndexType getAccelerationIndex() const { return IndexType(-1); }

  template <int SubDimension> boost::shared_ptr<SubState_<SubDimension> > addSubState(const SystemModel *model);
  template <int SubDimension> boost::shared_ptr<SubState_<SubDimension> > getSubState(const SystemModel *model) const;
  const SubStates& getSubStates() const { return substates_; }

  const ros::Time& getTimestamp() const { return timestamp_; }
  void setTimestamp(const ros::Time& timestamp) { timestamp_ = timestamp; }

private:
  Vector state_;
  Covariance covariance_;

  SystemStatus system_status_;
  SystemStatus measurement_status_;

  OrientationType orientation_;
  boost::shared_ptr<ColumnVector> rate_storage_;
  RateType rate_;
  PositionType position_;
  VelocityType velocity_;
  boost::shared_ptr<ColumnVector> acceleration_storage_;
  AccelerationType acceleration_;

  std::vector<SystemStatusCallback> status_callbacks_;

  SubStates substates_;
  std::map<const SystemModel *, SubStateWPtr> substate_map_;

  ros::Time timestamp_;
};

class SubState
{
public:
  virtual ~SubState() {}
  virtual int getDimension() const = 0;

  virtual void reset() = 0;
  virtual void updated() {}
  virtual void normalize() {}
};

template <int _Dimension>
class SubState_ : public SubState
{
public:
  static const unsigned int Dimension = _Dimension;
  typedef SubState_<Dimension> type;
  typedef ColumnVector_<Dimension> Vector;
  typedef SymmetricMatrix_<Dimension> Covariance;
  typedef Matrix_<State::Dimension, Dimension> CrossVariance;

  typedef boost::shared_ptr<SubState_<Dimension> > Ptr;

public:
  virtual ~SubState_() {}
  virtual int getDimension() const { return Dimension; }

  virtual void reset()
  {
    state_.setZero();
    covariance_.setZero();
    cross_variance_.setZero();
  }

  virtual const Vector& getVector() const { return state_; }
  virtual const Covariance& getCovariance() const { return covariance_; }
  template <int Size> VectorBlock<const typename Vector::Base, Size> getSegment(IndexType start) const { return state_.segment<Size>(start); }

  virtual Vector& x() { return state_; }
  virtual Covariance& P() { return covariance_; }
  virtual CrossVariance& S() { return cross_variance_; }

private:
  Vector state_;
  Covariance covariance_;
  CrossVariance cross_variance_;
};

template <int SubDimension>
boost::shared_ptr<SubState_<SubDimension> > State::getSubState(const SystemModel *model) const {
  return boost::shared_dynamic_cast<SubState_<SubDimension> >(substate_map_.count(model) ? substate_map_.at(model).lock() : SubStatePtr());
}

template <int SubDimension>
boost::shared_ptr<SubState_<SubDimension> > State::addSubState(const SystemModel *model) {
  boost::shared_ptr<SubState_<SubDimension> > substate(new SubState_<SubDimension>);
  substates_.push_back(boost::shared_static_cast<SubState>(substate));
  substate_map_[model] = SubStateWPtr(boost::shared_static_cast<SubState>(substate));
  return substate;
}

} // namespace hector_pose_estimation

#endif // HECTOR_POSE_ESTIMATION_STATE_H
