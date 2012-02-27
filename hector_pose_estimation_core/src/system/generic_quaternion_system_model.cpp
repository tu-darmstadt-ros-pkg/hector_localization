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

#include <hector_pose_estimation/system/generic_quaternion_system_model.h>

namespace hector_pose_estimation {

static const double GRAVITY = -9.8065;

GenericQuaternionSystemModel::GenericQuaternionSystemModel()
{
  gravity_ = GRAVITY;
  gyro_stddev_ = 1.0 * M_PI/180.0;
  acceleration_stddev_ = 1.0;
  velocity_stddev_ = 0.0;
  acceleration_drift_ = 1.0e-6;
  gyro_drift_ = 5.0e-6 * M_PI/180.0;
  parameters().add("gravity", gravity_);
  parameters().add("gyro_stddev", gyro_stddev_);
  parameters().add("acceleration_stddev", acceleration_stddev_);
  parameters().add("velocity_stddev", velocity_stddev_);
  parameters().add("acceleration_drift", acceleration_drift_);
  parameters().add("gyro_drift", gyro_drift_);

  noise_ = 0.0;
  noise_(QUATERNION_W,QUATERNION_W) = noise_(QUATERNION_X,QUATERNION_X) = noise_(QUATERNION_Y,QUATERNION_Y) = noise_(QUATERNION_Z,QUATERNION_Z) = pow(0.5 * gyro_stddev_, 2);
  noise_(POSITION_X,POSITION_X) = noise_(POSITION_Y,POSITION_Y) = noise_(POSITION_Z,POSITION_Z) = pow(velocity_stddev_, 2);
  noise_(VELOCITY_X,VELOCITY_X) = noise_(VELOCITY_Y,VELOCITY_Y) = noise_(VELOCITY_Z,VELOCITY_Z) = pow(acceleration_stddev_, 2);
  noise_(BIAS_ACCEL_X,BIAS_ACCEL_X) = 0.0; // noise_(BIAS_ACCEL_Y,BIAS_ACCEL_Y) = pow(acceleration_drift_, 2);
  noise_(BIAS_ACCEL_Z,BIAS_ACCEL_Z) = pow(acceleration_drift_, 2);
  noise_(BIAS_GYRO_X,BIAS_GYRO_X)	= noise_(BIAS_GYRO_Y,BIAS_GYRO_Y)	= noise_(BIAS_GYRO_Z,BIAS_GYRO_Z)	= pow(gyro_drift_, 2);
}

GenericQuaternionSystemModel::~GenericQuaternionSystemModel()
{
}

SystemStatus GenericQuaternionSystemModel::getStatusFlags() const
{
	SystemStatus flags = SystemStatus(0);
	if (measurement_status_ & STATE_XY_POSITION) flags |= STATE_XY_VELOCITY;
	if (measurement_status_ & STATE_Z_POSITION)  flags |= STATE_Z_VELOCITY;
	if (measurement_status_ & STATE_XY_VELOCITY) flags |= STATE_ROLLPITCH;
	return flags;
}

//--> System equation of this model xpred = x_(k+1) = f(x,u)
ColumnVector GenericQuaternionSystemModel::ExpectedValueGet(double dt) const
{
	x_pred_ = x_;

	//--> Enhance readability
	//----------------------------------------------------------
	double abx		= u_(ACCEL_X) + x_(BIAS_ACCEL_X);
	double aby		= u_(ACCEL_Y) + x_(BIAS_ACCEL_Y);
	double abz		= u_(ACCEL_Z) + x_(BIAS_ACCEL_Z);
	double wbx		= u_(GYRO_X)  + x_(BIAS_GYRO_X);
	double wby		= u_(GYRO_Y)  + x_(BIAS_GYRO_Y);
	double wbz		= u_(GYRO_Z)  + x_(BIAS_GYRO_Z);

	q0            = x_(QUATERNION_W);
	q1            = x_(QUATERNION_X);
	q2            = x_(QUATERNION_Y);
	q3            = x_(QUATERNION_Z);
	double p_x		= x_(POSITION_X);
	double p_y		= x_(POSITION_Y);
	double p_z    = x_(POSITION_Z);
	double v_x		= x_(VELOCITY_X);
	double v_y		= x_(VELOCITY_Y);
	double v_z		= x_(VELOCITY_Z);
	//----------------------------------------------------------

	//--> Attitude
	//----------------------------------------------------------
	x_pred_(QUATERNION_W) = q0 + dt*0.5*(          (-wbx)*q1+(-wby)*q2+(-wbz)*q3);
	x_pred_(QUATERNION_X) = q1 + dt*0.5*(( wbx)*q0          +( wbz)*q2+(-wby)*q3);
	x_pred_(QUATERNION_Y) = q2 + dt*0.5*(( wby)*q0+(-wbz)*q1          +( wbx)*q3);
	x_pred_(QUATERNION_Z) = q3 + dt*0.5*(( wbz)*q0+( wby)*q1+(-wbx)*q2          );
	// normalize(x_pred_);
	//----------------------------------------------------------

	//--> Velocity (without coriolis forces) and Position
	//----------------------------------------------------------
	if (measurement_status_ & (STATE_XY_POSITION | STATE_XY_VELOCITY)) {
		x_pred_(VELOCITY_X)  = v_x + dt*((q0*q0+q1*q1-q2*q2-q3*q3)*abx + (2.0*q1*q2-2.0*q0*q3)    *aby + (2.0*q1*q3+2.0*q0*q2)    *abz);
		x_pred_(VELOCITY_Y)  = v_y + dt*((2.0*q1*q2+2.0*q0*q3)    *abx + (q0*q0-q1*q1+q2*q2-q3*q3)*aby + (2.0*q2*q3-2.0*q0*q1)    *abz);
	}
	if (measurement_status_ & (STATE_Z_POSITION  | STATE_Z_VELOCITY)) {
		x_pred_(VELOCITY_Z)  = v_z + dt*((2.0*q1*q3-2.0*q0*q2)    *abx + (2.0*q2*q3+2.0*q0*q1)    *aby + (q0*q0-q1*q1-q2*q2+q3*q3)*abz + gravity_);
	}

	if (measurement_status_ & STATE_XY_POSITION) {
		x_pred_(POSITION_X)  = p_x + dt*(v_x);
		x_pred_(POSITION_Y)  = p_y + dt*(v_y);
	}
	if (measurement_status_ & STATE_Z_POSITION) {
		x_pred_(POSITION_Z)  = p_z + dt*(v_z);
	}
	//----------------------------------------------------------

	return x_pred_ + AdditiveNoiseMuGet();
}

//--> Covariance
// Warning: CovarianceGet() must be called AFTER ExpectedValueGet(...) or dfGet(...)
// unfortunately MatrixWrapper::SymmetricMatrix CovarianceGet(const MatrixWrapper::ColumnVector& u, const MatrixWrapper::ColumnVector& x) cannot be overridden
SymmetricMatrix GenericQuaternionSystemModel::CovarianceGet(double dt) const
{
	double gyro_variance_4 = 0.25 * pow(gyro_stddev_, 2);
	noise_(QUATERNION_W,QUATERNION_W) = gyro_variance_4 * (q1*q1+q2*q2+q3*q3);
	noise_(QUATERNION_X,QUATERNION_X) = gyro_variance_4 * (q0*q0+q2*q2+q3*q3);
	noise_(QUATERNION_Y,QUATERNION_Y) = gyro_variance_4 * (q0*q0+q1*q1+q3*q3);
	noise_(QUATERNION_Z,QUATERNION_Z) = gyro_variance_4 * (q0*q0+q1*q1+q2*q2);
	return noise_ * (dt*dt);
}

//--> Jacobian matrix A
Matrix GenericQuaternionSystemModel::dfGet(unsigned int i, double dt) const
{
	if (i != 0) return Matrix();

	//--> Enhance readability
	//----------------------------------------------------------
	double abx		= u_(ACCEL_X) + x_(BIAS_ACCEL_X);
	double aby		= u_(ACCEL_Y) + x_(BIAS_ACCEL_Y);
	double abz		= u_(ACCEL_Z) + x_(BIAS_ACCEL_Z);
	double wbx		= u_(GYRO_X)  + x_(BIAS_GYRO_X);
	double wby		= u_(GYRO_Y)  + x_(BIAS_GYRO_Y);
	double wbz		= u_(GYRO_Z)  + x_(BIAS_GYRO_Z);

	q0     = x_(QUATERNION_W);
	q1     = x_(QUATERNION_X);
	q2     = x_(QUATERNION_Y);
	q3     = x_(QUATERNION_Z);
	//----------------------------------------------------------

	//--> Set A-Matrix
	//----------------------------------------------------------
	A_(QUATERNION_W,QUATERNION_X) = dt*(-0.5*wbx);
	A_(QUATERNION_W,QUATERNION_Y) = dt*(-0.5*wby);
	A_(QUATERNION_W,QUATERNION_Z) = dt*(-0.5*wbz);
	A_(QUATERNION_W,BIAS_GYRO_X)  = -0.5*dt*q1;
	A_(QUATERNION_W,BIAS_GYRO_Y)  = -0.5*dt*q2;
	A_(QUATERNION_W,BIAS_GYRO_Z)  = -0.5*dt*q3;

	A_(QUATERNION_X,QUATERNION_W) = dt*( 0.5*wbx);
	A_(QUATERNION_X,QUATERNION_Y) = dt*( 0.5*wbz);
	A_(QUATERNION_X,QUATERNION_Z) = dt*(-0.5*wby);
	A_(QUATERNION_X,BIAS_GYRO_X)  =  0.5*dt*q0;
	A_(QUATERNION_X,BIAS_GYRO_Y)  = -0.5*dt*q3;
	A_(QUATERNION_X,BIAS_GYRO_Z)  = 0.5*dt*q2;

	A_(QUATERNION_Y,QUATERNION_W) = dt*( 0.5*wby);
	A_(QUATERNION_Y,QUATERNION_X) = dt*(-0.5*wbz);
	A_(QUATERNION_Y,QUATERNION_Z) = dt*( 0.5*wbx);
	A_(QUATERNION_Y,BIAS_GYRO_X)  = 0.5*dt*q3;
	A_(QUATERNION_Y,BIAS_GYRO_Y)  = 0.5*dt*q0;
	A_(QUATERNION_Y,BIAS_GYRO_Z)  = -0.5*dt*q1;

	A_(QUATERNION_Z,QUATERNION_W) = dt*( 0.5*wbz);
	A_(QUATERNION_Z,QUATERNION_X) = dt*( 0.5*wby);
	A_(QUATERNION_Z,QUATERNION_Y) = dt*(-0.5*wbx);
	A_(QUATERNION_Z,BIAS_GYRO_X)  = -0.5*dt*q2;
	A_(QUATERNION_Z,BIAS_GYRO_Y)  = 0.5*dt*q1;
	A_(QUATERNION_Z,BIAS_GYRO_Z)  = 0.5*dt*q0;

  if (measurement_status_ & (STATE_XY_POSITION | STATE_XY_VELOCITY)) {
    A_(VELOCITY_X,QUATERNION_W) = dt*(-2.0*q3*aby+2.0*q2*abz+2.0*q0*abx);
    A_(VELOCITY_X,QUATERNION_X) = dt*( 2.0*q2*aby+2.0*q3*abz+2.0*q1*abx);
    A_(VELOCITY_X,QUATERNION_Y) = dt*(-2.0*q2*abx+2.0*q1*aby+2.0*q0*abz);
    A_(VELOCITY_X,QUATERNION_Z) = dt*(-2.0*q3*abx-2.0*q0*aby+2.0*q1*abz);
//    A_(VELOCITY_X,BIAS_ACCEL_X) = dt*(q0*q0+q1*q1-q2*q2-q3*q3);
//    A_(VELOCITY_X,BIAS_ACCEL_Y) = dt*(2.0*q1*q2-2.0*q0*q3);
//    A_(VELOCITY_X,BIAS_ACCEL_Z) = dt*(2.0*q1*q3+2.0*q0*q2);

		A_(VELOCITY_Y,QUATERNION_W) = dt*(2.0*q3*abx-2.0*q1*abz+2.0*q0*aby);
		A_(VELOCITY_Y,QUATERNION_X) = dt*(2.0*q2*abx-2.0*q1*aby-2.0*q0*abz);
		A_(VELOCITY_Y,QUATERNION_Y) = dt*(2.0*q1*abx+2.0*q3*abz+2.0*q2*aby);
		A_(VELOCITY_Y,QUATERNION_Z) = dt*(2.0*q0*abx-2.0*q3*aby+2.0*q2*abz);
//		A_(VELOCITY_Y,BIAS_ACCEL_X) = dt*(2.0*q1*q2+2.0*q0*q3);
//		A_(VELOCITY_Y,BIAS_ACCEL_Y) = dt*(q0*q0-q1*q1+q2*q2-q3*q3);
//		A_(VELOCITY_Y,BIAS_ACCEL_Z) = dt*(2.0*q2*q3-2.0*q0*q1);
  } else {
    A_(VELOCITY_X,QUATERNION_W) = 0.0;
    A_(VELOCITY_X,QUATERNION_X) = 0.0;
    A_(VELOCITY_X,QUATERNION_Y) = 0.0;
    A_(VELOCITY_X,QUATERNION_Z) = 0.0;
    A_(VELOCITY_X,BIAS_ACCEL_X) = 0.0;
    A_(VELOCITY_X,BIAS_ACCEL_Y) = 0.0;
    A_(VELOCITY_X,BIAS_ACCEL_Z) = 0.0;

		A_(VELOCITY_Y,QUATERNION_W) = 0.0;
		A_(VELOCITY_Y,QUATERNION_X) = 0.0;
		A_(VELOCITY_Y,QUATERNION_Y) = 0.0;
		A_(VELOCITY_Y,QUATERNION_Z) = 0.0;
		A_(VELOCITY_Y,BIAS_ACCEL_X) = 0.0;
		A_(VELOCITY_Y,BIAS_ACCEL_Y) = 0.0;
		A_(VELOCITY_Y,BIAS_ACCEL_Z) = 0.0;
	}

	if (measurement_status_ & (STATE_Z_POSITION  | STATE_Z_VELOCITY)) {
		A_(VELOCITY_Z,QUATERNION_W) = dt*(-2.0*q2*abx+2.0*q1*aby+2.0*q0*abz);
		A_(VELOCITY_Z,QUATERNION_X) = dt*( 2.0*q3*abx+2.0*q0*aby-2.0*q1*abz);
		A_(VELOCITY_Z,QUATERNION_Y) = dt*(-2.0*q0*abx+2.0*q3*aby-2.0*q2*abz);
		A_(VELOCITY_Z,QUATERNION_Z) = dt*( 2.0*q1*abx+2.0*q2*aby+2.0*q3*abz);
		A_(VELOCITY_Z,BIAS_ACCEL_X) = dt*( 2.0*q1*q3-2.0*q0*q2);
		A_(VELOCITY_Z,BIAS_ACCEL_Y) = dt*( 2.0*q2*q3+2.0*q0*q1);
		A_(VELOCITY_Z,BIAS_ACCEL_Z) = dt*(q0*q0-q1*q1-q2*q2+q3*q3);
	} else {
		A_(VELOCITY_Z,QUATERNION_W) = 0.0;
		A_(VELOCITY_Z,QUATERNION_X) = 0.0;
		A_(VELOCITY_Z,QUATERNION_Y) = 0.0;
		A_(VELOCITY_Z,QUATERNION_Z) = 0.0;
		A_(VELOCITY_Z,BIAS_ACCEL_X) = 0.0;
		A_(VELOCITY_Z,BIAS_ACCEL_Y) = 0.0;
		A_(VELOCITY_Z,BIAS_ACCEL_Z) = 0.0;
	}

  if (measurement_status_ & STATE_XY_POSITION) {
    A_(POSITION_X,VELOCITY_X)   = dt;
    A_(POSITION_Y,VELOCITY_Y)   = dt;
  } else {
    A_(POSITION_X,VELOCITY_X)   = 0.0;
    A_(POSITION_Y,VELOCITY_Y)   = 0.0;
  }

  if (measurement_status_ & STATE_Z_POSITION) {
    A_(POSITION_Z,VELOCITY_Z)   = dt;
  } else {
    A_(POSITION_Z,VELOCITY_Z)   = 0.0;
  }
  //----------------------------------------------------------

  return A_;
}

void GenericQuaternionSystemModel::Limit(StateVector& x) const {
  normalize(x);
}

void GenericQuaternionSystemModel::normalize(StateVector& x) {
	double s = 1.0/sqrt(x(QUATERNION_W)*x(QUATERNION_W)+x(QUATERNION_X)*x(QUATERNION_X)+x(QUATERNION_Y)*x(QUATERNION_Y)+x(QUATERNION_Z)*x(QUATERNION_Z));
	x(QUATERNION_W) *= s;
	x(QUATERNION_X) *= s;
	x(QUATERNION_Y) *= s;
	x(QUATERNION_Z) *= s;
}

} // namespace hector_pose_estimation
