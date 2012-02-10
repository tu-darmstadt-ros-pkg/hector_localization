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

#include <hector_pose_estimation/measurements/magnetic.h>

namespace hector_pose_estimation {

MagneticModel::MagneticModel()
  : MeasurementModel(3)
  , declination_(0.0), inclination_(60.0 * M_PI/180.0), magnitude_(20.0)
  , C_full_(3,StateDimension)
{
  SymmetricMatrix noise(3);
  parameters().add("stddev", stddev_, 1.0);
  parameters().add("declination", declination_);
  parameters().add("inclination", inclination_);
  parameters().add("magnitude", magnitude_);
  noise(1,1) = noise(2,2) = noise(3,3) = pow(stddev_, 2);
  this->AdditiveNoiseSigmaSet(noise);
  C_full_= 0.0;
  init();
}

MagneticModel::~MagneticModel() {}

bool MagneticModel::init()
{
  setMagneticField(declination_, inclination_, magnitude_);
  return true;
}

SystemStatus MagneticModel::getStatusFlags() const {
  return STATE_YAW;
}

ColumnVector MagneticModel::ExpectedValueGet() const {
  const double qw = x_(QUATERNION_W);
  const double qx = x_(QUATERNION_X);
  const double qy = x_(QUATERNION_Y);
  const double qz = x_(QUATERNION_Z);

	y_(1) = (qw*qw+qx*qx-qy*qy-qz*qz) * magnetic_field_(1) + (2.0*qx*qy+2.0*qw*qz)     * magnetic_field_(2) + (2.0*qx*qz-2.0*qw*qy)     * magnetic_field_(3);
	y_(2) = (2.0*qx*qy-2.0*qw*qz)     * magnetic_field_(1) + (qw*qw-qx*qx+qy*qy-qz*qz) * magnetic_field_(2) + (2.0*qy*qz+2.0*qw*qx)     * magnetic_field_(3);
	y_(3) = (2.0*qx*qz+2.0*qw*qy)     * magnetic_field_(1) + (2.0*qy*qz-2.0*qw*qx)     * magnetic_field_(2) + (qw*qw-qx*qx-qy*qy+qz*qz) * magnetic_field_(3);

  return y_;
}

Matrix MagneticModel::dfGet(unsigned int i) const {
  if (i != 0) return Matrix();

  const double qw = x_(QUATERNION_W);
  const double qx = x_(QUATERNION_X);
  const double qy = x_(QUATERNION_Y);
  const double qz = x_(QUATERNION_Z);

	C_full_(1,QUATERNION_W) =  2.0*qw * magnetic_field_(1) + 2.0*qz * magnetic_field_(2) - 2.0*qy * magnetic_field_(3);
	C_full_(1,QUATERNION_X) =  2.0*qx * magnetic_field_(1) + 2.0*qy * magnetic_field_(2) + 2.0*qz * magnetic_field_(3);
	C_full_(1,QUATERNION_Y) = -2.0*qy * magnetic_field_(1) + 2.0*qx * magnetic_field_(2) - 2.0*qw * magnetic_field_(3);
	C_full_(1,QUATERNION_Z) = -2.0*qz * magnetic_field_(1) + 2.0*qw * magnetic_field_(2) + 2.0*qx * magnetic_field_(3);
	C_full_(2,QUATERNION_W) = -2.0*qz * magnetic_field_(1) + 2.0*qw * magnetic_field_(2) + 2.0*qx * magnetic_field_(3);
	C_full_(2,QUATERNION_X) =  2.0*qy * magnetic_field_(1) - 2.0*qx * magnetic_field_(2) + 2.0*qw * magnetic_field_(3);
	C_full_(2,QUATERNION_Y) =  2.0*qx * magnetic_field_(1) + 2.0*qy * magnetic_field_(2) + 2.0*qz * magnetic_field_(3);
	C_full_(2,QUATERNION_Z) = -2.0*qw * magnetic_field_(1) - 2.0*qz * magnetic_field_(2) + 2.0*qy * magnetic_field_(3);
	C_full_(3,QUATERNION_W) =  2.0*qy * magnetic_field_(1) - 2.0*qx * magnetic_field_(2) + 2.0*qw * magnetic_field_(3);
	C_full_(3,QUATERNION_X) =  2.0*qz * magnetic_field_(1) - 2.0*qw * magnetic_field_(2) - 2.0*qx * magnetic_field_(3);
	C_full_(3,QUATERNION_Y) =  2.0*qw * magnetic_field_(1) + 2.0*qz * magnetic_field_(2) - 2.0*qy * magnetic_field_(3);
	C_full_(3,QUATERNION_Z) =  2.0*qx * magnetic_field_(1) + 2.0*qy * magnetic_field_(2) + 2.0*qz * magnetic_field_(3);

	return C_full_;

	// dq/dyaw * dyaw*dq = 1/2 * [-qz -qy qx qw] * 2 * [-qz; -qy; qx; qw] =
	//  [ qz*qz  qz*qy -qz*qx -qz*qw ;
	//    qy*qz  qy*qy -qy*qx -qy*qw ;
	//   -qx*qz -qx*qy  qx*qx  qx*qw ;
	//   -qw*qz -qw*qy  qw*qx  qw*qw ]

	for(int i = 1; i <= 3; ++i) {
		C_(i,QUATERNION_W) =  C_full_(i,QUATERNION_W) * qz*qz + C_full_(i,QUATERNION_X) * qy*qz - C_full_(i,QUATERNION_Y) * qx*qz - C_full_(i,QUATERNION_Z) * qw*qz;
		C_(i,QUATERNION_X) =  C_full_(i,QUATERNION_W) * qz*qy + C_full_(i,QUATERNION_X) * qy*qy - C_full_(i,QUATERNION_Y) * qx*qy - C_full_(i,QUATERNION_Z) * qw*qy;
		C_(i,QUATERNION_Y) = -C_full_(i,QUATERNION_W) * qz*qx - C_full_(i,QUATERNION_X) * qy*qx + C_full_(i,QUATERNION_Y) * qx*qx + C_full_(i,QUATERNION_Z) * qw*qx;
		C_(i,QUATERNION_Z) = -C_full_(i,QUATERNION_W) * qz*qw - C_full_(i,QUATERNION_X) * qy*qw + C_full_(i,QUATERNION_Y) * qx*qw + C_full_(i,QUATERNION_Z) * qw*qw;
	}

  return C_;
}

void MagneticModel::setMagneticField(double declination, double inclination, double magnitude)
{
  magnetic_field_(1) = magnitude * cos(inclination_) * cos(declination_);
  magnetic_field_(2) = magnitude * -sin(declination_);
  magnetic_field_(3) = magnitude * -sin(inclination_) * cos(declination_);
}

} // namespace hector_pose_estimation
