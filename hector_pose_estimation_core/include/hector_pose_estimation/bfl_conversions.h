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

#ifndef HECTOR_POSE_ESTIMATION_BFL_CONVERSIONS_H
#define HECTOR_POSE_ESTIMATION_BFL_CONVERSIONS_H

#include <geometry_msgs/PoseWithCovariance.h>
#include <bfl/wrappers/matrix/matrix_wrapper.h>
#include <bfl/wrappers/matrix/vector_wrapper.h>

namespace hector_pose_estimation {

  void covarianceMsgToBfl(geometry_msgs::PoseWithCovariance::_covariance_type const &msg, MatrixWrapper::SymmetricMatrix_Wrapper &bfl) {
    unsigned int dim = sqrt(msg.size());
    bfl.resize(dim,false,false);
    for(unsigned int i = 0; i < dim; ++i)
      for(unsigned int j = 0; j < dim; ++j)
        bfl(i+1,j+1) = msg[i*dim+j];
  }

  void covarianceBflToMsg(MatrixWrapper::SymmetricMatrix const &bfl, geometry_msgs::PoseWithCovariance::_covariance_type &msg) {
    unsigned int dim = sqrt(msg.size());
    for(unsigned int i = 0; i < dim; ++i)
      for(unsigned int j = 0; j < dim; ++j)
        msg[i*6+j] = bfl(i+1,j+1);
  }

  void pointMsgToBfl(geometry_msgs::Point const &msg, MatrixWrapper::ColumnVector_Wrapper &bfl) {
    bfl.resize(3);
    bfl(1) = msg.x;
    bfl(2) = msg.y;
    bfl(3) = msg.z;
  }

  void pointBflToMsg(MatrixWrapper::ColumnVector_Wrapper const &bfl, geometry_msgs::Point &msg) {
    msg.x = bfl(1);
    msg.y = bfl(2);
    msg.z = bfl(3);
  }

  void quaternionMsgToBfl(geometry_msgs::Quaternion const &msg, MatrixWrapper::ColumnVector_Wrapper &bfl) {
    bfl.resize(4);
    bfl(1) = msg.w;
    bfl(2) = msg.x;
    bfl(3) = msg.y;
    bfl(4) = msg.z;
  }

  void quaternionBflToMsg(MatrixWrapper::ColumnVector_Wrapper const &bfl, geometry_msgs::Quaternion &msg) {
    msg.w = bfl(1);
    msg.x = bfl(2);
    msg.y = bfl(3);
    msg.z = bfl(4);
  }

} // namespace hector_pose_estimation

#endif // HECTOR_POSE_ESTIMATION_BFL_CONVERSIONS_H
