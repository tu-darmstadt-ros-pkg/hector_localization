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

#include <hector_pose_estimation/parameters.h>
#include <ros/node_handle.h>

#include <boost/algorithm/string.hpp>

namespace hector_pose_estimation {

template <typename T>
class RegisterParameterImpl {
public:
  static bool registerParam(ParameterPtr& parameter, ros::NodeHandle nh) {
    try {
      TypedParameter<T> p(*parameter);
      std::string param_key(boost::algorithm::to_lower_copy(parameter->key));
      if (!nh.getParam(param_key, p.value)) {
        nh.setParam(param_key, p.value);
        ROS_DEBUG_STREAM("Registered parameter " << param_key << " with new value " << p.value);
      } else {
        ROS_DEBUG_STREAM("Found parameter " << param_key << " with value " << p.value);
      }
      return true;
    } catch(std::bad_cast&) {
      return false;
    }
  }
};

static void registerParamRos(ParameterPtr& parameter, ros::NodeHandle nh) {
  if (RegisterParameterImpl<std::string>::registerParam(parameter, nh)) return;
  if (RegisterParameterImpl<double>::registerParam(parameter, nh)) return;
  if (RegisterParameterImpl<int>::registerParam(parameter, nh)) return;
  if (RegisterParameterImpl<bool>::registerParam(parameter, nh)) return;
  ROS_ERROR("Could not register parameter %s due to unknown type %s!", parameter->key.c_str(), parameter->type());
}

void ParameterList::registerParamsRos(ros::NodeHandle nh) const {
  registerParams(boost::bind(&registerParamRos, _1, nh));
}

void ParameterList::registerParams(const ParameterRegisterFunc& func) const {
  for(const_iterator it = begin(); it != end(); ++it) func(*it);
}

} // namespace hector_pose_estimation
