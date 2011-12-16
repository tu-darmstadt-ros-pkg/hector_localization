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

namespace hector_pose_estimation {

void ParameterList::registerParams(ros::NodeHandle nh) {
  for(const_iterator it = begin(); it != end(); ++it) {
    (*it)->registerParam(nh);
  }
}

template <typename T>
class Parameter::RegisterParameterImpl {
public:
  static bool registerParam(ros::NodeHandle nh, Parameter const& parameter) {
    try {
      TypedParameter<T> p(parameter);
      // nh.getParam(p.key, p.value);
      if (!nh.getParam(p.key, p.value)) nh.setParam(p.key, p.value);
      return true;
    } catch(std::bad_cast&) {
      return false;
    }
  }
};
template class Parameter::RegisterParameterImpl<std::string>;
template class Parameter::RegisterParameterImpl<double>;
template class Parameter::RegisterParameterImpl<int>;
template class Parameter::RegisterParameterImpl<bool>;

void Parameter::registerParam(ros::NodeHandle nh) {
  if (Parameter::RegisterParameterImpl<std::string>::registerParam(nh, *this)) return;
  if (Parameter::RegisterParameterImpl<double>::registerParam(nh, *this)) return;
  if (Parameter::RegisterParameterImpl<int>::registerParam(nh, *this)) return;
  if (Parameter::RegisterParameterImpl<bool>::registerParam(nh, *this)) return;
  ROS_ERROR("Could not register parameter %s due to unknown type %s!", key.c_str(), type());
}

} // namespace hector_pose_estimation
