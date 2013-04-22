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

const std::string ParameterList::s_separator = "/";

namespace {
  struct null_deleter {
    void operator()(void const *) const {}
  };
}

ParameterList& ParameterList::add(ParameterPtr const& parameter) {
  erase(parameter->key);
  push_back(parameter);
  update(parameter);
  return *this;
}

ParameterList& ParameterList::add(ParameterList const& other) {
  for(ParameterList::const_iterator it = other.begin(); it != other.end(); ++it) push_back(*it);
  return *this;
}

ParameterList& ParameterList::add(Parameter& alias, const std::string& key) {
  if (!key.empty()) alias.key = key;
  return add(ParameterPtr(&alias, null_deleter()));
}

ParameterList& ParameterList::copy(const std::string& prefix, ParameterList const& parameters) {
  for(ParameterList::const_iterator it = parameters.begin(); it != parameters.end(); ++it) {
    ParameterPtr copy((*it)->clone());
    if (!prefix.empty()) copy->key = prefix + s_separator + copy->key;
    push_back(copy);
  }
  return *this;
}

ParameterList& ParameterList::copy(ParameterList const& parameters) {
  copy(std::string(), parameters);
  return *this;
}

ParameterPtr const& ParameterList::get(const std::string& key) const {
  for(const_iterator it = begin(); it != end(); ++it) {
    if ((*it)->key == key) {
      return *it;
    }
  }
  throw std::runtime_error("parameter not found");
}

ParameterList::iterator ParameterList::erase(const std::string& key) {
  iterator it = begin();
  for(; it != end(); ++it) {
    if ((*it)->key == key) return erase(it);
  }
  return it;
}

namespace internal {
  template <typename T>
  static bool registerParamRosTyped(ParameterPtr& parameter, std::string key, const ros::NodeHandle &nh, bool set_all = false) {
    try {
      ParameterT<T> p(*parameter);
      boost::algorithm::to_lower(key);
      if (!nh.getParam(key, p.value())) {
        if (set_all) nh.setParam(key, p.value());
        ROS_DEBUG_STREAM("Registered parameter " << key << " with new value " << p.value());
      } else {
        ROS_DEBUG_STREAM("Found parameter " << key << " with value " << p.value());
      }
      return true;

    } catch(std::bad_cast&) {
      return false;
    }
  }

  static bool getSetAllFlag(const ros::NodeHandle &nh) {
    bool set_all = false;
    nh.getParam("set_all_parameters", set_all);
    return set_all;
  }

  static bool registerParamRos(ParameterPtr& parameter, const std::string& key, const ros::NodeHandle &nh) {
    static bool set_all = getSetAllFlag(nh);

    if (registerParamRosTyped<std::string>(parameter, key, nh, set_all)) return true;
    if (registerParamRosTyped<double>(parameter, key, nh, set_all)) return true;
    if (registerParamRosTyped<int>(parameter, key, nh, set_all)) return true;
    if (registerParamRosTyped<bool>(parameter, key, nh, set_all)) return true;
    ROS_ERROR("Could not register parameter %s due to unknown type %s!", key.c_str(), parameter->type());
    return false;
  }
}

void ParameterList::setRegistry(const ParameterUpdateFunc& func, bool recursive, bool update) {
  register_func_ = func;
  register_recursive_ = recursive;
  if (update) this->update();
}

void ParameterList::setNodeHandle(const ros::NodeHandle &nh, bool update) {
  setRegistry(boost::bind(&internal::registerParamRos, _1, _2, ros::NodeHandle(nh)), update);
}

bool ParameterList::update() {
  bool result = true;
  for(const_iterator it = begin(); it != end(); ++it) result &= update(*it);
  return result;
}

bool ParameterList::update(const ParameterPtr& parameter) {
  if (parameter->empty()) return false;
  if (parameter->isAlias()) return true;

  std::string key = parameter->key;
  if (!prefix_.empty()) key = prefix_ + s_separator + key;

  if (parameter->hasType<ParameterList>()) {
    if (!register_recursive_) return true;
    ParameterList& nested = parameter->as<ParameterList>();
    nested.prefix_ = key;
    nested.setRegistry(register_func_, false);
    return nested.update();
  }

  if (!register_func_) return false;
  return register_func_(parameter, key);
}

} // namespace hector_pose_estimation
