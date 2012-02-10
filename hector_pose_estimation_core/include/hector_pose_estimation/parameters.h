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

#ifndef HECTOR_POSE_ESTIMATION_PARAMETERS_H
#define HECTOR_POSE_ESTIMATION_PARAMETERS_H

#include <list>
#include <string>
#include <boost/shared_ptr.hpp>

namespace ros {
  class NodeHandle;
}

namespace hector_pose_estimation {

  class Parameter {
  public:
    std::string key;
    Parameter(const std::string& key) : key(key) {}
    virtual Parameter *clone() = 0;
    virtual const char *type() const = 0;

    void registerParam(ros::NodeHandle nh);

  private:
    template <typename T> class RegisterParameterImpl;
  };

  typedef boost::shared_ptr<Parameter> ParameterPtr;
  typedef boost::shared_ptr<const Parameter> ParameterConstPtr;

  template <typename T>
  class TypedParameter : public Parameter {
  public:
    T& value;
    TypedParameter(const std::string& key, T &value) : Parameter(key), value(value) {}
    TypedParameter(const Parameter& other) : Parameter(other), value(dynamic_cast<const TypedParameter<T> &>(other).value) {}

    Parameter *clone() { return new TypedParameter<T>(*this); }
    const char *type() const { return typeid(T).name(); }
  };

  class ParameterList : public std::list<ParameterPtr> {
  public:
    using std::list<ParameterPtr>::iterator;
    using std::list<ParameterPtr>::const_iterator;

    ParameterList() {}
    ~ParameterList() {}

    template <typename T>
    ParameterList& add(const std::string& key, T& value, const T& default_value) {
      value = default_value;
      return add(key, value);
    }

    template <typename T>
    ParameterList& add(const std::string& key, T& value) {
      erase(key);
      push_back(ParameterPtr(new TypedParameter<T>(key, value)));
      return *this;
    }

    template <typename T>
    ParameterList& add(const std::string& key, T* value) {
      erase(key);
      push_back(ParameterPtr(new TypedParameter<T>(key, *value)));
      return *this;
    }

    ParameterList& add(ParameterList const& other) {
      for(ParameterList::const_iterator it = other.begin(); it != other.end(); ++it) push_back(*it);
      return *this;
    }

    ParameterList& copy(const std::string& prefix, ParameterList const& parameters) {
      for(ParameterList::const_iterator it = parameters.begin(); it != parameters.end(); ++it) {
        ParameterPtr copy((*it)->clone());
        if (!prefix.empty()) copy->key = prefix + "/" + copy->key;
        push_back(copy);
      }
      return *this;
    }

    ParameterList& copy(ParameterList const& parameters) {
      copy(std::string(), parameters);
      return *this;
    }

    template <typename T>
    T& get(const std::string& key) const {
      for(iterator it = begin(); it != end(); ++it) {
        if ((*it)->key == key) {
          return boost::shared_dynamic_cast<TypedParameter<T> &>(*it)->value;
        }
      }
      throw std::bad_cast();
    }

    using std::list<ParameterPtr>::erase;
    iterator erase(const std::string& key) {
      iterator it = begin();
      for(; it != end(); ++it) {
        if ((*it)->key == key) return erase(it);
      }
      return it;
    }

    void registerParams(ros::NodeHandle nh);
  };

  static inline ParameterList operator+(ParameterList const& list1, ParameterList const& list2)
  {
    ParameterList result;
    return result.add(list1).add(list2);
  }

} // namespace hector_pose_estimation

#endif // HECTOR_POSE_ESTIMATION_PARAMETERS_H
