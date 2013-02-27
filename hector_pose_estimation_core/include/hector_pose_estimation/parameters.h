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
#include <boost/function.hpp>
#include <boost/type_traits.hpp>

namespace ros {
  class NodeHandle;
}

namespace hector_pose_estimation {

  class Parameter;
  template <typename T> class TypedParameter;
  typedef boost::shared_ptr<Parameter> ParameterPtr;
  typedef boost::shared_ptr<const Parameter> ParameterConstPtr;
  typedef boost::function<bool(ParameterPtr, std::string)> ParameterUpdateFunc;

  class Parameter {
  public:
    std::string key;
    Parameter(const std::string& key) : key(key) {}
    virtual ~Parameter() {}

    virtual ParameterPtr clone() = 0;
    virtual const char *type() const = 0;

    template <typename T> bool hasType() const {
      return dynamic_cast<const TypedParameter<T> *>(this) != 0;
    }

    template <typename T> T& as() const {
      const TypedParameter<T>& p = dynamic_cast<const TypedParameter<T> &>(*this);
      return p.value;
    }
  };

  template <typename T>
  class TypedParameter : public Parameter {
  public:
    typedef typename boost::remove_reference<typename boost::remove_const<T>::type>::type param_type;

    virtual ~TypedParameter() {}

    param_type& value;
    TypedParameter(const std::string& key, param_type &value) : Parameter(key), value(value) {}
    TypedParameter(const Parameter& other) : Parameter(other), value(dynamic_cast<const TypedParameter<T> &>(other).value) {}

    ParameterPtr clone() { return ParameterPtr(new TypedParameter<T>(*this)); }
    const char *type() const { return typeid(param_type).name(); }
  };

  class ParameterList : public std::list<ParameterPtr> {
  public:
    using std::list<ParameterPtr>::iterator;
    using std::list<ParameterPtr>::const_iterator;

    ~ParameterList() {}

    template <typename T> ParameterList& add(const std::string& key, T& value, const T& default_value) {
      value = default_value;
      return add(key, value);
    }

    template <typename T> ParameterList& add(const std::string& key, T* value) {
      return add(key, *value);
    }

    template <typename T> ParameterList& add(const std::string& key, T& value) {
      return add(ParameterPtr(new TypedParameter<T>(key, value)));
    }

    ParameterList& add(ParameterPtr const& parameter);
    ParameterList& add(ParameterList const& other);

    ParameterList& copy(const std::string& prefix, ParameterList const& parameters);
    ParameterList& copy(ParameterList const& parameters);

    ParameterPtr const& get(const std::string& key) const;
    template <typename T> T& getAs(const std::string& key) const {
      return get(key)->as<T>();
    }

    using std::list<ParameterPtr>::erase;
    iterator erase(const std::string& key);

    bool update();
    void setRegistry(const ParameterUpdateFunc& func, bool recursive = true, bool update = true);
    void setNodeHandle(const ros::NodeHandle &nh, bool update = true);

  private:
    ParameterUpdateFunc register_func_;
    bool register_recursive_;

    std::string prefix_;
    static const std::string s_separator;

    bool update(const ParameterPtr& parameter);
  };

  static inline ParameterList operator+(ParameterList const& list1, ParameterList const& list2)
  {
    ParameterList result;
    return result.add(list1).add(list2);
  }

} // namespace hector_pose_estimation

#endif // HECTOR_POSE_ESTIMATION_PARAMETERS_H
