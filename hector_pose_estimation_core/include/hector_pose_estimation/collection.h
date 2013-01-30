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

#ifndef HECTOR_POSE_ESTIMATION_COLLECTION_H
#define HECTOR_POSE_ESTIMATION_COLLECTION_H

#include <vector>
#include <map>

#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

namespace hector_pose_estimation {

template <typename T>
class Collection {
public:
  typedef boost::shared_ptr<T> Ptr;
  typedef boost::weak_ptr<T> WPtr;
  typedef std::vector<Ptr> ListType;
  typedef std::map<std::string, WPtr> MapType;
  typedef typename ListType::const_iterator iterator;
  typedef typename ListType::const_iterator const_iterator;

  const Ptr& add(const Ptr& p) {
    list_.push_back(p);
    map_[p->getName()] = p;
    return p;
  }

  template <typename Derived> Ptr add(Derived *p) { return add(Ptr(p)); }

  template <typename Derived> const Ptr& create() {
    Derived *p = new Derived();
    return add(p);
  }

  template <typename Derived> const Ptr& create(const std::string& name) {
    Derived *p = new Derived(name);
    return add(p);
  }

  Ptr get(std::size_t index) const {
    if (index >= size()) return Ptr();
    return list_[index];

  }

  Ptr get(const std::string& name) const {
    if (!map_.count(name)) return Ptr();
    return map_.at(name).lock();
  }

  bool empty()           const { return list_.empty(); }
  std::size_t size()     const { return list_.size(); }
  const_iterator begin() const { return list_.begin(); }
  const_iterator end()   const { return list_.end(); }

  void clear() {
    map_.clear();
    list_.clear();
  }

private:
  ListType list_;
  MapType map_;
};

}

#endif // HECTOR_POSE_ESTIMATION_COLLECTION_H
