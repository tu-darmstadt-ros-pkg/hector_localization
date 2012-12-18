//=================================================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
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

#include "services.h"

namespace hector_pose_estimation {

SystemService::SystemService(RTT::TaskContext *owner, const SystemPtr& system, const std::string& name)
  : RTT::Service(name.empty() ? system->getName() : name, owner)
{
  system->parameters().registerParams(boost::bind(&registerParamAsProperty, _1, this->properties()));
}

SystemService::~SystemService()
{}

MeasurementService::MeasurementService(RTT::TaskContext *owner, const MeasurementPtr& measurement, const std::string& name)
  : RTT::Service(name.empty() ? measurement->getName() : name, owner)
{
  measurement->parameters().registerParams(boost::bind(&registerParamAsProperty, _1, this->properties()));
}

MeasurementService::~MeasurementService()
{}

void registerParamAsProperty(ParameterPtr &parameter, RTT::PropertyBag *bag) {
  bag->removeProperty(bag->getProperty(parameter->key));
  if (parameter->hasType<std::string>()) bag->addProperty(parameter->key, parameter->as<std::string>());
  if (parameter->hasType<double>()) bag->addProperty(parameter->key, parameter->as<double>());
  if (parameter->hasType<int>()) bag->addProperty(parameter->key, parameter->as<int>());
  if (parameter->hasType<bool>()) bag->addProperty(parameter->key, parameter->as<bool>());
}

} // namespace hector_pose_estimation
