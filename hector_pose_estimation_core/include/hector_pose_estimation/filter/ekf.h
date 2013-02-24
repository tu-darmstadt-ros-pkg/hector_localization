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

#ifndef HECTOR_POSE_ESTIMATION_FILTER_EKF_H
#define HECTOR_POSE_ESTIMATION_FILTER_EKF_H

#include <hector_pose_estimation/filter.h>

namespace hector_pose_estimation {
namespace filter {

class EKF : public Filter
{
public:
  EKF(State& state) : Filter(state) {}
  virtual ~EKF() {}

  std::string getType() const { return "EKF"; }

  template <typename ConcreteModel, typename Enabled = void>
  class PredictorImpl_ : public Filter::template Predictor_<ConcreteModel>
  {
  public:
    typedef ConcreteModel Model;
    typedef typename Filter::template Predictor_<ConcreteModel> Base;

    PredictorImpl_(EKF *filter, Model *model) : Base(filter, model), filter_(filter) {}
    virtual ~PredictorImpl_() {}

    virtual bool predict(State &state, double dt);

  protected:
    EKF *filter_;

  public:
    typename Model::NoiseVariance Q;
  };

  template <typename ConcreteModel>
  class PredictorImpl_<ConcreteModel, typename boost::enable_if<typename ConcreteModel::IsSubSystem >::type> : public Filter::template Predictor_<ConcreteModel>
  {
  public:
    typedef ConcreteModel Model;
    typedef typename Filter::template Predictor_<ConcreteModel> Base;

    PredictorImpl_(EKF *filter, Model *model) : Base(filter, model), filter_(filter) {}
    virtual ~PredictorImpl_() {}

    virtual bool predict(State &state, double dt);

  protected:
    EKF *filter_;

  public:
    typename Model::StateVector x_pred;
    typename Model::NoiseVariance Q;
    typename Model::SystemMatrix A1;
    typename Model::CrossSystemMatrix A01;
  };

  template <typename ConcreteModel, typename Enabled = void>
  class CorrectorImpl_ : public Filter::template Corrector_<ConcreteModel>
  {
  public:
    typedef ConcreteModel Model;
    typedef typename Filter::template Corrector_<ConcreteModel> Base;

    CorrectorImpl_(EKF *filter, Model *model) : Base(filter, model), filter_(filter) {}
    virtual ~CorrectorImpl_() {}

    virtual bool correct(State &state, const typename ConcreteModel::MeasurementVector& y, const typename ConcreteModel::NoiseVariance& R);

  protected:
    EKF *filter_;

  public:
    typename Model::MeasurementVector y_pred;
    typename Model::MeasurementVector error;
    typename Model::MeasurementMatrix C;
    typename Model::GainMatrix K;
  };

  template <typename ConcreteModel>
  class CorrectorImpl_<ConcreteModel, typename boost::enable_if<typename ConcreteModel::HasSubSystem >::type> : public Filter::template Corrector_<ConcreteModel>
  {
  public:
    typedef ConcreteModel Model;
    typedef typename Filter::template Corrector_<ConcreteModel> Base;

    CorrectorImpl_(EKF *filter, Model *model) : Base(filter, model), filter_(filter) {}
    virtual ~CorrectorImpl_() {}

    virtual bool correct(State &state, const typename ConcreteModel::MeasurementVector& y, const typename ConcreteModel::NoiseVariance& R);

  protected:
    EKF *filter_;

  public:
    typename Model::MeasurementVector y_pred;
    typename Model::MeasurementVector error;
    typename Model::MeasurementMatrix C;
    typename Model::GainMatrix K;
    typename Model::CrossMeasurementMatrix C1;
    typename Model::CrossGainMatrix K1;
  };

  template <typename ConcreteModel>
  class Predictor_ : public PredictorImpl_<ConcreteModel> {
  public:
     Predictor_(EKF *filter, ConcreteModel *model) : PredictorImpl_<ConcreteModel>(filter, model) {}
     virtual ~Predictor_() {}
  };

  template <typename ConcreteModel>
  class Corrector_ : public CorrectorImpl_<ConcreteModel> {
  public:
     Corrector_(EKF *filter, ConcreteModel *model) : CorrectorImpl_<ConcreteModel>(filter, model) {}
     virtual ~Corrector_() {}
  };

public:
  ColumnVector_<State::Dimension> x_pred;
  Matrix_<State::Dimension,State::Dimension> A;
};

} // namespace filter
} // namespace hector_pose_estimation

#include <hector_pose_estimation/filter/ekf.inl>

#endif // HECTOR_POSE_ESTIMATION_FILTER_EKF_H
