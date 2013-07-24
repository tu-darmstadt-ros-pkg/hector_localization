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
  EKF();
  virtual ~EKF();

  virtual std::string getType() const { return "EKF"; }

  virtual bool init(PoseEstimation &estimator);
  virtual bool doPredict(double dt);

  template <class ConcreteModel, typename Enabled = void>
  class PredictorImpl_ : public Filter::template Predictor_<ConcreteModel>
  {
  public:
    typedef ConcreteModel Model;
    typedef typename Filter::template Predictor_<ConcreteModel> Base;
    using Filter::template Predictor_<ConcreteModel>::state;

    PredictorImpl_(EKF *filter, Model *model)
      : Base(filter, model)
      , filter_(filter)
      , x_pred(filter_->x_pred.segment<State::Dimension>(0))
      , A(filter_->A.block<State::Dimension,State::Dimension>(0,0))
      , Q(filter->Q.block<State::Dimension,State::Dimension>(0,0))
    {}
    virtual ~PredictorImpl_() {}

    virtual bool predict(double dt);

  protected:
    EKF *filter_;

  public:
    typename Model::StateVectorSegment x_pred;
    typename Model::SystemMatrixBlock A;
    typename Model::NoiseVarianceBlock Q;
  };

  template <class ConcreteModel>
  class PredictorImpl_<ConcreteModel, typename boost::enable_if<class ConcreteModel::IsSubSystem >::type> : public Filter::template Predictor_<ConcreteModel>
  {
  public:
    typedef ConcreteModel Model;
    typedef typename Filter::template Predictor_<ConcreteModel> Base;
    using Filter::template Predictor_<ConcreteModel>::state;
    using Filter::template Predictor_<ConcreteModel>::sub;

    PredictorImpl_(EKF *filter, Model *model)
      : Base(filter, model)
      , filter_(filter)
      , x_pred(filter_->x_pred.segment<ConcreteModel::Dimension>(model->getStateIndex(state())))
      , A11(filter_->A.block<ConcreteModel::Dimension,ConcreteModel::Dimension>(model->getStateIndex(state()), model->getStateIndex(state())))
      , A01(filter_->A.block<State::Dimension,ConcreteModel::Dimension>(0, model->getStateIndex(state())))
      , Q1(filter->Q.block<ConcreteModel::Dimension,ConcreteModel::Dimension>(model->getStateIndex(state()), model->getStateIndex(state())))
    {}
    virtual ~PredictorImpl_() {}

    virtual bool predict(double dt);

  protected:
    EKF *filter_;

  public:
    typename Model::StateVectorSegment x_pred;
    typename Model::SystemMatrixBlock A11;
    typename Model::CrossSystemMatrixBlock A01;
    typename Model::NoiseVarianceBlock Q1;
  };

  template <class ConcreteModel, typename Enabled = void>
  class CorrectorImpl_ : public Filter::template Corrector_<ConcreteModel>
  {
  public:
    typedef ConcreteModel Model;
    typedef typename Filter::template Corrector_<ConcreteModel> Base;
    using Filter::template Corrector_<ConcreteModel>::state;

    CorrectorImpl_(EKF *filter, Model *model) : Base(filter, model), filter_(filter) {}
    virtual ~CorrectorImpl_() {}

    virtual bool correct(const typename ConcreteModel::MeasurementVector& y, const typename ConcreteModel::NoiseVariance& R);

  protected:
    EKF *filter_;

  public:
    typename Model::MeasurementVector y_pred;
    typename Model::MeasurementVector error;
    typename Model::MeasurementMatrix C;
    Matrix_<ConcreteModel::MeasurementDimension, Dynamic> CP;
    typename Model::NoiseVariance S;
    typename Model::GainMatrix K;
  };

  template <class ConcreteModel>
  class CorrectorImpl_<ConcreteModel, typename boost::enable_if<class ConcreteModel::HasSubSystem >::type> : public Filter::template Corrector_<ConcreteModel>
  {
  public:
    typedef ConcreteModel Model;
    typedef typename Filter::template Corrector_<ConcreteModel> Base;
    using Filter::template Corrector_<ConcreteModel>::state;
    using Filter::template Corrector_<ConcreteModel>::sub;

    CorrectorImpl_(EKF *filter, Model *model) : Base(filter, model), filter_(filter) {}
    virtual ~CorrectorImpl_() {}

    virtual bool correct(const typename ConcreteModel::MeasurementVector& y, const typename ConcreteModel::NoiseVariance& R);

  protected:
    EKF *filter_;

  public:
    typename Model::MeasurementVector y_pred;
    typename Model::MeasurementVector error;
    typename Model::MeasurementMatrix C0;
    typename Model::SubMeasurementMatrix C1;
    Matrix_<ConcreteModel::MeasurementDimension, Dynamic> CP;
    typename Model::NoiseVariance S;
    typename Model::GainMatrix K;
  };

  template <class ConcreteModel>
  class Predictor_ : public PredictorImpl_<ConcreteModel> {
  public:
     Predictor_(EKF *filter, ConcreteModel *model) : PredictorImpl_<ConcreteModel>(filter, model) {}
     virtual ~Predictor_() {}
  };

  template <class ConcreteModel>
  class Corrector_ : public CorrectorImpl_<ConcreteModel> {
  public:
     Corrector_(EKF *filter, ConcreteModel *model) : CorrectorImpl_<ConcreteModel>(filter, model) {}
     virtual ~Corrector_() {}
  };

public:
  State::Vector x_pred;
  Matrix A;
  SymmetricMatrix Q;
};

} // namespace filter
} // namespace hector_pose_estimation

#include "ekf.inl"

#endif // HECTOR_POSE_ESTIMATION_FILTER_EKF_H
