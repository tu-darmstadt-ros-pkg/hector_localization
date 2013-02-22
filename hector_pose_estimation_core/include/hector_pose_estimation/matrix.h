//=================================================================================================
// Copyright (c) 2011, Johannes Meyer and Martin Nowara, TU Darmstadt
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

#ifndef HECTOR_POSE_ESTIMATION_MATRIX_H
#define HECTOR_POSE_ESTIMATION_MATRIX_H

#include <Eigen/Geometry>
#include <stdexcept>

namespace hector_pose_estimation {
  using Eigen::Dynamic;
  using Eigen::Lower;
  using Eigen::Upper;

  typedef double ScalarType;
  typedef Eigen::DenseIndex IndexType;
  typedef Eigen::Matrix<ScalarType,Dynamic,1> ColumnVector;
  typedef Eigen::Matrix<ScalarType,1,Dynamic> RowVector;
  typedef Eigen::Matrix<ScalarType,Dynamic,Dynamic> Matrix;
  typedef Eigen::Quaternion<ScalarType> Quaternion;

  using Eigen::VectorBlock;
  typedef VectorBlock<ColumnVector,3>       VectorBlock3;
  typedef VectorBlock<ColumnVector,4>       VectorBlock4;
  typedef VectorBlock<const ColumnVector,3> ConstVectorBlock3;
  typedef VectorBlock<const ColumnVector,4> ConstVectorBlock4;

  using Eigen::Block;
  typedef Eigen::Block<Matrix,Dynamic,Dynamic> MatrixBlock;

  template <int _Rows>
  class ColumnVector_ : public Eigen::Matrix<ScalarType,_Rows,1> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF(_Rows != Dynamic) // is this really required if Eigen::Matrix is the base class?

    typedef Eigen::Matrix<ScalarType,_Rows,1> Base;
    typedef typename Eigen::internal::traits<Base>::Scalar Scalar;
    typedef typename Eigen::internal::traits<Base>::Index Index;

    ColumnVector_() {}
    ColumnVector_(Scalar value) { this->setConstant(value); }
    ColumnVector_(Scalar x, Scalar y, Scalar z) : Eigen::Matrix<ScalarType,_Rows,1>(x, y, z) {}
    template <typename Derived> ColumnVector_(const Eigen::MatrixBase<Derived>& other) : Eigen::Matrix<ScalarType,_Rows,1>(other) {}
  };

  template <int _Cols>
  class RowVector_ : public Eigen::Matrix<ScalarType,1,_Cols> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF(_Cols != Dynamic) // is this really required if Eigen::Matrix is the base class?

    typedef Eigen::Matrix<ScalarType,1,_Cols> Base;
    typedef typename Eigen::internal::traits<Base>::Scalar Scalar;
    typedef typename Eigen::internal::traits<Base>::Index Index;

    RowVector_() {}
    RowVector_(Scalar value) { this->setConstant(value); }
    RowVector_(Scalar x, Scalar y, Scalar z) : Eigen::Matrix<ScalarType,1,_Cols>(x, y, z) {}
    template <typename Derived> RowVector_(const Eigen::MatrixBase<Derived>& other) : Eigen::Matrix<ScalarType,1,_Cols>(other) {}
  };

  template <int _Rows, int _Cols>
  class Matrix_ : public Eigen::Matrix<ScalarType,_Rows,_Cols> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF(_Rows != Dynamic || _Cols != Dynamic) // is this really required if Eigen::Matrix is the base class?

    typedef Eigen::Matrix<ScalarType,_Rows,_Cols> Base;
    typedef typename Eigen::internal::traits<Base>::Scalar Scalar;
    typedef typename Eigen::internal::traits<Base>::Index Index;

    Matrix_() {}
    Matrix_(Scalar value) { this->setConstant(value); }
    template <typename Derived> Matrix_(const Eigen::MatrixBase<Derived>& other) : Eigen::Matrix<ScalarType,_Rows,_Cols>(other) {}

  protected:
    explicit Matrix_(Index rows, Index cols) : Eigen::Matrix<ScalarType,_Rows,_Cols>(rows, cols) {}
  };

//  namespace internal {
//    template <int _Dimension> struct SymmetricMatrixHelper {
//      SymmetricMatrixHelper() {}
//      SymmetricMatrixHelper(Index dim) : storage_(dim,dim) {}
//      template <typename Derived> SymmetricMatrixHelper(const Eigen::MatrixBase<Derived>& other) : storage_(other) {}

//      typedef Eigen::Matrix<ScalarType,_Dimension,_Dimension> StorageType;
//      typedef Eigen::SelfAdjointView<StorageType,Upper> SelfAdjointViewType;
//      StorageType storage_;
//    };
//  }

//  template <int _Dimension>
//  class SymmetricMatrix_ : public internal::SymmetricMatrixHelper<_Dimension>, public internal::SymmetricMatrixHelper<_Dimension>::SelfAdjointViewType {
//  public:
//    typedef typename internal::SymmetricMatrixHelper<_Dimension>::StorageType Base;

//    // Constructors
//    SymmetricMatrix_() : internal::SymmetricMatrixHelper<_Dimension>::SelfAdjointViewType(storage_) {}
//    SymmetricMatrix_(Scalar value) : internal::SymmetricMatrixHelper<_Dimension>::SelfAdjointViewType(storage_) { storage_.setConstant(value); }
//    SymmetricMatrix_(Index dim, Scalar value) : internal::SymmetricMatrixHelper<_Dimension>(dim), internal::SymmetricMatrixHelper<_Dimension>::SelfAdjointViewType(storage_) { storage_.setConstant(value); }
//    SymmetricMatrix_(Index dim) : internal::SymmetricMatrixHelper<_Dimension>(dim), internal::SymmetricMatrixHelper<_Dimension>::SelfAdjointViewType(storage_) {}
//    template <typename Derived> SymmetricMatrix_(const Eigen::MatrixBase<Derived>& other) : internal::SymmetricMatrixHelper<_Dimension>(other), internal::SymmetricMatrixHelper<_Dimension>::SelfAdjointViewType(storage_) {}

//    SymmetricMatrix_& setZero() {
//      storage_.setZero();
//      return *this;
//    }
//  };

  template <int _Dimension>
  class SymmetricMatrix_ : public Matrix_<_Dimension,_Dimension> /* , public Eigen::SelfAdjointView<typename Matrix_<_Dimension,_Dimension>::Base,Upper> */ {
  public:
    typedef Eigen::Matrix<ScalarType,_Dimension,_Dimension> Base;
    typedef Eigen::SelfAdjointView<typename Matrix_<_Dimension,_Dimension>::Base,Upper> SelfAdjointViewType;
    typedef typename Eigen::internal::traits<Base>::Scalar Scalar;
    typedef typename Eigen::internal::traits<Base>::Index Index;

    // Constructors
    SymmetricMatrix_() /* : SelfAdjointViewType(static_cast<Matrix_<_Dimension,_Dimension>&>(*this)) */ {}
    SymmetricMatrix_(Scalar value) /* : SelfAdjointViewType(static_cast<Matrix_<_Dimension,_Dimension>&>(*this)) */ { this->setConstant(value); }
    template <typename Derived> SymmetricMatrix_(const Eigen::MatrixBase<Derived>& other) : Matrix_<_Dimension,_Dimension>(other) /* , SelfAdjointViewType(static_cast<Matrix_<_Dimension,_Dimension>&>(*this)) */ {}

//    SymmetricMatrix_& setZero() {
//      this->Matrix_<_Dimension,_Dimension>::setZero();
//      return *this;
//    }

    // using SelfAdjointViewType::operator();

  protected:
    explicit SymmetricMatrix_(Index dim) : Matrix_<_Dimension,_Dimension>(dim, dim) /*, SelfAdjointViewType(static_cast<Matrix_<_Dimension,_Dimension>&>(*this)) */ {}
  };

  class SymmetricMatrix : public SymmetricMatrix_<Dynamic> {
  public:
    // Constructors
    SymmetricMatrix() : SymmetricMatrix_<Dynamic>() {}
    SymmetricMatrix(Index dim) : SymmetricMatrix_<Dynamic>(dim) {}
    SymmetricMatrix(Index dim, Scalar value) : SymmetricMatrix_<Dynamic>(dim) { this->setConstant(value); }
    template <typename Derived> SymmetricMatrix(const Eigen::MatrixBase<Derived>& other) : SymmetricMatrix_<Dynamic>(other) {}
  };

} // namespace hector_pose_estimation

#endif // HECTOR_POSE_ESTIMATION_MATRIX_H
