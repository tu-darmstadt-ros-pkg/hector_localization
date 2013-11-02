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

#define ASSERT_SYMMETRIC_MATRIX_TO_BE_SYMMETRIC
#define ASSERT_SYMMETRIC_MATRIX_TO_BE_SYMMETRIC_PRECISION 1e-5
#define FORCE_SYMMETRIC_MATRIX_TO_BE_SYMMETRIC

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

  typedef Eigen::Matrix<ScalarType,3,1> Vector3;
  typedef Eigen::Matrix<ScalarType,4,1> Vector4;

  using Eigen::VectorBlock;
  typedef VectorBlock<ColumnVector,3>       VectorBlock3;
  typedef VectorBlock<ColumnVector,4>       VectorBlock4;
  typedef VectorBlock<const ColumnVector,3> ConstVectorBlock3;
  typedef VectorBlock<const ColumnVector,4> ConstVectorBlock4;

  using Eigen::Block;
  typedef Eigen::Block<Matrix,Dynamic,Dynamic> MatrixBlock;

  template <int Rows>
  class ColumnVector_ : public Eigen::Matrix<ScalarType,Rows,1> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF(Rows != Dynamic) // is this really required if Eigen::Matrix is the base class?

    typedef ColumnVector_<Rows> Derived;
    typedef Eigen::Matrix<ScalarType,Rows,1> Base;
    typedef typename Eigen::internal::traits<Base>::Scalar Scalar;
    typedef typename Eigen::internal::traits<Base>::Index Index;
    typedef Eigen::Map<Base> Map;
    typedef Eigen::Map<const Base> ConstMap;

    ColumnVector_() { this->setZero(); }
    ColumnVector_(Scalar value) { this->setConstant(value); }
    ColumnVector_(Scalar x, Scalar y, Scalar z) : Eigen::Matrix<ScalarType,Rows,1>(x, y, z) {}
    template <typename OtherDerived> ColumnVector_(const Eigen::MatrixBase<OtherDerived>& other) : Base(other) {}

    EIGEN_INHERIT_ASSIGNMENT_EQUAL_OPERATOR(Derived)
  };
  typedef ColumnVector_<3> ColumnVector3;

  template <int Cols>
  class RowVector_ : public Eigen::Matrix<ScalarType,1,Cols> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF(Cols != Dynamic) // is this really required if Eigen::Matrix is the base class?

    typedef RowVector_<Cols> Derived;
    typedef Eigen::Matrix<ScalarType,1,Cols> Base;
    typedef typename Eigen::internal::traits<Base>::Scalar Scalar;
    typedef typename Eigen::internal::traits<Base>::Index Index;
    typedef Eigen::Map<Base> Map;
    typedef Eigen::Map<const Base> ConstMap;

    RowVector_() { this->setZero(); }
    RowVector_(Scalar value) { this->setConstant(value); }
    RowVector_(Scalar x, Scalar y, Scalar z) : Eigen::Matrix<ScalarType,1,Cols>(x, y, z) {}
    template <typename OtherDerived> RowVector_(const Eigen::MatrixBase<OtherDerived>& other) : Base(other) {}

    EIGEN_INHERIT_ASSIGNMENT_EQUAL_OPERATOR(Derived)
  };
  typedef RowVector_<3> RowVector3;

  template <int Rows, int Cols>
  class Matrix_ : public Eigen::Matrix<ScalarType,Rows,Cols> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF(Rows != Dynamic || Cols != Dynamic) // is this really required if Eigen::Matrix is the base class?

    typedef Matrix_<Rows,Cols> Derived;
    typedef Eigen::Matrix<ScalarType,Rows,Cols> Base;
    typedef typename Eigen::internal::traits<Base>::Scalar Scalar;
    typedef typename Eigen::internal::traits<Base>::Index Index;
    typedef Eigen::Map<Base> Map;
    typedef Eigen::Map<const Base> ConstMap;

    Matrix_() { this->setZero(); }
    Matrix_(Scalar value) { this->setConstant(value); }
    template <typename OtherDerived> Matrix_(const Eigen::MatrixBase<OtherDerived>& other) : Base(other) {}

    EIGEN_INHERIT_ASSIGNMENT_EQUAL_OPERATOR(Derived)

  protected:
    explicit Matrix_(Index rows, Index cols) : Base(rows, cols) {}
  };
  typedef Matrix_<3,3> Matrix3;

//  namespace internal {
//    template <int RowsCols>
//    class SymmetricMatrixStorage {
//    public:
//      EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF(RowsCols != Dynamic)
//      typedef typename Matrix_<RowsCols,RowsCols>::Base Base;
//      typedef typename Eigen::internal::traits<Base>::Scalar Scalar;
//      typedef typename Eigen::internal::traits<Base>::Index Index;

//      SymmetricMatrixStorage() {}
//      template <typename OtherDerived> SymmetricMatrixStorage(const Eigen::MatrixBase<OtherDerived>& other) : storage_(other) {}

//    protected:
//      explicit SymmetricMatrixStorage(Index dim) : storage_(dim) {}
//      Matrix_<RowsCols,RowsCols> storage_;
//    };
//  }

//  template <int RowsCols>
//  class SymmetricMatrix_ : public internal::SymmetricMatrixStorage<RowsCols>, public Eigen::SelfAdjointView<typename internal::SymmetricMatrixStorage<RowsCols>::Base,Upper> {
//  public:
//    typedef internal::SymmetricMatrixStorage<RowsCols> Storage;
//    typedef typename Storage::Base Base;
//    typedef Eigen::SelfAdjointView<typename Storage::Base,Upper> SelfAdjointViewType;
////    typedef SelfAdjointViewType Base;
//    typedef typename Eigen::internal::traits<Base>::Scalar Scalar;
//    typedef typename Eigen::internal::traits<Base>::Index Index;
//    using Storage::storage_;

//    // Constructors
//    SymmetricMatrix_() : SelfAdjointViewType(storage_) {}
//    SymmetricMatrix_(Scalar value) : SelfAdjointViewType(storage_) { this->setConstant(value); }
//    template <int OtherRowsCols> SymmetricMatrix_(const SymmetricMatrix_<OtherRowsCols>& other) : Storage(other), SelfAdjointViewType(storage_) {}
//    template <typename OtherDerived> SymmetricMatrix_(const Eigen::MatrixBase<OtherDerived>& other) : Storage(other), SelfAdjointViewType(storage_) {}

//    SymmetricMatrix_<RowsCols>& setZero() {
//      storage_.setZero();
//      return *this;
//    }

//    SymmetricMatrix_<RowsCols>& setConstant(Scalar scalar) {
//      storage_.setConstant(scalar);
//      return *this;
//    }

//    template<int BlockRows, int BlockCols>
//    Block<Base, BlockRows, BlockCols> block(Index startRow, Index startCol) {
//      eigen_assert(startRow <= startCol);
//      return storage_.block<BlockRows,BlockCols>(startRow, startCol);
//    }

//    template<int BlockRows, int BlockCols>
//    const Block<const Base, BlockRows, BlockCols> block(Index startRow, Index startCol) const {
//      eigen_assert(startRow <= startCol);
//      return storage_.block<BlockRows,BlockCols>(startRow, startCol);
//    }

//    void conservativeResize(Index rows, Index cols) {
//      storage_.conservativeResize(rows, cols);
//    }

//    void conservativeResize(Index size) {
//      storage_.conservativeResize(size, size);
//    }

//    using SelfAdjointViewType::operator*;
//    SymmetricMatrix_<RowsCols> operator*(const Scalar& scalar) {
//      return SymmetricMatrix_<RowsCols>(storage_ * scalar);
//    }

//    template <int OtherRowsCols> SymmetricMatrix_<RowsCols>& operator=(const SymmetricMatrix_<OtherRowsCols>& other) {
//      storage_ = other;
//      return *this;
//    }

//    template <typename OtherDerived> SymmetricMatrix_<RowsCols>& operator=(const Eigen::MatrixBase<OtherDerived>& other) {
//      storage_ = other;
//      return *this;
//    }

//    template <typename Other> SymmetricMatrix_<RowsCols>& operator+(const Other& other) {
//      storage_ += other;
//      return *this;
//    }

//    template <typename Other> SymmetricMatrix_<RowsCols>& operator-(const Other& other) {
//      storage_ -= other;
//      return *this;
//    }

//    template <typename OtherDerived> SymmetricMatrix_<Derived::RowsAtCompileTime> quadratic(const Eigen::MatrixBase<OtherDerived>& other) {
//      return other * (*this) * other.transpose();
//    }

//    SymmetricMatrix_<RowsCols> inverse() const {
//      return SymmetricMatrix_<RowsCols>(this->toDenseMatrix().inverse());
//    }

//  protected:
//    explicit SymmetricMatrix_(Index dim) : Storage(dim), SelfAdjointViewType(storage_) {}
//  };

//  template <int RowsCols>
//  std::ostream& operator<<(std::ostream& os, const SymmetricMatrix_<RowsCols>& matrix) {
//    return os << matrix.storage_;
//  }

  template <int RowsCols>
  class SymmetricMatrix_ : public Matrix_<RowsCols,RowsCols> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF(RowsCols != Dynamic) // is this really required if Eigen::Matrix is the base class?

    typedef SymmetricMatrix_<RowsCols> Derived;
    typedef Matrix_<RowsCols,RowsCols> Storage;
    typedef typename Storage::Base Base;
    typedef Eigen::SelfAdjointView<typename Storage::Base,Upper> SelfAdjointView;
    typedef typename Eigen::internal::traits<Base>::Scalar Scalar;
    typedef typename Eigen::internal::traits<Base>::Index Index;
    typedef Eigen::Map<Base> Map;
    typedef Eigen::Map<const Base> ConstMap;

    // Constructors
    SymmetricMatrix_() {}
    SymmetricMatrix_(Scalar value) { this->setConstant(value); }
    template <typename OtherDerived> SymmetricMatrix_(const Eigen::MatrixBase<OtherDerived>& other) : Storage(other) { symmetric(); }

    template <typename OtherDerived> Derived& operator=(const Eigen::MatrixBase<OtherDerived>& other) {
      this->Base::operator=(other);
      return symmetric();
    }

    Derived& symmetric() {
#if defined(ASSERT_SYMMETRIC_MATRIX_TO_BE_SYMMETRIC)
      eigen_assert(this->isApprox(this->transpose(), ASSERT_SYMMETRIC_MATRIX_TO_BE_SYMMETRIC_PRECISION));
#endif
#if defined(FORCE_SYMMETRIC_MATRIX_TO_BE_SYMMETRIC)
      this->template triangularView<Eigen::Lower>() = this->transpose();
#endif
      return *this;
    }

//    template <typename OtherDerived> SymmetricMatrix_<Derived::RowsAtCompileTime> quadratic(const Eigen::MatrixBase<OtherDerived>& other) {
//      return other * this->template selfadjointView<Upper>() * other.transpose();
//    }

    Derived inverse() const {
//      return this->template selfadjointView<Upper>().inverse();
      return this->Storage::inverse().eval();
    }

  protected:
    explicit SymmetricMatrix_(Index dim) : Storage(dim, dim) {}
  };
  typedef SymmetricMatrix_<3> SymmetricMatrix3;

  class SymmetricMatrix : public SymmetricMatrix_<Dynamic> {
  public:
    typedef SymmetricMatrix Derived;
    typedef SymmetricMatrix_<Dynamic> Storage;
    typedef typename Storage::Base Base;
    typedef Eigen::SelfAdjointView<typename Storage::Base,Upper> SelfAdjointView;
    typedef typename Eigen::internal::traits<Base>::Scalar Scalar;
    typedef typename Eigen::internal::traits<Base>::Index Index;
    typedef Eigen::Map<Base> Map;
    typedef Eigen::Map<const Base> ConstMap;

    // Constructors
    SymmetricMatrix() : SymmetricMatrix_<Dynamic>() {}
    SymmetricMatrix(Index dim) : SymmetricMatrix_<Dynamic>(dim) { this->setZero(); }
    SymmetricMatrix(Index dim, Scalar value) : SymmetricMatrix_<Dynamic>(dim) { this->setConstant(value); }
    template <int OtherRowsCols> SymmetricMatrix(const SymmetricMatrix_<OtherRowsCols>& other) : SymmetricMatrix_<Dynamic>(other) {}
    template <typename OtherDerived> SymmetricMatrix(const Eigen::MatrixBase<OtherDerived>& other) : SymmetricMatrix_<Dynamic>(other) {}

    template <typename OtherDerived> Derived& operator=(const Eigen::MatrixBase<OtherDerived>& other) {
      this->Base::operator=(other);
      return symmetric();
    }

    Derived& symmetric() {
      Storage::symmetric();
      return *this;
    }

    void resize(Index size) {
      Base::resize(size, size);
    }

    void conservativeResize(Index size) {
      Base::conservativeResize(size, size);
    }
  };

} // namespace hector_pose_estimation

#endif // HECTOR_POSE_ESTIMATION_MATRIX_H
