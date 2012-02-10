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

#include <bfl/wrappers/matrix/matrix_wrapper.h>
#include <stdexcept>

namespace hector_pose_estimation {

  using namespace MatrixWrapper;

  template <unsigned int dim>
  class ColumnVector_ : public ColumnVector {
  public:
    ColumnVector_() : ColumnVector(dim) {}
    ColumnVector_(double value) : ColumnVector(dim, value) {}
    ColumnVector_(const ColumnVector_<dim>& other) : ColumnVector(other) {}
    ColumnVector_(const ColumnVector& other) : ColumnVector(other)
    {
      if (other.rows() != dim) {
        std::cerr << (int)dim << " != " << (int)other.rows() << std::endl;
        throw std::runtime_error("Illegal vector assignment");
      }
    }
//    ColumnVector_<dim>& operator=(const ColumnVector& other) {
//      if (other.rows() != dim) throw std::runtime_error("Illegal vector assignment");
//      static_cast<ColumnVector &>(*this) = other;
//      return *this;
//    }
  };

  template <unsigned int dim>
  class RowVector_ : public RowVector {
  public:
    RowVector_() : RowVector(dim) {}
    RowVector_(double value) : RowVector(dim, value) {}
    RowVector_(const RowVector_<dim>& other) : RowVector(other) {}
    RowVector_(const RowVector& other) : RowVector(other) {
      if (other.columns() != dim) {
        std::cerr << (int)dim << " != " << (int)other.columns() << std::endl;
        throw std::runtime_error("Illegal vector assignment");
      }
    }
//    RowVector_<dim>& operator=(const RowVector& other)  {
//      if (other.columns() != dim) throw std::runtime_error("Illegal vector assignment");
//      static_cast<RowVector &>(*this) = other;
//      return *this;
//    }
  };

  template <unsigned int m, unsigned int n>
  class Matrix_ : public Matrix {
  public:
    Matrix_() : Matrix(m, n) {}
    Matrix_(double value) : Matrix(m, RowVector_<n>(value)) {}
    Matrix_(const RowVector_<n> &row_value) : Matrix(m, row_value) {}
    Matrix_(const Matrix_<m,n>& other) : Matrix(other) {}
    Matrix_(const Matrix& other) : Matrix(other) {
      if (other.rows() != m || other.columns() != n) {
        std::cerr << (int)m << "x" << (int)n << " != " << (int)other.rows() << "x" << (int)other.columns() << std::endl;
        throw std::runtime_error("Illegal matrix assignment");
      }
    }
//    Matrix_<m,n>& operator=(const Matrix& other) {
//      if (other.rows() != m || other.columns() != n) throw std::runtime_error("Illegal matrix assignment");
//      static_cast<Matrix &>(*this) = other;
//      return *this;
//    }
  };

  template <unsigned int dim>
  class SymmetricMatrix_ : public SymmetricMatrix {
  public:
    SymmetricMatrix_() : SymmetricMatrix(dim) {}
    SymmetricMatrix_(double value) : SymmetricMatrix(dim, RowVector_<dim>(value)) {}
    SymmetricMatrix_(const SymmetricMatrix_<dim>& other) : SymmetricMatrix(other) {}
    SymmetricMatrix_(const SymmetricMatrix& other) : SymmetricMatrix(other) {
      if (other.rows() != dim || other.columns() != dim) {
         std::cerr << (int)dim << "x" << (int)dim << " != " << (int)other.rows() << "x" << (int)other.columns() << std::endl;
         throw std::runtime_error("Illegal matrix assignment");
      }
    }
//    SymmetricMatrix_<dim>& operator=(const SymmetricMatrix& other) {
//      if (other.rows() != dim || other.columns() != dim) throw std::runtime_error("Illegal matrix assignment");
//      static_cast<SymmetricMatrix &>(*this) = other;
//      return *this;
//    }
  };

} // namespace hector_pose_estimation

#endif // HECTOR_POSE_ESTIMATION_MATRIX_H
