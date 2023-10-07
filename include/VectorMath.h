/***************************************************************************
 * libRSF - A Robust Sensor Fusion Library
 *
 * Copyright (C) 2023 Chair of Automation Technology / TU Chemnitz
 * For more information see https://www.tu-chemnitz.de/etit/proaut/libRSF
 *
 * libRSF is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * libRSF is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with libRSF.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Tim Pfeifer (tim.pfeifer@etit.tu-chemnitz.de)
 ***************************************************************************/

/**
 * @file VectorMath.h
 * @author Tim Pfeifer
 * @date 07.03.2019
 * @brief Derived vector type and some simple helper function.
 * @copyright GNU Public License.
 *
 */

#ifndef VECTORMATH_H
#define VECTORMATH_H

#include "VectorTypes.h"

#include <Eigen/Eigenvalues>
#include <ceres/ceres.h>

#include <algorithm>

namespace libRSF
{
  /** math with vector/matrix class */
  template <int Dim, typename T>
  MatrixT<T, Dim, Dim> SquareRoot(MatrixT<T, Dim, Dim> A)
  {
    /** compute SVD */
    Eigen::SelfAdjointEigenSolver<MatrixT<T, Dim, Dim>> SAES(A);

    /** compute tolerance (idea from OKVIS) */
    T Tolerance = std::numeric_limits<T>::epsilon() * A.cols() * SAES.eigenvalues().array().maxCoeff();

    /** set small eigen values to zero */
    VectorT<T, Dim>  EigVal = VectorT<T, Dim>((SAES.eigenvalues().array() > Tolerance).select(SAES.eigenvalues().array(), 0));

    /** use modified eigen values to compute sqrt */
    return SAES.eigenvectors() * EigVal.cwiseSqrt().asDiagonal() * SAES.eigenvectors().transpose();
  }

  template <int Dim, typename T>
  MatrixT<T, Dim, Dim> InverseSquareRoot(MatrixT<T, Dim, Dim> A)
  {
    /** compute SVD */
    Eigen::SelfAdjointEigenSolver<MatrixT<T, Dim, Dim>> SAES(A);

    /** compute tolerance (idea from OKVIS) */
    T Tolerance = std::numeric_limits<T>::epsilon() * A.cols() * SAES.eigenvalues().array().maxCoeff();

    /** set small eigen values to zero */
    VectorT<T, Dim> EigValInv = Vector((SAES.eigenvalues().array() > Tolerance).select(SAES.eigenvalues().array().inverse(), 0));

    /** use modified eigen values to compute inverse sqrt */
    return SAES.eigenvectors() * EigValInv.cwiseSqrt().asDiagonal() * SAES.eigenvectors().transpose();
  }

  template <int Dim, typename T>
  MatrixT<T, Dim, Dim> Inverse(MatrixT<T, Dim, Dim> A)
  {
    /** compute SVD */
    Eigen::SelfAdjointEigenSolver<MatrixT<T, Dim, Dim>> SAES(A);

    /** compute tolerance (idea from OKVIS) */
    T Tolerance = std::numeric_limits<T>::epsilon() * A.cols() * SAES.eigenvalues().array().maxCoeff();

    /** set small eigen values to zero */
    VectorT<T, Dim> EigValInv = Vector((SAES.eigenvalues().array() > Tolerance).select(SAES.eigenvalues().array().inverse(), 0));

    /** use modified eigen values to compute sqrt */
    return SAES.eigenvectors() * EigValInv.asDiagonal() * SAES.eigenvectors().transpose();
  }

  void RobustSqrtAndInvSqrt(const Matrix &Mat, Matrix &MatSqrt, Matrix &MatSqrtInv);

  /** vector math with raw pointers (compatible to ceres jet type) */
  template <int Dim, typename T1, typename T2>
  void VectorDifference(const T1* const Vector1, const T2* const Vector2,  T1* Difference)
  {
    VectorRefConst<T1, Dim> V1(Vector1);
    VectorRefConst<T2, Dim> V2(Vector2);
    VectorRef<T1, Dim>      Diff(Difference);

    Diff = V1 - V2;
  }

  template <int Dim, typename T>
  T VectorLength(const T* const Vector)
  {
    VectorRefConst<T, Dim> V(Vector);
    T SquaredNorm = V.squaredNorm();

    /** for stability of the derivation */
    if(SquaredNorm < T(1e-10)) 
    {
      SquaredNorm += 1e-10;
    }

    return sqrt(SquaredNorm);
  }

  template <int Dim, typename T1, typename T2>
  T1 VectorDistance(const T1* const Vector1, const T2* const Vector2)
  {
    T1 Difference[Dim];
    VectorDifference<Dim, T1, T2>(Vector1, Vector2, Difference);
    return VectorLength<Dim, T1>(Difference);
  }

  /** test matrix against semi-definitiveness */
  template <int Dim>
  bool IsPositiveSemidefinite(MatrixStatic<Dim, Dim> Mat)
  {
    Eigen::LDLT<MatrixStatic<Dim, Dim>> CholeskyOfSqrtInfo(Mat);
    return (CholeskyOfSqrtInfo.info() != Eigen::NumericalIssue);
  }

  void RemoveColumn (Matrix& Matrix, int ColToRemove);

  void CRSToMatrix(const ceres::CRSMatrix &CRSMat, Matrix &Mat);
}

#endif // VECTORMATH_H
