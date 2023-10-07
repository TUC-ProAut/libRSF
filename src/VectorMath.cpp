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

#include "VectorMath.h"

namespace libRSF
{
  void RobustSqrtAndInvSqrt(const Matrix &Mat, Matrix &MatSqrt, Matrix & MatSqrtInv)
  {
    /** compute SVD */
    Eigen::SelfAdjointEigenSolver<Matrix> SAES(Mat);

    /** compute tolerance (idea from OKVIS) */
    double Tolerance = std::numeric_limits<double>::epsilon() * static_cast<double>(Mat.cols()) * SAES.eigenvalues().array().maxCoeff();

    /** set small eigen values to zero */
    Vector EigVal = Vector((SAES.eigenvalues().array() > Tolerance).select(SAES.eigenvalues().array(), 0));
    Vector EigValInv = Vector((SAES.eigenvalues().array() > Tolerance).select(SAES.eigenvalues().array().inverse(), 0));

    /** use modified eigen values to compute sqrt */
    MatSqrt = SAES.eigenvectors() * EigVal.cwiseSqrt().asDiagonal() * SAES.eigenvectors().transpose();
    MatSqrtInv = SAES.eigenvectors() * EigValInv.cwiseSqrt().asDiagonal() * SAES.eigenvectors().transpose();
  }

  void RemoveColumn(Matrix& Matrix, int ColToRemove)
  {
      int numRows = Matrix.rows();
      int numCols = Matrix.cols()-1;

      if( ColToRemove < numCols ) 
      {
          Matrix.block(0,ColToRemove,numRows,numCols-ColToRemove) = Matrix.rightCols(numCols-ColToRemove);
      }

      Matrix.conservativeResize(numRows,numCols);
  }

  void CRSToMatrix(const ceres::CRSMatrix &CRSMat, Matrix &Mat)
  {
    /** construct empty matrix */
    Mat.resize(CRSMat.num_rows, CRSMat.num_cols);
    Mat.setZero();

    /** fill with sparse matrix */
    for (int Row = 0; Row < CRSMat.num_rows; Row++)
    {
      int Start = CRSMat.rows[Row];
      int End = CRSMat.rows[Row + 1];

      for (int n = Start; n < End; n++)
      {
        int Col = CRSMat.cols[n];

        Mat(Row, Col) = CRSMat.values[n];
      }
    }
  }
}
