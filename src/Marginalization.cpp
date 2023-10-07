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

#include "Marginalization.h"

namespace libRSF
{

  void Marginalize(const Vector &Residual, const Matrix &Jacobian,
                   Vector &ResidualMarg, Matrix &JacobianMarg,
                   const int SizeMarginal, const double HessianInflation)
  {
    /**
    Construct a linear system:
    H * x = b

    Which is defined as:
    J^T*J * x = -J^T * r
    */

    /** H = J^T * J */
    Matrix Hessian = Jacobian.transpose() * Jacobian;
    Hessian = Matrix((Hessian.array().abs() > 1e-8).select(Hessian.array(), 0));/**< remove small non-zero entries for stability */

    /** b = -J^T * r */
    const Vector B = -Jacobian.transpose() * Residual;

    /** calculate size of the linear system */
    const int SizeTotal = static_cast<int>(Hessian.cols());
    const int SizeRemain = SizeTotal - SizeMarginal;

    /** select sub matrices  */
    /**
    Hessian:
    H = |H_MM H_MR|
        |H_RM H_RR|

    Residual vector:
    b = |b_M b_R|'
    */
    const Matrix HessMM = Hessian.block(0, 0, SizeMarginal, SizeMarginal);
    const Matrix HessRR = Hessian.block(SizeMarginal, SizeMarginal, SizeRemain, SizeRemain);
    const Matrix HessMR = Hessian.block(0, SizeMarginal, SizeMarginal, SizeRemain);
    const Matrix HessRM = HessMR.transpose();

    const Vector BM = B.head(SizeMarginal);
    const Vector BR = B.tail(SizeRemain);

    /** compute reduced linear system */
    /**
    New Hessian:
    H^* = H_RR - H_RM * H_MM^-1 x H_MR

    New residual vector:
    b^* = b_R - H_RM * H_MM^-1 * b_M
    */

    /** at first, invert the marginalized Hessian part */
    Eigen::CompleteOrthogonalDecomposition<Matrix> CODHess(HessMM);
    const Matrix HessMMInv = CODHess.pseudoInverse(); /**< pseudo-inverse for rank deficient matrices*/

    /** than compute the new system */
    Matrix HessRRStar = HessRR - HessRM * HessMMInv * HessMR;
    const Vector BRStar = BR - HessRM * HessMMInv * BM;

    /** add a small uncertainty to prevent accumulation of error (I recommend a value of 1.01)*/
    if (HessianInflation != 1.0)
    {
      HessRRStar /= HessianInflation;
    }

    /** convert to linear constraint*/
    ResidualMarg.resize(SizeRemain);
    JacobianMarg.resize(SizeRemain, SizeRemain);
    Matrix JacobianMargInv(SizeRemain, SizeRemain);

    /**
    New linear factor:
    |J^* x + r^*|^2

    with:
    J^* = sqrt(H^*)
    r^* = -sqrt(H^*)^-T * b^*
    */
    RobustSqrtAndInvSqrt(HessRRStar, JacobianMarg, JacobianMargInv);
    ResidualMarg = -JacobianMargInv.transpose() * BRStar;
  }

}
