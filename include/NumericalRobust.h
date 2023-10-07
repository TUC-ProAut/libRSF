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
 * @file NumericalRobust.h
 * @author Tim Pfeifer
 * @date 22.08.2017
 * @brief File containing numerical robust functions that might be helpful for some calculation in factor graphs. (Especially Sum-Mixture!)
 * @copyright GNU Public License.
 *
 */

#ifndef NUMERICALROBUST_H
#define NUMERICALROBUST_H

#include "VectorMath.h"
#include <iostream>

namespace libRSF
{
  /** \brief An implementation of the numerically robust Log-Sum-Exp algorithm.
   * See: https://en.wikipedia.org/wiki/LogSumExp
   *
   * \param Exponents
   * \param Scaling
   * \return log(Scaling *sum(exp(Exponents)))
   *
   */
  template <typename T>
  T ScaledLogSumExp(const MatrixT<T, Dynamic, 1> &Exponents,
                    const MatrixT<T, Dynamic, 1> &Scaling)
  {
    /** find biggest (relevant) exponent */
    const T MaxExp = (Scaling.array() != T(0.0)).select(Exponents, Exponents.minCoeff()).maxCoeff();

    /** sum only terms with non-zero scaling (to prevent NaNs) */
    const T Sum = (Scaling.array() != T(0.0)).select((Exponents.array() - MaxExp).exp() * Scaling.array(), T(0.0)).sum();

    return (log(Sum) + MaxExp);
  }

  template <typename T, int Dim>
  MatrixT<T, Dim+1, 1> VectorizedLogSumExp(const MatrixT<T, Dynamic, Dim> &LinearExponents,
                                           const MatrixT<T, Dynamic, 1> &Scaling)
  {
    /** [linear term Dim x 1; nonlinear term 1 x 1] */
    MatrixT<T, Dim+1, 1> LSE;

    /** compute squared errors */
    const MatrixT<T, Dynamic, 1> Exponents = -0.5 * LinearExponents.rowwise().squaredNorm();
    const MatrixT<T, Dynamic, 1> SquaredError = Exponents + Scaling.array().log().matrix();

    /** find maximum probability (minimum of squared error) */
    Index MaxIndex;
    SquaredError.maxCoeff(&MaxIndex);

    /** pass max directly */
    LSE.template head<Dim>() = LinearExponents.row(MaxIndex);

    /** apply log sum exp for the other components */
    LSE(Dim) = ScaledLogSumExp<T>(Exponents.array() - Exponents(MaxIndex), Scaling.array());

    return LSE;
  }

}

#endif // NUMERICALROBUST_H
