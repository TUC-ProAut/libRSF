/***************************************************************************
 * libRSF - A Robust Sensor Fusion Library
 *
 * Copyright (C) 2018 Chair of Automation Technology / TU Chemnitz
 * For more information see https://www.tu-chemnitz.de/etit/proaut/self-tuning
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
    const T MaxExp = Exponents.maxCoeff();
    const T Sum = ((Exponents.array() - MaxExp).exp() * Scaling.array()).sum();

    /** catch cases where only one component dominates (Sum goes to 0 --> log(0) = -inf) */
    if (isinf(log(Sum)))
    {
      return MaxExp;
    }
    else
    {
      return (log(Sum) + MaxExp);
    }
  }

  template <typename T, int Dim>
  MatrixT<T, Dim+1, 1> VectorizedLogSumExp(const MatrixT<T, Dynamic, Dim> &LinearExponents,
                                           const MatrixT<T, Dynamic, 1> &Scaling)
  {
    /** [linear term Dim x 1; nonlinear term 1 x 1] */
    MatrixT<T, Dim+1, 1> LSE;

    /** compute squared errors */
    const MatrixT<T, Dynamic, 1> Exponents = LinearExponents.rowwise().squaredNorm();
    const MatrixT<T, Dynamic, 1> SquaredError = 0.5*Exponents - Scaling.array().log().matrix();

    /** find max */
    Index MaxIndex;
    SquaredError.minCoeff(&MaxIndex);

    /** pass max directly */
    LSE.template head<Dim>() = LinearExponents.row(MaxIndex);

    /** apply log sum exp for the other components */
    LSE(Dim) = ScaledLogSumExp<T>(-0.5 * (Exponents.array() - Exponents(MaxIndex) + T(1e-10)), Scaling.array());

    return LSE;
  }

}

#endif // NUMERICALROBUST_H
