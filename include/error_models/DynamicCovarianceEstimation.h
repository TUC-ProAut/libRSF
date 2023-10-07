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
 * @file DynamicCovarianceEstimation.h
 * @author Tim Pfeifer
 * @date 18.09.2018
 * @brief Error functions for the DCE algorithm.
 * @copyright GNU Public License.
 *
 */

#ifndef DYNAMICCOVARIANCEESTIMATION_H
#define DYNAMICCOVARIANCEESTIMATION_H

#include "ErrorModel.h"
#include "../VectorMath.h"

namespace libRSF
{
  template <int Dim>
  class DynamicCovarianceEstimation : public ErrorModel < Dim, Dim + 1, Dim>
  {
    public:

      DynamicCovarianceEstimation() = default;

      explicit DynamicCovarianceEstimation(const VectorStatic<Dim> &CovMinDiagonal) : LogSqrtDetCovMin_(log(sqrt(CovMinDiagonal.prod())))
      {}

      template <typename T>
      bool weight(const VectorT<T, Dim> &RawError, const T* const Covariance, T* Error) const
      {
        if (this->Enable_)
        {
          /** map error to matrix */
          VectorRef <T, Dim+1> ErrorMap(Error);

          /** map covariance matrix diagonal to matrix */
          VectorRefConst <T, Dim> CovMap(Covariance);

          /** apply sqrt info */
          ErrorMap.template head<Dim>() = RawError.template head<Dim>().array() / CovMap.array().sqrt();

          /** add nonlinear prior */
          ErrorMap(Dim) = sqrt(log(sqrt(CovMap.prod())) - LogSqrtDetCovMin_ + 1e-4) * sqrt(2.0);
        }
        else
        {
          /** pass raw error trough */
          VectorRef<T, Dim> ErrorMap(Error);
          ErrorMap = RawError;

          /** set unused dimension to 0 */
          Error[Dim] = T(0.0);
        }

        return true;
      }

    private:
      double LogSqrtDetCovMin_ = 1.0;
  };
}

#endif // DYNAMICCOVARIANCEESTIMATION_H
