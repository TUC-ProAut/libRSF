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
 * @file Statistics.h
 * @author Tim Pfeifer
 * @date 30.08.2020
 * @brief Collection of statistical tools.
 * @copyright GNU Public License.
 *
 */

#ifndef STATISTICS_H
#define STATISTICS_H

#include "VectorMath.h"
#include "StateDataSet.h"
#include "SensorDataSet.h"

#include <limits>

namespace libRSF
{
  /** calculate covariance from data */
  template <int Dim>
  MatrixStatic<Dim, Dim> EstimateSampleCovariance(const MatrixStatic<Dim, Dynamic> &Data)
  {
    /** estimate mean */
    VectorStatic<Dim> Mean = Data.rowwise().mean();

    /** estimate cov */
    MatrixStatic<Dim, Dim> Covariance = MatrixStatic<Dim, Dim>::Zero();
    for (int n = 0; n < Data.cols(); ++n)
    {
      VectorStatic<Dim> Diff = Data.col(n) - Mean;
      Covariance += Diff * Diff.transpose();
    }

    /** return sample covariance */
    return Covariance / (Data.cols() - 1);
  }

  /** median calculation */
  double Median(const std::vector<double> &V);
  double Median(const Vector &V);

  /** median absolute deviation (robust variance estimator) */
  double MAD(const Vector &V);
  double EstimateMADCovariance(const Vector &V);

  /** RMSE of a vector */
  double RMSE(const Vector &V);
}

#endif // STATISTICS_H
