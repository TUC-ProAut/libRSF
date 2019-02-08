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
 * @file Gaussian.h
 * @author Tim Pfeifer
 * @date 18.09.2018
 * @brief Header for probabilistic error models that are based on a single zero-mean Gaussian.
 * @copyright GNU Public License.
 *
 */

#ifndef GAUSSIAN_H
#define GAUSSIAN_H

#include <Eigen/Dense>
#include "ErrorModel.h"

namespace libRSF
{
  template <int Dimension>
  class GaussianDiagonal : public ErrorModel <Dimension, Dimension>
  {
    public:

      GaussianDiagonal(){};

      explicit GaussianDiagonal(Eigen::Matrix<double, Dimension, 1> StdDev)
      {
        setStdDev(StdDev);
      };

      void setStdDev(Eigen::Matrix<double, Dimension, 1> StdDev)
      {
        _SqrtInformation = StdDev.cwiseInverse().asDiagonal();
      }

      Eigen::Matrix<double, Dimension, 1> getStdDev()
      {

      }

      template <typename T>
      bool Evaluate(T* Error) const
      {
        Eigen::Map<Eigen::Matrix<T, Dimension, 1>> ErrorMap(Error);

        ErrorMap = _SqrtInformation.template cast<T>() * ErrorMap;

        return true;
      };

    private:
      Eigen::Matrix<double, Dimension, Dimension> _SqrtInformation;
  };

  template <int Dimension>
  class GaussianFull : public ErrorModel <Dimension, Dimension>
  {
    public:

      GaussianFull()
      {};

      template <typename T>
      bool Evaluate(T* Error) const
      {
        Eigen::Map<Eigen::Matrix<T, Dimension, 1>> ErrorMap(Error);

        ErrorMap = _SqrtInformation.template cast<T>() * ErrorMap;

        return true;
      };

    private:

      Eigen::Matrix<double, Dimension, Dimension> _SqrtInformation;
  };
}

#endif // GAUSSIAN_H
