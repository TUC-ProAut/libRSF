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
 * @file SumMixture.h
 * @author Tim Pfeifer
 * @date 18.09.2018
 * @brief Sum-Mixture error model inspired from the work of Rosen.
 * @copyright GNU Public License.
 *
 */

#ifndef SUMMIXTURE_H
#define SUMMIXTURE_H

#include <ceres/ceres.h>
#include "ErrorModel.h"
#include "GaussianMixture.h"
#include "NumericalRobust.h"

namespace libRSF
{
  template <int Dimension, typename MixtureType>
  class SumMixture : public ErrorModel <Dimension, 1>
  {
  public:

      SumMixture()
      {
        _Normalization = 0;
      };

     explicit SumMixture(MixtureType &Mixture)
     {
       addMixture(Mixture);
     };

      void addMixture (MixtureType &Mixture)
      {
        _Mixture = Mixture;

        _Normalization = 0;

        size_t NumberOfComponents = _Mixture.getNumberOfComponents();
        for (int nComponent = 1; nComponent <= NumberOfComponents; ++nComponent)
        {
          _Normalization += _Mixture.getMaximumOfComponent(nComponent);
        }
      }

      void clear()
      {
        _Normalization = 0;
        _Mixture.clear();
      }

      template <typename T>
      bool Evaluate(T* Error) const
      {
        if (this->_Enable)
        {
          Eigen::Matrix<T, Eigen::Dynamic, 1> Scalings;
          Eigen::Matrix<T, Eigen::Dynamic, 1> Exponents;
          T SquaredError;

          size_t NumberOfComponents = _Mixture.getNumberOfComponents();

          Scalings.resize(NumberOfComponents,1);
          Exponents.resize(NumberOfComponents,1);

          /** calculate all exponents and scalings */
          for(int nComponent = 1; nComponent <= NumberOfComponents; ++nComponent)
          {
            Exponents(nComponent-1,0) = - 0.5 * _Mixture.getExponentialPartOfComponent(nComponent, Error).squaredNorm();
            Scalings(nComponent-1,0) = T(_Mixture.getLinearPartOfComponent(nComponent, Error)/_Normalization);
          }

          /** combine them numerically robust */
          SquaredError = - ScaledLogSumExp(Exponents.data(), Scalings.data(), NumberOfComponents);
          *Error = ceres::sqrt(SquaredError * T(2.0));
        }
        return true;
      };

  private:
    MixtureType _Mixture;
    double _Normalization;
  };

  typedef SumMixture<1, GaussianMixture<1>> SumMix1;
}

#endif // SUMMIXTURE_H
