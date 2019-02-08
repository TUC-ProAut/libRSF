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
 * @file MaxMixture.h
 * @author Tim Pfeifer
 * @date 18.09.2018
 * @brief Max-Mixture error model based on the work of Olson.
 * @copyright GNU Public License.
 *
 */

#ifndef MAXMIXTURE_H
#define MAXMIXTURE_H

#include <ceres/ceres.h>
#include "ErrorModel.h"
#include "GaussianMixture.h"

namespace libRSF
{
  template <int Dimension, typename MixtureType>
  class MaxMixture : public ErrorModel <Dimension, Dimension+1>
  {
    public:

      MaxMixture()
      {
        _Normalization = -std::numeric_limits<double>::infinity();
      }

      explicit MaxMixture(MixtureType &Mixture)
      {
        addMixture(Mixture);
      }

      void clear()
      {
        _Normalization = -std::numeric_limits<double>::infinity();
        _Mixture.clear();
      }

      void addMixture(MixtureType &Mixture)
      {
        _Mixture = Mixture;

        _Normalization = _Mixture.getMaximumOfComponent(1);

        size_t NumberOfComponents = _Mixture.getNumberOfComponents();
        for(int nComponent = 2; nComponent <= NumberOfComponents; ++nComponent)
        {
          _Normalization = std::max(_Normalization, _Mixture.getMaximumOfComponent(nComponent));
        }
      }

      template <typename T>
      bool Evaluate(T* Error) const
      {
        if(this->_Enable)
        {
          T Loglike = T(NAN);

          /** map the error pointer to a matrix */
          Eigen::Map<Eigen::Matrix<T, Dimension + 1, 1> > ErrorMap(Error);
          Eigen::Matrix<T, Dimension + 1, 1> ErrorShadow, ErrorShadowBest;

          size_t NumberOfComponents = _Mixture.getNumberOfComponents();

          /** calculate Log-Likelihood for each Gaussian component */
          for(int nComponent = 1; nComponent <= NumberOfComponents; ++nComponent)
          {
            ErrorShadow << _Mixture.getExponentialPartOfComponent(nComponent, Error),
                           sqrt(ceres::fmax(T(-log(_Mixture.getLinearPartOfComponent(nComponent, Error) / _Normalization)), T(1e-10)));/** fmax() is required to handle numeric tolerances */

            /** keep only the most likely component */
            if(ErrorShadow.squaredNorm() < Loglike || ceres::IsNaN(Loglike))
            {
              Loglike = ErrorShadow.squaredNorm();
              ErrorShadowBest = ErrorShadow;
            }
          }

          ErrorMap = ErrorShadowBest;
        }
        else
        {
          /** write something to the second component, if the error model is disabled */
          Error[Dimension] = T(0.0);
        }

        return true;
      };

    private:
      MixtureType _Mixture;
      double _Normalization;

  };

  typedef MaxMixture<1, GaussianMixture<1>> MaxMix1;
}

#endif // MAXMIXTURE_H
