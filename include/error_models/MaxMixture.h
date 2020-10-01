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

#include "ErrorModel.h"
#include "GaussianMixture.h"

#include <ceres/ceres.h>

namespace libRSF
{
  /** \brief The robust Max-Mixture error model
   * Based on:
   * E. Olson and P. Agarwal
   * “Inference on networks of mixtures for robust robot mapping”
   * Proc. of Robotics: Science and Systems (RSS), Sydney, 2012.
   * DOI: 10.15607/RSS.2012.VIII.040
   *
   * \param Mixture Underlying mixture distribution
   *
   */
  template <int Dim, typename MixtureType>
  class MaxMixture : public ErrorModel <Dim, Dim+1>
  {
    public:

      MaxMixture()
      {
        _Normalization = std::numeric_limits<double>::lowest();
      }

      virtual ~MaxMixture() = default;

      explicit MaxMixture(const MixtureType &Mixture)
      {
        this->addMixture(Mixture);
      }

      void clear()
      {
        _Normalization = std::numeric_limits<double>::lowest();
        _Mixture.clear();
      }

      template <typename T>
      bool weight(const VectorT<T, Dim> &RawError, T* Error) const
      {
        if(this->_Enable)
        {
          /** map the error pointer to a matrix */
          VectorRef<T, Dim+1> ErrorMap(Error);

          T Loglike = T(NAN);

          MatrixT<T, Dim + 1, 1> ErrorShadow, ErrorShadowBest;

          size_t NumberOfComponents = _Mixture.getNumberOfComponents();

          /** calculate Log-Likelihood for each Gaussian component */
          for(int nComponent = 0; nComponent < static_cast<int>(NumberOfComponents); ++nComponent)
          {
            ErrorShadow << _Mixture.template getExponentialPartOfComponent<T>(nComponent, RawError),
                           sqrt(ceres::fmax(-2.0 * T(log(_Mixture.template getLinearPartOfComponent<T>(nComponent, RawError) / _Normalization)), T(1e-10)));/** fmax() is required to handle numeric tolerances */

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
          /** pass raw error trough */
          VectorRef<T, Dim> ErrorMap(Error);
          ErrorMap = RawError;

          /** set unused dimension to 0 */
          Error[Dim] = T(0);
        }

        return true;
      }

    private:

      void addMixture(const MixtureType &Mixture)
      {
        _Mixture = Mixture;

        _Normalization = _Mixture.getMaximumOfComponent(0);

        size_t NumberOfComponents = _Mixture.getNumberOfComponents();
        for(int nComponent = 1; nComponent < static_cast<int>(NumberOfComponents); ++nComponent)
        {
          _Normalization = std::max(_Normalization, _Mixture.getMaximumOfComponent(nComponent));
        }
      }

      MixtureType _Mixture;
      double _Normalization;

  };

  typedef MaxMixture<1, GaussianMixture<1>> MaxMix1;
  typedef MaxMixture<2, GaussianMixture<2>> MaxMix2;
  typedef MaxMixture<3, GaussianMixture<3>> MaxMix3;
  typedef MaxMixture<6, GaussianMixture<6>> MaxMix6;
}

#endif // MAXMIXTURE_H
