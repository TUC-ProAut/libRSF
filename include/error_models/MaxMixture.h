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
        Normalization_ = std::numeric_limits<double>::lowest();
      }

      virtual ~MaxMixture() = default;

      explicit MaxMixture(const MixtureType &Mixture) : Normalization_(std::numeric_limits<double>::lowest())
      {
        this->addMixture_(Mixture);
      }

      void clear()
      {
        Normalization_ = std::numeric_limits<double>::lowest();
        Mixture_.clear();
      }

      template <typename T>
      bool weight(const VectorT<T, Dim> &RawError, T* Error) const
      {
        if(this->Enable_)
        {
          /** map the error pointer to a matrix */
          VectorRef<T, Dim+1> ErrorMap(Error);

          T Loglike = T(std::numeric_limits<double>::quiet_NaN());

          MatrixT<T, Dim + 1, 1> ErrorShadow, ErrorShadowBest;

          const int NumberOfComponents = Mixture_.getNumberOfComponents();

          /** calculate Log-Likelihood for each Gaussian component */
          for(int nComponent = 0; nComponent < NumberOfComponents; ++nComponent)
          {
            ErrorShadow << Mixture_.template getExponentialPartOfComponent<T>(nComponent, RawError),
                           sqrt(ceres::fmax(-2.0 * T(log(Mixture_.template getLinearPartOfComponent<T>(nComponent, RawError) / Normalization_)), T(1e-10)));/** fmax() is required to handle numeric tolerances */

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
          Error[Dim] = T(0.0);
        }

        return true;
      }

    private:

      void addMixture_(const MixtureType &Mixture)
      {
        Mixture_ = Mixture;
        const int NumberOfComponents = Mixture.getNumberOfComponents();

        if (NumberOfComponents > 0)
        {
          Normalization_ = Mixture.getMaximumOfComponent(0);
          for(int nComponent = 1; nComponent < NumberOfComponents; ++nComponent)
          {
            Normalization_ = std::max(Normalization_, Mixture.getMaximumOfComponent(nComponent));
          }
        }
      }

      MixtureType Mixture_;
      double Normalization_;
  };

  template <int Dim>
  using MaxMix = MaxMixture<Dim, GaussianMixture<Dim>>;

  using MaxMix1 = MaxMixture<1, GaussianMixture<1>>;
  using MaxMix2 = MaxMixture<2, GaussianMixture<2>>;
  using MaxMix3 = MaxMixture<3, GaussianMixture<3>>;
  using MaxMix5 = MaxMixture<5, GaussianMixture<5>>;
  using MaxMix6 = MaxMixture<6, GaussianMixture<6>>;
  using MaxMix8 = MaxMixture<8, GaussianMixture<8>>;
  using MaxMix9 = MaxMixture<9, GaussianMixture<9>>;
  using MaxMix11 = MaxMixture<11, GaussianMixture<11>>;
}

#endif // MAXMIXTURE_H
