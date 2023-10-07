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
 * @file MaxSumMixture.h
 * @author Tim Pfeifer
 * @date 07.01.2020
 * @brief A vectorized, optimizer-friendly Gaussian mixture model.
 * @copyright GNU Public License.
 *
 */

#ifndef MAXSUMMIXTURE_H
#define MAXSUMMIXTURE_H

#include "ErrorModel.h"
#include "GaussianMixture.h"
#include "../NumericalRobust.h"

namespace libRSF
{
  extern const double DampingFactor;

  /** \brief The robust Max-Sum-Mixture error model
   *
   * \param Mixture Underlying mixture distribution
   *
   */
  template <int Dim, typename MixtureType>
  class MaxSumMixture : public ErrorModel <Dim, Dim+1>
  {
    public:

      MaxSumMixture()
      {
        this->clear();
      }

      virtual ~MaxSumMixture() = default;

      explicit MaxSumMixture(const MixtureType &Mixture)
      {
        this->addMixture_(Mixture);
      }

      void clear()
      {
        Normalization_ = 0;
        Mixture_.clear();
      }

      template <typename T>
      bool weight(const VectorT<T, Dim> &RawError, T* Error) const
      {
        if(this->Enable_)
        {
          /** calculate linear errors and scalings */
          const int NumberOfComponents = Mixture_.getNumberOfComponents();
          MatrixT<T, Dynamic, 1> Scalings(NumberOfComponents);
          MatrixT<T, Dynamic, Dim> LinearExponents(NumberOfComponents, Dim);

          /** calculate component-wise */
          for(int nComponent = 0; nComponent < NumberOfComponents; ++nComponent)
          {
            LinearExponents.row(nComponent) = Mixture_.template getExponentialPartOfComponent<T>(nComponent, RawError);
            Scalings(nComponent) = T(Mixture_.template getLinearPartOfComponent<T>(nComponent, RawError));
          }

          /** apply the LSE */
          const VectorT<T, Dim+1> LSE = VectorizedLogSumExp(LinearExponents, Scalings);

          /** map the error pointer to a matrix */
          VectorRef<T, Dim+1> ErrorMap(Error);

          ErrorMap.template head<Dim>() = LSE.template head<Dim>(); /**< linear */

          /** nonlinear part is not required if there is only one component */
          if (NumberOfComponents > 1)
          {
            /** to prevent numerical issues close to zero, we set a lower bound of the following term */
            ErrorMap(Dim) = sqrt(ceres::fmax(-2.0 * (LSE(Dim) - log(Normalization_ + DampingFactor)), T(1e-20))); /**< non-linear */
          }
          else
          {
            ErrorMap(Dim) = T(0.0); /**< cancel non-linear */
          }

          /** catch bad numerical cases (extreme covariances in a unimportant component) */
          if (ceres::isfinite(ErrorMap(Dim)) == false)
          {
            ErrorMap(Dim) = T(0.0); /**< cancel non-linear */
          }
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

      MixtureType getMixture()
      {
        return Mixture_;
      }

    private:

      void addMixture_(const MixtureType &Mixture)
      {
        Mixture_ = Mixture;
        const int NumberOfComponents = Mixture.getNumberOfComponents();

        Normalization_ = Mixture.getMaximumOfComponent(0);
        for(int nComponent = 1; nComponent < static_cast<int>(NumberOfComponents); ++nComponent)
        {
          Normalization_ = std::max(Normalization_, Mixture.getMaximumOfComponent(nComponent));
        }
        Normalization_ *= NumberOfComponents;
      }

      MixtureType Mixture_;
      double Normalization_{0};
  };

  template <int Dim>
  using MaxSumMix = MaxSumMixture<Dim, GaussianMixture<Dim>>;

  using MaxSumMix1 = MaxSumMixture<1, GaussianMixture<1>>;
  using MaxSumMix2 = MaxSumMixture<2, GaussianMixture<2>>;
  using MaxSumMix3 = MaxSumMixture<3, GaussianMixture<3>>;
}

#endif // MAXSUMMIXTURE_H
