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
 * @file SumMixture.h
 * @author Tim Pfeifer
 * @date 18.09.2018
 * @brief Sum-Mixture error model inspired from the work of Rosen.
 * @copyright GNU Public License.
 *
 */

#ifndef SUMMIXTURE_H
#define SUMMIXTURE_H

#include "ErrorModel.h"
#include "GaussianMixture.h"
#include "../NumericalRobust.h"

namespace libRSF
{
  /** \brief The robust Sum-Mixture error model
   * Based on:
   * D. M. Rosen, M. Kaess, and J. J. Leonard
   * “Robust incremental online inference over sparse factor graphs: Beyond the Gaussian case”
   * Proc. of Intl. Conf. on Robotics and Automation (ICRA), Karlsruhe, 2013.
   * DOI: 10.1109/ICRA.2013.6630699
   *
   * \param Mixture Underlying mixture distribution
   *
   */
  template <int Dim, typename MixtureType, bool SpecialNormalization>
  class SumMixture : public ErrorModel <Dim, Dim>
  {
  public:
    SumMixture()
    {
      this->clear();
    }

    virtual ~SumMixture() = default;

    explicit SumMixture(const MixtureType &Mixture): Normalization_(0)
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
      /** map error to eigen matrix for easier access */
      VectorRef<T, Dim> ErrorMap(Error);

      if(this->Enable_)
      {
        const int NumberOfComponents = Mixture_.getNumberOfComponents();

        MatrixT<T, Dynamic, 1> Scalings(NumberOfComponents);
        MatrixT<T, Dynamic, 1> Exponents(NumberOfComponents);

        /** calculate all exponents and scalings */
        for(int nComponent = 0; nComponent < NumberOfComponents; ++nComponent)
        {
          Exponents(nComponent) = - 0.5 * (Mixture_.template getExponentialPartOfComponent<T>(nComponent, RawError).squaredNorm() + 1e-10);
          Scalings(nComponent) = T(Mixture_.template getLinearPartOfComponent<T>(nComponent, RawError));
        }

        /** combine them numerically robust and distribute the error equally over all dimensions */
        ErrorMap.fill(sqrt(-2.0* (ScaledLogSumExp(Exponents, Scalings) - log(Normalization_ + 1e-10))) / sqrt(Dim));
      }
      else
      {
        /** pass raw error trough */
        ErrorMap = RawError;
      }

      return true;
    }

  private:

    void addMixture_(const MixtureType &Mixture)
    {
      Mixture_ = Mixture;

      const int NumberOfComponents = Mixture.getNumberOfComponents();

      if constexpr(!SpecialNormalization)
      {
        /** original version */
        Normalization_ = 0;
        for(int nComponent = 0; nComponent < NumberOfComponents; ++nComponent)
        {
          Normalization_ += Mixture.getMaximumOfComponent(nComponent);
        }
      }
      else
      {
        /** version for Reviewer 3 */
        Normalization_ = Mixture.getMaximumOfComponent(0);
        for(int nComponent = 1; nComponent < NumberOfComponents; ++nComponent)
        {
          Normalization_ = std::max(Normalization_, Mixture.getMaximumOfComponent(nComponent));
        }
        Normalization_ = Normalization_ *NumberOfComponents + 10;
      }
    }

    MixtureType Mixture_;
    double Normalization_{0};
  };

  template <int Dim>
  using SumMix = SumMixture<Dim, GaussianMixture<Dim>, false>;

  using SumMix1 = SumMixture<1, GaussianMixture<1>, false>;
  using SumMix2 = SumMixture<2, GaussianMixture<2>, false>;
  using SumMix3 = SumMixture<3, GaussianMixture<3>, false>;

  using SumMix1Special = SumMixture<1, GaussianMixture<1>, true>;
  using SumMix2Special = SumMixture<2, GaussianMixture<2>, true>;
  using SumMix3Special = SumMixture<3, GaussianMixture<3>, true>;
}

#endif // SUMMIXTURE_H
