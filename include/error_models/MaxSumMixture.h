/***************************************************************************
 * libRSF - A Robust Sensor Fusion Library
 *
 * Copyright (C) 2020 Chair of Automation Technology / TU Chemnitz
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
        this->addMixture(Mixture);
      }

      void clear()
      {
        _Normalization = 0;
        _Mixture.clear();
      }

      template <typename T>
      bool weight(const VectorT<T, Dim> &RawError, T* Error) const
      {
        if(this->_Enable)
        {
          /** calculate linear errors and scalings */
          const int NumberOfComponents = _Mixture.getNumberOfComponents();
          MatrixT<T, Dynamic, 1> Scalings(NumberOfComponents);
          MatrixT<T, Dynamic, Dim> LinearExponents(NumberOfComponents, Dim);

          /** calculate component-wise */
          for(int nComponent = 0; nComponent < NumberOfComponents; ++nComponent)
          {
            LinearExponents.row(nComponent) = _Mixture.template getExponentialPartOfComponent<T>(nComponent, RawError);
            Scalings(nComponent) = T(_Mixture.template getLinearPartOfComponent<T>(nComponent, RawError));
          }

          /** apply the LSE */
          const VectorT<T, Dim+1> LSE = VectorizedLogSumExp(LinearExponents, Scalings);

          /** map the error pointer to a matrix */
          VectorRef<T, Dim+1> ErrorMap(Error);

          ErrorMap.template head<Dim>() = LSE.template head<Dim>(); /**< linear */
          /** to prevent numerical issues close to zero, we set a lower bound of the following term */
          ErrorMap(Dim) = sqrt(ceres::fmax(-2.0 * (LSE(Dim) - log(_Normalization + DampingFactor)), T(1e-20))); /**< non-linear */
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
        return _Mixture;
      }

    private:

      void addMixture(const MixtureType &Mixture)
      {
        _Mixture = Mixture;
        const int NumberOfComponents = _Mixture.getNumberOfComponents();

        _Normalization = _Mixture.getMaximumOfComponent(0);
        for(int nComponent = 1; nComponent < static_cast<int>(NumberOfComponents); ++nComponent)
        {
          _Normalization = std::max(_Normalization, _Mixture.getMaximumOfComponent(nComponent));
        }
        _Normalization *= NumberOfComponents;
      }

      MixtureType _Mixture;
      double _Normalization;
  };

  typedef MaxSumMixture<1, GaussianMixture<1>> MaxSumMix1;
  typedef MaxSumMixture<2, GaussianMixture<2>> MaxSumMix2;
  typedef MaxSumMixture<3, GaussianMixture<3>> MaxSumMix3;
}

#endif // MAXSUMMIXTURE_H
