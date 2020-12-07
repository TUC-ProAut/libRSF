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

#include "ErrorModel.h"
#include "../VectorMath.h"

#include <Eigen/Dense>

namespace libRSF
{
  template <int Dim>
  class GaussianDiagonal : public ErrorModel <Dim, Dim>
  {
  public:

      GaussianDiagonal() = default;

      void setStdDevSharedDiagonal(double StdDev)
      {
        _SqrtInformationDiagonal.fill(1.0/StdDev);
      }

      void setStdDevDiagonal(const VectorStatic<Dim> &StdDev)
      {
        _SqrtInformationDiagonal = StdDev.cwiseInverse();
      }

      void setCovarianceDiagonal(const VectorStatic<Dim> &Cov)
      {
        _SqrtInformationDiagonal = Cov.cwiseInverse().cwiseSqrt();
      }

      void setSqrtInformationDiagonal(const VectorStatic<Dim> &SqrtInfo)
      {
        _SqrtInformationDiagonal = SqrtInfo;
      }

      template <typename T>
      bool weight(const VectorT<T, Dim> &RawError, T* Error) const
      {
        /** wrap raw pointer to vector*/
        VectorRef<T, Dim> ErrorMap(Error);

        if(this->_Enable)
        {
          /** scale with diagonal information matrix */
          ErrorMap = RawError.cwiseProduct(_SqrtInformationDiagonal.template cast<T>());
        }
        else
        {
          /** pass-trough if error model is disabled*/
          ErrorMap = RawError;
        }

        return true;
      }

  private:
    /** square root information is more efficient to apply than covariance */
    VectorStatic<Dim> _SqrtInformationDiagonal;
  };

  template <int Dim>
  class GaussianFull : public ErrorModel <Dim, Dim>
  {
  public:

      GaussianFull() = default;

      void setCovarianceMatrix(const MatrixStatic<Dim, Dim> &CovMat)
      {
        _SqrtInformation = InverseSquareRoot<Dim, double>(CovMat);
      }

      void setSqrtInformationMatrix(const MatrixStatic<Dim, Dim> &SqrtInfoMat)
      {
        _SqrtInformation = SqrtInfoMat;
      }

      template <typename T>
      bool weight(const VectorT<T, Dim> &RawError, T* Error) const
      {
        /** wrap raw pointer to vector*/
        VectorRef<T, Dim> ErrorMap(Error);

        if(this->_Enable)
        {
          /** scale with full information matrix */
          ErrorMap = _SqrtInformation.template cast<T>() * RawError;
        }
        else
        {
          /** pass-trough if error model is disabled*/
          ErrorMap = RawError;
        }

        return true;
      }

  private:
      /** square root information is more efficient to apply than covariance */
      MatrixStatic<Dim, Dim> _SqrtInformation;
  };
}

#endif // GAUSSIAN_H
