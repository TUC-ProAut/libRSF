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
        SqrtInformationDiagonal_.fill(1.0/StdDev);
      }

      void setStdDevDiagonal(const VectorStatic<Dim> &StdDev)
      {
        SqrtInformationDiagonal_ = StdDev.cwiseInverse();
      }

      void setCovarianceDiagonal(const VectorStatic<Dim> &Cov)
      {
        SqrtInformationDiagonal_ = Cov.cwiseInverse().cwiseSqrt();
      }

      void setSqrtInformationDiagonal(const VectorStatic<Dim> &SqrtInfo)
      {
        SqrtInformationDiagonal_ = SqrtInfo;
      }

      template <typename T>
      bool weight(const VectorT<T, Dim> &RawError, T* Error) const
      {
        /** wrap raw pointer to vector*/
        VectorRef<T, Dim> ErrorMap(Error);

        if(this->Enable_)
        {
          /** scale with diagonal information matrix */
          ErrorMap = RawError.cwiseProduct(SqrtInformationDiagonal_.template cast<T>());
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
    VectorStatic<Dim> SqrtInformationDiagonal_;
  };

  template <int Dim>
  class GaussianFull : public ErrorModel <Dim, Dim>
  {
  public:

      GaussianFull() = default;

      void setCovarianceMatrix(const MatrixStatic<Dim, Dim> &CovMat)
      {
        SqrtInformation_ = InverseSquareRoot<Dim, double>(CovMat);
      }

      void setSqrtInformationMatrix(const MatrixStatic<Dim, Dim> &SqrtInfoMat)
      {
        SqrtInformation_ = SqrtInfoMat;
      }

      template <typename T>
      bool weight(const VectorT<T, Dim> &RawError, T* Error) const
      {
        /** wrap raw pointer to vector*/
        VectorRef<T, Dim> ErrorMap(Error);

        if(this->Enable_)
        {
          /** scale with full information matrix */
          ErrorMap = SqrtInformation_.template cast<T>() * RawError;
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
      MatrixStatic<Dim, Dim> SqrtInformation_;
  };
}

#endif // GAUSSIAN_H
