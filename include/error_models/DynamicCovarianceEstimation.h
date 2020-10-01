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
 * @file DynamicCovarianceEstimation.h
 * @author Tim Pfeifer
 * @date 18.09.2018
 * @brief Error functions for the DCE algorithm.
 * @copyright GNU Public License.
 *
 */

#ifndef DYNAMICCOVARIANCEESTIMATION_H
#define DYNAMICCOVARIANCEESTIMATION_H

#include <Eigen/Dense>

#include "ErrorModel.h"

//TODO: adapt to sqrt information matrix (multidimensional!!!)

namespace libRSF
{
  template <int Dim, typename BaseModelType>
  class DynamicCovarianceEstimation : public ErrorModel <Dim, Dim+1, Dim>
  {
  public:

      DynamicCovarianceEstimation(){}

      explicit DynamicCovarianceEstimation(BaseModelType BaseModel): _BaseModel(BaseModel)
      {}


      template <typename T>
      bool Evaluate(const T* const SwitchVariable, T* Error) const
      {
//        /** evaluate with the non robust model */
//        _BaseModel.Evaluate(Error);
//
//        /** store error in matrix */
//        Eigen::Map<MatrixT<T, Dim+1, 1>> ErrorMap(Error);
//
//        /** apply switch variable */
//        ErrorMap.topLeftCorner(Dim,1).array() *=  SwitchVariable[0];
//
//        /** add switch prior */
//        ErrorMap(Dim,0) = (SwitchVariable[0] - 1.0) / T(_Sigma);

        return true;
      }

  private:
    BaseModelType _BaseModel;
  };



//  template <typename T> bool DCE(double const SigmaMin, const T* const Sigma, T* Error)
//  {
//    Error[0] = ceres::sqrt(ceres::log( Sigma[0] / T(SigmaMin - 1e-5)) * T(2.0));
//    return true;
//  }
//
//  template <typename T> bool DCE1(double const SigmaMin, const T* const Sigma, T* Error)
//  {
//    Error[0] /= Sigma[0];
//    return DCE(SigmaMin, Sigma, &Error[1]);
//  }
//
//  template <typename T> bool DCE2(double const SigmaMin, const T* const Sigma, T* Error)
//  {
//    Error[0] /= Sigma[0];
//    Error[1] /= Sigma[0];
//    return DCE(SigmaMin, Sigma, &Error[2]);
//  }
//
//  template <typename T> bool DCE3(double const SigmaMin, const T* const Sigma, T* Error)
//  {
//    Error[0] /= Sigma[0];
//    Error[1] /= Sigma[0];
//    Error[2] /= Sigma[0];
//    return DCE(SigmaMin, Sigma, &Error[3]);
//  }
}

#endif // DYNAMICCOVARIANCEESTIMATION_H
