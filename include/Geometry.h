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
 * @file Geometry.h
 * @author Tim Pfeifer
 * @date 18.09.2018
 * @brief Simple geometric helper functions.
 * @copyright GNU Public License.
 *
 */

#ifndef GEOMETRY_H
#define GEOMETRY_H


#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Dense>
#include "NormalizeAngle.h"

namespace libRSF
{
  template <typename T>
  Eigen::Matrix<T, 2, 2> RotationMatrix2D(T Yaw)
  {
    const T CosYaw = ceres::cos(Yaw);
    const T SinYaw = ceres::sin(Yaw);

    Eigen::Matrix<T, 2, 2> RotationMatrix;
    RotationMatrix << CosYaw, -SinYaw,
                      SinYaw,  CosYaw;

    return RotationMatrix;
  }

  template <typename T>
  Eigen::Matrix<T, 3, 1> RelativeMotion2D(const T* PointOld, const T* PointNew, const T* YawOld, const T* const YawNew)
  {
    const Eigen::Matrix<T, 2, 1> POld(PointOld[0], PointOld[1]);
    const Eigen::Matrix<T, 2, 1> PNew(PointNew[0], PointNew[1]);

    Eigen::Matrix<T, 3, 1> RelativeMotion;

    RelativeMotion.head(2) = RotationMatrix2D(YawOld[0]).transpose() * (PNew - POld);
    RelativeMotion[2] = NormalizeAngle(YawNew[0] - YawOld[0]);

    return RelativeMotion;
  }
  template <int Dimension, typename T1, typename T2>
  void VectorDifference(const T1* const Vector1, const T2* const Vector2,  T1* Difference)
  {
    for(int nDim = 0; nDim < Dimension; nDim++)
    {
      Difference[nDim] = Vector1[nDim] - Vector2[nDim];
    }
  }

  template <int Dimension, typename T>
  T VectorLength(const T* const Vector)
  {
    T SquaredSum = T(0.0);

    for(int nDim = 0; nDim < Dimension; nDim++)
    {
      SquaredSum += ceres::pow(Vector[nDim], 2);
    }

    /** for stability of the derivation */
    if(SquaredSum < T(1e-10))
      SquaredSum += T(1e-10);

    return ceres::sqrt(SquaredSum);
  }

  template <int Dimension, typename T1, typename T2>
  T1 VectorDistance(const T1* const Vector1, const T2* const Vector2)
  {
    T1 Difference[Dimension];
    VectorDifference<Dimension, T1, T2>(Vector1, Vector2, Difference);
    return T1(VectorLength<Dimension, T1>(Difference));
  }

}

#endif // GEOMETRY_H
