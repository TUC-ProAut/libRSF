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
 * @file QuaternionCalc.h
 * @author Tim Pfeifer
 * @date 12.12.2016
 * @brief Collection of factors that connect two poses with an odometry measurement.
 * @copyright GNU Public License.
 *
 */

#ifndef QUATERNIONCALC_H
#define QUATERNIONCALC_H
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "Geometry.h"
#include "VectorMath.h"

namespace libRSF
{
  /** @brief invert a quaternion
   *
   * @param Quaternion    ceres quaternion with ordering w,x,y,z
   * @param QuaternionInv inverted quaternion
   */
  template<typename T>
  inline void InvertQuaterion(const T* Quaternion, T* QuaternionInv);

  template<typename T>
  inline T RotationBetween(const T* Vector1, const T* Vector2, T* Quaternion);

  template<typename T>
  inline void InvertQuaterion(const T* Quaternion, T* QuaternionInv)
  {
    /** invert quaternion */
    const T &QuaternionAbs = Quaternion[0] * Quaternion[0] + Quaternion[1] * Quaternion[1] + Quaternion[2] * Quaternion[2] + Quaternion[3] * Quaternion[3];

    QuaternionInv[0] = Quaternion[0] / QuaternionAbs;
    QuaternionInv[1] = -Quaternion[1] / QuaternionAbs;
    QuaternionInv[2] = -Quaternion[2] / QuaternionAbs;
    QuaternionInv[3] = -Quaternion[3] / QuaternionAbs;
  }

  template<typename T>
  inline T Norm3D(const T* Vector)
  {
    return ceres::sqrt(ceres::DotProduct(Vector, Vector));
  }

  template<typename T>
  inline T NormalizeQuaternion(T* Quaternion)
  {
    const T Norm = VectorLength<4, T>(Quaternion);
    Quaternion[0] /= Norm;
    Quaternion[1] /= Norm;
    Quaternion[2] /= Norm;
    Quaternion[3] /= Norm;
  }

  template<typename T>
  inline T RotationBetween(const T* Vector1, const T* Vector2, T* Quaternion)
  {
    T Axis[3];
    ceres::CrossProduct(Vector1, Vector2, Axis);

    Quaternion[0] = Norm3D(Vector1) * Norm3D(Vector2) + ceres::DotProduct(Vector1, Vector2);
    Quaternion[1] = Axis[0];
    Quaternion[2] = Axis[1];
    Quaternion[3] = Axis[2];

    /** catch equal vectors */
    if(Quaternion[0] < T(1e-10))
    {
      Quaternion[0] += 1;
    }

    NormalizeQuaternion(Quaternion);
  }
}

#endif // QUATERNIONCALC_H
