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
 * @file NormalizeAngle.h
 * @author Tim Pfeifer
 * @date 18.09.2018
 * @brief A simple normalization function for angles.
 * @copyright GNU Public License.
 *
 */

#ifndef NORMALIZE_ANGLE_H_
#define NORMALIZE_ANGLE_H_

#include "VectorMath.h"
#include "Messages.h"
#include <cmath>

namespace libRSF
{
  /** normalizes an Value between [-Limit Limit] */
  template <typename T>
  T NormalizeCustom(const T &Value, const double Limit)
  {
    T ValueWrapped = Value;
    while (ValueWrapped > Limit)
    {
      ValueWrapped -= 2*Limit;
    }
    while (ValueWrapped < -Limit)
    {
      ValueWrapped += 2*Limit;
    }

    return ValueWrapped;
  }

  template <typename T, int Dim>
  VectorT<T, Dim> NormalizeCustomVector(const VectorT<T, Dim> &Vector, const double Limit)
  {
    VectorT<T, Dim> VectorNormalized;
    for (int n = 0; n < Dim; n++)
    {
      VectorNormalized(n) = NormalizeCustom(Vector(n), Limit);
    }
    return VectorNormalized;
  }

  /** Normalizes the angle in radians between [-pi and pi]. */
  template <typename T>
  T NormalizeAngle(const T &Angle)
  {
    return NormalizeCustom(Angle, M_PI);
  }

  template <typename T, int Dim>
  VectorT<T, Dim>  NormalizeAngleVector(const VectorT<T, Dim> &Angle)
  {
    return NormalizeCustomVector(Angle, M_PI);
  }

  /** normalize turn rate  */
  template <typename T>
  T NormalizeAngleVelocity(const T &AngleVel, const double dt)
  {
    return NormalizeCustom(AngleVel, M_PI / dt);
  }

  template <typename T, int Dim>
  VectorT<T, Dim> NormalizeAngleVelocityVector(const VectorT<T, Dim> &AngleVel, const double dt)
  {
    return NormalizeCustomVector(AngleVel, M_PI / dt);
  }

}

#endif  // NORMALIZE_ANGLE_H_
