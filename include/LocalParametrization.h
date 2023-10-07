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
 * @file LocalParametrization.h
 * @author Tim Pfeifer
 * @date 18.09.2018
 * @brief Collection of local parametrizations for ceres
 * @copyright GNU Public License.
 *
 */

#ifndef LOCAL_PARAMETERIZATION_H_
#define LOCAL_PARAMETERIZATION_H_

#include "NormalizeAngle.h"
#include "VectorMath.h"
#include "Geometry.h"

#include <ceres/local_parameterization.h>

namespace libRSF
{
  /** from ceres examples */
  class AngleLocalParameterization
  {
    public:

      template <typename T>
      bool operator()(const T* Angle, const T* DeltaAngle, T* AnglePlusDelta) const
      {
        *AnglePlusDelta = NormalizeAngle(*Angle + *DeltaAngle);
        return true;
      }

      static ceres::LocalParameterization* Create()
      {
        return (new ceres::AutoDiffLocalParameterization<AngleLocalParameterization, 1, 1>);
      }
  };

  /** unit circle to represent angular value */
  class UnitCircleLocalParameterization
  {
    public:

      template <typename T>
      bool operator()(const T* Circle, const T* DeltaCircle, T* CirclePlusDelta) const
      {
        /** real part --> cos() */
        CirclePlusDelta[0] = Circle[0] * cos(DeltaCircle[0]) - Circle[1] * sin(DeltaCircle[0]);

        /** complex part --> sin() */
        CirclePlusDelta[1] = Circle[1] * cos(DeltaCircle[0]) + Circle[0] * sin(DeltaCircle[0]);

        return true;
      }

      static ceres::LocalParameterization* Create()
      {
        return (new ceres::AutoDiffLocalParameterization<UnitCircleLocalParameterization, 2, 1>);
      }
  };

  /** modified autodiff version of original ceres one */
  class QuaternionLocalParameterization
  {
    public:

      template <typename T>
      bool operator()(const T* Quaternion, const T* Delta, T* QuaternionPlusDelta) const
      {
        QuaternionRefConst<T> Q(Quaternion);
        VectorRefConst<T, 3> DeltaVec(Delta);
        QuaternionRef<T> QPlusDelta(QuaternionPlusDelta);

        const T Norm = DeltaVec.norm();
        if (Norm > T(1e-20))
        {
          /** using axis-angle definition */
          QPlusDelta = AngleAxisT<T> (Norm, DeltaVec / Norm) * Q;
        }
        else
        {
          /** use linear approximation for small angles (for stable derivative) */
          QuaternionT<T> QuatDelta;
          QuatDelta.x() = 0.5 * DeltaVec(0);
          QuatDelta.y() = 0.5 * DeltaVec(1);
          QuatDelta.z() = 0.5 * DeltaVec(2);
          QuatDelta.w() = T(1.0);

          QPlusDelta = QuatDelta * Q;
        }

        return true;
      }

      static ceres::LocalParameterization* Create()
      {
        return (new ceres::AutoDiffLocalParameterization<QuaternionLocalParameterization, 4, 3>);
      }
  };
}

#endif  // LOCAL_PARAMETERIZATION_H_
