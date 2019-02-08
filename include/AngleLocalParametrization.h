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
 * @file AngleLocalParametrization.h
 * @author Tim Pfeifer
 * @date 18.09.2018
 * @brief A simple local parametrization for angles in radiant.
 * @copyright GNU Public License.
 *
 */

#ifndef ANGLE_LOCAL_PARAMETERIZATION_H_
#define ANGLE_LOCAL_PARAMETERIZATION_H_

#include <ceres/local_parameterization.h>
#include "NormalizeAngle.h"

namespace libRSF
{
  class AngleLocalParameterization
  {
    public:

      template <typename T>
      bool operator()(const T* Angle, const T* DeltaAngle, T* AnglePlusDelta) const
      {
        *AnglePlusDelta = libRSF::NormalizeAngle(*Angle + *DeltaAngle);
        return true;
      }

      static ceres::LocalParameterization* Create()
      {
        return (new ceres::AutoDiffLocalParameterization<AngleLocalParameterization, 1, 1>);
      }
  };
}

#endif  // ANGLE_LOCAL_PARAMETERIZATION_H_
