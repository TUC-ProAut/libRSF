/***************************************************************************
 * libRSF - A Robust Sensor Fusion Libary
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

#ifndef ANGLE_LOCAL_PARAMETERIZATION_H_
#define ANGLE_LOCAL_PARAMETERIZATION_H_

#include <ceres/local_parameterization.h>
#include "NormalizeAngle.h"

namespace libisf2
{


// Defines a local parameterization for updating the angle to be constrained in
// [-pi to pi).
  class AngleLocalParameterization
  {
    public:

      template <typename T>
      bool operator()(const T* theta_radians, const T* delta_theta_radians,
                      T* theta_radians_plus_delta) const
      {
        *theta_radians_plus_delta =
          libisf2::NormalizeAngle(*theta_radians + *delta_theta_radians);

        return true;
      }

      static ceres::LocalParameterization* Create()
      {
        return (new ceres::AutoDiffLocalParameterization<AngleLocalParameterization, 1, 1>);
      }
  };

}

#endif  // ANGLE_LOCAL_PARAMETERIZATION_H_
