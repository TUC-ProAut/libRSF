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
 * @file NormalizeAngle.h
 * @author Tim Pfeifer
 * @date 18.09.2018
 * @brief A simple normalization function for angles.
 * @copyright GNU Public License.
 *
 */

#ifndef NORMALIZE_ANGLE_H_
#define NORMALIZE_ANGLE_H_

#include <cmath>
#include "ceres/ceres.h"

namespace libRSF
{

/** Normalizes the angle in radians between [-pi and pi]. */
template <typename T>
inline T NormalizeAngle(const T& Angle)
{
  T TwoPi(2.0 * M_PI);
  return Angle - TwoPi * ceres::floor((Angle + M_PI) / TwoPi);
}

}

#endif  // NORMALIZE_ANGLE_H_
