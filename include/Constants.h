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
 * @file Constants.h
 * @author Tim Pfeifer
 * @date 29.03.2019
 * @brief Central place for all constant values;
 * @copyright GNU Public License.
 *
 */

#ifndef CONSTANTS_H
#define CONSTANTS_H

#include "VectorMath.h"

#include <limits>

namespace libRSF
{
  /** physical constants*/
  #define EARTH_ROTATION_RATE   7.2921151467e-5
  #define SPEED_OF_LIGHT        2.99792458e8
  #define GRAVITY               9.81

  static const Vector3 GRAVITY_VECTOR {0.0, 0.0, GRAVITY};

  /** numerical constants */
  #define NAN_DOUBLE           std::numeric_limits<double>::signaling_NaN()
}

#endif // CONSTANTS_H
