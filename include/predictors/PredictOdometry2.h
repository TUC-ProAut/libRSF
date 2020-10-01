/***************************************************************************
 * libRSF - A Robust Sensor Fusion Library
 *
 * Copyright (C) 2019 Chair of Automation Technology / TU Chemnitz
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
 * @file PredictOdometry2.h
 * @author Tim Pfeifer
 * @date 04.09.2019
 * @brief Predictors for 2D odometry models.
 * @copyright GNU Public License.
 *
 */

#ifndef PREDICTODOMETRY2_H
#define PREDICTODOMETRY2_H

#include "../Geometry.h"

namespace libRSF
{
  template<typename T>
  void PredictOdometry2 (const T* PosOld, T* PosNew, const T* RotOld, T* RotNew, const Vector3& Odometry, double DeltaTime)
  {
    /** map pointer to vectors */
    VectorRefConst<T, 2> P1(PosOld);
    VectorRef<T, 2>      P2(PosNew);

    /** map to quaternions */
    const Rotation2DT<T>      R1(RotOld[0]);
    const Rotation2DT<double> ROdom(Odometry(2)*DeltaTime);

    P2 = P1 + R1 * Odometry.head(2) * DeltaTime;

    RotNew[0] = (ROdom * R1).angle();
  }
}


#endif // PREDICTODOMETRY2_H
