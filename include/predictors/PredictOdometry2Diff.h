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
 * @file PredictOdometry2DDifferential.h
 * @author Tim Pfeifer
 * @date 18.09.2018
 * @brief Predictor for a 2D differential drive odometry model.
 * @copyright GNU Public License.
 *
 */

#ifndef PREDICTODOMETRY2DDIFFERENTIAL_H
#define PREDICTODOMETRY2DDIFFERENTIAL_H

#include "../Geometry.h"

namespace libRSF
{
  template<typename T>
  void PredictMotionDifferential2D (const T* PoseOld, T* PoseNew, const T* YawOld, T* YawNew, const Vector2& Odometry, double WheelBase, double DeltaTime)
  {
    VectorT<T, 2> PoseNewTemp;

    PoseNewTemp << (Odometry[0] + Odometry[1]) / 2, 0;
    PoseNewTemp = RotationMatrix2D(YawOld[0]) * PoseNewTemp;
    PoseNewTemp *= DeltaTime;

    PoseNew[0] = PoseNewTemp[0] + PoseOld[0];
    PoseNew[1] = PoseNewTemp[1] + PoseOld[1];

    YawNew[0] = NormalizeAngle(YawOld[0] + (Odometry[1] - Odometry[0]) / (WheelBase * 2.0) * DeltaTime);
  }
}


#endif // PREDICTODOMETRY2DDIFFERENTIAL_H
